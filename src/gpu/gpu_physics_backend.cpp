#ifdef CAMPELLO_PHYSICS_GPU_ENABLED

#include "gpu_physics_backend.h"
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_gpu/descriptors/pipeline_layout_descriptor.hpp>
#include <campello_gpu/descriptors/bind_group_layout_descriptor.hpp>
#include <campello_gpu/descriptors/bind_group_descriptor.hpp>
#include <campello_gpu/descriptors/compute_pipeline_descriptor.hpp>
#include <campello_gpu/constants/shader_stage.hpp>
#include <campello_gpu/constants/buffer_usage.hpp>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

// ── Embedded shader bytecode (pre-compiled, committed to repo) ────────────────
#if defined(CAMPELLO_PLATFORM_MACOS) || defined(CAMPELLO_PLATFORM_IOS)
#  include "../shaders/metal/broadphase_aabb.h"
#  include "../shaders/metal/broadphase_pairs.h"
#  include "../shaders/metal/xpbd_contacts.h"
#  include "../shaders/metal/xpbd_finalize.h"
#  include "../shaders/metal/xpbd_predict.h"
#  include "../shaders/metal/write_render_transforms.h"
#elif defined(CAMPELLO_PLATFORM_LINUX) || defined(CAMPELLO_PLATFORM_ANDROID)
#  include "../shaders/glsl/broadphase_aabb.h"
#  include "../shaders/glsl/broadphase_pairs.h"
#  include "../shaders/glsl/xpbd_contacts.h"
#  include "../shaders/glsl/xpbd_finalize.h"
#  include "../shaders/glsl/xpbd_predict.h"
#  include "../shaders/glsl/write_render_transforms.h"
#elif defined(CAMPELLO_PLATFORM_WINDOWS)
#  include "../shaders/hlsl/broadphase_aabb.h"
#  include "../shaders/hlsl/broadphase_pairs.h"
#  include "../shaders/hlsl/xpbd_contacts.h"
#  include "../shaders/hlsl/xpbd_finalize.h"
#  include "../shaders/hlsl/xpbd_predict.h"
#  include "../shaders/hlsl/write_render_transforms.h"
#endif

namespace campello::physics {

// ── Platform helpers ──────────────────────────────────────────────────────────

static cg::BufferUsage operator|(cg::BufferUsage a, cg::BufferUsage b) {
    return static_cast<cg::BufferUsage>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

static std::string shaderEntryPoint(const std::string& kernelName) {
#if defined(CAMPELLO_PLATFORM_LINUX) || defined(CAMPELLO_PLATFORM_ANDROID)
    (void)kernelName;
    return "main";  // GLSL entry point is always "main"
#else
    return kernelName;
#endif
}

// ── Shader loading from embedded bytecode ────────────────────────────────────

struct EmbeddedShader { const uint8_t* data; uint32_t size; };

static EmbeddedShader embeddedShader(const std::string& kernelName) {
    if (kernelName == "broadphase_aabb")          return { kShader_broadphase_aabb,          kShader_broadphase_aabb_size          };
    if (kernelName == "broadphase_pairs")         return { kShader_broadphase_pairs,         kShader_broadphase_pairs_size         };
    if (kernelName == "xpbd_contacts")            return { kShader_xpbd_contacts,            kShader_xpbd_contacts_size            };
    if (kernelName == "xpbd_finalize")            return { kShader_xpbd_finalize,            kShader_xpbd_finalize_size            };
    if (kernelName == "xpbd_predict")             return { kShader_xpbd_predict,             kShader_xpbd_predict_size             };
    if (kernelName == "write_render_transforms")  return { kShader_write_render_transforms,  kShader_write_render_transforms_size  };
    return { nullptr, 0 };
}

std::shared_ptr<cg::ShaderModule> GpuPhysicsBackend::loadShaderFile(const std::string& kernelName) {
    auto [data, size] = embeddedShader(kernelName);
    if (!data || size == 0) return nullptr;
    return m_device->createShaderModule(data, static_cast<uint64_t>(size));
}

// ── BindGroupLayout helpers ───────────────────────────────────────────────────

std::shared_ptr<cg::BindGroupLayout> GpuPhysicsBackend::makeStorageBGL(
    const std::vector<std::pair<uint32_t,bool>>& storageBindings,
    uint32_t uniformSlot)
{
    cg::BindGroupLayoutDescriptor desc;
    for (auto& [slot, readOnly] : storageBindings) {
        cg::EntryObject e{};
        e.binding    = slot;
        e.visibility = cg::ShaderStage::compute;
        e.type       = cg::EntryObjectType::buffer;
        e.data.buffer.hasDinamicOffaset = false;
        e.data.buffer.minBindingSize    = 0;
        e.data.buffer.type = readOnly
            ? cg::EntryObjectBufferType::readOnlyStorage
            : cg::EntryObjectBufferType::storage;
        desc.entries.push_back(e);
    }
    if (uniformSlot != ~0u) {
        cg::EntryObject e{};
        e.binding    = uniformSlot;
        e.visibility = cg::ShaderStage::compute;
        e.type       = cg::EntryObjectType::buffer;
        e.data.buffer.hasDinamicOffaset = false;
        e.data.buffer.minBindingSize    = 0;
        e.data.buffer.type              = cg::EntryObjectBufferType::uniform;
        desc.entries.push_back(e);
    }
    return m_device->createBindGroupLayout(desc);
}

// ── Pipeline creation ─────────────────────────────────────────────────────────

std::shared_ptr<cg::ComputePipeline> GpuPhysicsBackend::buildKernelPipeline(
    const std::string& kernelName,
    const std::shared_ptr<cg::BindGroupLayout>& bgl)
{
    auto shaderMod = loadShaderFile(kernelName);
    if (!shaderMod) return nullptr;

    auto layout = m_device->createPipelineLayout(cg::PipelineLayoutDescriptor{});

    cg::ComputePipelineDescriptor desc{};
    desc.compute.module     = shaderMod;
    desc.compute.entryPoint = shaderEntryPoint(kernelName);
    desc.layout             = layout;
    (void)bgl;  // layout inferred from shader; bgl used only for bind group creation
    return m_device->createComputePipeline(desc);
}

// ── initialize ────────────────────────────────────────────────────────────────

bool GpuPhysicsBackend::initialize(std::shared_ptr<void> device) {
    if (device)
        m_device = std::static_pointer_cast<cg::Device>(device);
    else
        m_device = cg::Device::createDefaultDevice(nullptr);
    if (!m_device) return false;
    return loadPipelines();
}

bool GpuPhysicsBackend::loadPipelines() {
    // broadphase_aabb: b0=state(ro), b1=shape(rw), b2=params(uniform)
    m_aabbBGL = makeStorageBGL({{0,true},{1,false}}, 2);

    // broadphase_pairs: b0=shape(ro), b1=pairs(rw), b2=params(uniform)
    m_pairsBGL = makeStorageBGL({{0,true},{1,false}}, 2);

    // xpbd_predict: b0=state(rw), b1=params(uniform)
    m_predictBGL = makeStorageBGL({{0,false}}, 1);

    // xpbd_contacts: b0=state(rw), b1=shape(ro), b2=pairs(ro), b3=params(uniform)
    m_contactsBGL = makeStorageBGL({{0,false},{1,true},{2,true}}, 3);

    // xpbd_finalize: b0=state(rw), b1=params(uniform)
    m_finalizeBGL = makeStorageBGL({{0,false}}, 1);

    // write_render_transforms: b0=state(ro), b1=indexMap(ro), b2=render(rw), b3=params(uniform)
    m_renderWriteBGL = makeStorageBGL({{0,true},{1,true},{2,false}}, 3);

    m_aabbPipeline        = buildKernelPipeline("broadphase_aabb",         m_aabbBGL);
    m_pairsPipeline       = buildKernelPipeline("broadphase_pairs",        m_pairsBGL);
    m_predictPipeline     = buildKernelPipeline("xpbd_predict",            m_predictBGL);
    m_contactsPipeline    = buildKernelPipeline("xpbd_contacts",           m_contactsBGL);
    m_finalizePipeline    = buildKernelPipeline("xpbd_finalize",           m_finalizeBGL);
    m_renderWritePipeline = buildKernelPipeline("write_render_transforms", m_renderWriteBGL);

    return m_aabbPipeline && m_pairsPipeline && m_predictPipeline
        && m_contactsPipeline && m_finalizePipeline && m_renderWritePipeline;
}

// ── Capacity management ───────────────────────────────────────────────────────

void GpuPhysicsBackend::ensureCapacity(uint32_t bodyCount) {
    if (bodyCount <= m_capacity) return;
    m_capacity = bodyCount + 64;  // grow with headroom

    // Cap pairs: each body has at most 32 neighbors; also cap at 65536
    m_maxPairs = std::min(m_capacity * 32u, 65536u);
    uint64_t pairsBufSize =
        (kPairsHeaderUints + static_cast<uint64_t>(m_maxPairs) * 2) * sizeof(uint32_t);

    auto storageBuf = [&](uint64_t size) {
        return m_device->createBuffer(size,
            cg::BufferUsage::storage |
            cg::BufferUsage::copyDst |
            cg::BufferUsage::copySrc);
    };

    m_stateBuffer  = storageBuf(m_capacity * sizeof(GpuBodyState));
    m_shapeBuffer  = storageBuf(m_capacity * sizeof(GpuBodyShape));
    m_paramsBuffer = m_device->createBuffer(sizeof(GpuShaderParams), cg::BufferUsage::uniform);
    m_pairsBuffer  = storageBuf(pairsBufSize);

    // Index map: gpuIdx → poolSlot, uploaded from CPU each step
    m_indexMapBuffer = m_device->createBuffer(
        m_capacity * sizeof(uint32_t),
        cg::BufferUsage::storage | cg::BufferUsage::copyDst);

    // Staging buffer for CPU readback: Shared storage mode (CPU-visible) on all platforms
    m_stateStagingBuffer = m_device->createBuffer(
        m_capacity * sizeof(GpuBodyState),
        cg::BufferUsage::copyDst | cg::BufferUsage::mapRead);

    rebuildBindGroups();
}

void GpuPhysicsBackend::ensureRenderCapacity(uint32_t poolCapacity) {
    if (poolCapacity <= m_renderCapacity) return;
    m_renderCapacity = poolCapacity + 64;
    m_renderBuffer = m_device->createBuffer(
        m_renderCapacity * sizeof(GpuRenderTransform),
        cg::BufferUsage::storage | cg::BufferUsage::copySrc);
}

static cg::BindGroupEntryDescriptor bufEntry(uint32_t slot,
    const std::shared_ptr<cg::Buffer>& buf, uint64_t size)
{
    cg::BindGroupEntryDescriptor e{};
    e.binding  = slot;
    e.resource = cg::BufferBinding{buf, 0, size};
    return e;
}

void GpuPhysicsBackend::rebuildBindGroups() {
    uint64_t stateSz  = m_capacity * sizeof(GpuBodyState);
    uint64_t shapeSz  = m_capacity * sizeof(GpuBodyShape);
    uint64_t paramsSz = sizeof(GpuShaderParams);
    uint64_t pairsSz  = (kPairsHeaderUints + static_cast<uint64_t>(m_maxPairs) * 2) * sizeof(uint32_t);

    auto makeBG = [&](const std::shared_ptr<cg::BindGroupLayout>& layout,
                      std::vector<cg::BindGroupEntryDescriptor> entries) {
        cg::BindGroupDescriptor desc;
        desc.layout  = layout;
        desc.entries = std::move(entries);
        return m_device->createBindGroup(desc);
    };

    m_aabbBindGroup = makeBG(m_aabbBGL, {
        bufEntry(0, m_stateBuffer,  stateSz),
        bufEntry(1, m_shapeBuffer,  shapeSz),
        bufEntry(2, m_paramsBuffer, paramsSz),
    });
    m_pairsBindGroup = makeBG(m_pairsBGL, {
        bufEntry(0, m_shapeBuffer,  shapeSz),
        bufEntry(1, m_pairsBuffer,  pairsSz),
        bufEntry(2, m_paramsBuffer, paramsSz),
    });
    m_predictBindGroup = makeBG(m_predictBGL, {
        bufEntry(0, m_stateBuffer,  stateSz),
        bufEntry(1, m_paramsBuffer, paramsSz),
    });
    m_contactsBindGroup = makeBG(m_contactsBGL, {
        bufEntry(0, m_stateBuffer,  stateSz),
        bufEntry(1, m_shapeBuffer,  shapeSz),
        bufEntry(2, m_pairsBuffer,  pairsSz),
        bufEntry(3, m_paramsBuffer, paramsSz),
    });
    m_finalizeBindGroup = makeBG(m_finalizeBGL, {
        bufEntry(0, m_stateBuffer,  stateSz),
        bufEntry(1, m_paramsBuffer, paramsSz),
    });
    if (m_renderBuffer && m_indexMapBuffer) {
        uint64_t renderSz   = m_renderCapacity * sizeof(GpuRenderTransform);
        uint64_t indexMapSz = m_capacity * sizeof(uint32_t);
        m_renderWriteBindGroup = makeBG(m_renderWriteBGL, {
            bufEntry(0, m_stateBuffer,    stateSz),
            bufEntry(1, m_indexMapBuffer, indexMapSz),
            bufEntry(2, m_renderBuffer,   renderSz),
            bufEntry(3, m_paramsBuffer,   paramsSz),
        });
    }
}

// ── Upload / download ─────────────────────────────────────────────────────────

void GpuPhysicsBackend::uploadBodies(const BodyPool& pool) {
    m_gpuToBodyId.clear();
    m_bodyIdToGpu.clear();

    std::vector<GpuBodyState> states;
    std::vector<GpuBodyShape> shapes;
    states.reserve(m_capacity);
    shapes.reserve(m_capacity);

    pool.forEach([&](uint32_t id, const BodyData& d) {
        if (!d.active || !d.shape) return;

        uint32_t gpuIdx = static_cast<uint32_t>(states.size());
        m_gpuToBodyId.push_back(id);
        m_bodyIdToGpu[id] = gpuIdx;

        GpuBodyState s{};
        s.pos[0] = d.transform.position.x();
        s.pos[1] = d.transform.position.y();
        s.pos[2] = d.transform.position.z();
        s.rot[0] = d.transform.rotation.x();
        s.rot[1] = d.transform.rotation.y();
        s.rot[2] = d.transform.rotation.z();
        s.rot[3] = d.transform.rotation.w();
        // predPos/predRot start equal to current state
        s.predPos[0] = s.pos[0];
        s.predPos[1] = s.pos[1];
        s.predPos[2] = s.pos[2];
        s.predRot[0] = s.rot[0];
        s.predRot[1] = s.rot[1];
        s.predRot[2] = s.rot[2];
        s.predRot[3] = s.rot[3];
        s.vel[0]    = d.linearVelocity.x();
        s.vel[1]    = d.linearVelocity.y();
        s.vel[2]    = d.linearVelocity.z();
        s.angVel[0] = d.angularVelocity.x();
        s.angVel[1] = d.angularVelocity.y();
        s.angVel[2] = d.angularVelocity.z();
        states.push_back(s);

        GpuBodyShape sh{};
        bool isStatic = (d.type == BodyType::Static || d.type == BodyType::Kinematic);
        sh.invMass     = isStatic ? 0.f : d.invMass;
        sh.invInertiaX = isStatic ? 0.f : d.invInertiaTensorLocal.x();
        sh.invInertiaY = isStatic ? 0.f : d.invInertiaTensorLocal.y();
        sh.invInertiaZ = isStatic ? 0.f : d.invInertiaTensorLocal.z();

        if (const auto* sp = dynamic_cast<const SphereShape*>(d.shape.get())) {
            sh.shapeType = 0.f;
            sh.param0    = sp->radius();
        } else if (const auto* bx = dynamic_cast<const BoxShape*>(d.shape.get())) {
            sh.shapeType = 1.f;
            sh.param0    = bx->halfExtents().x();
            sh.param1    = bx->halfExtents().y();
            sh.param2    = bx->halfExtents().z();
        } else if (const auto* ca = dynamic_cast<const CapsuleShape*>(d.shape.get())) {
            sh.shapeType = 2.f;
            sh.param0    = ca->radius();
            sh.param1    = ca->halfHeight();
        } else {
            // Unknown shape — approximate as sphere using AABB radius
            AABB aabb = d.shape->computeAABB(d.transform);
            float rx = (aabb.max.x() - aabb.min.x()) * 0.5f;
            float ry = (aabb.max.y() - aabb.min.y()) * 0.5f;
            float rz = (aabb.max.z() - aabb.min.z()) * 0.5f;
            sh.shapeType = 0.f;
            sh.param0    = std::max({rx, ry, rz});
        }
        shapes.push_back(sh);
    });

    if (!states.empty()) {
        m_stateBuffer->upload(0, states.size() * sizeof(GpuBodyState), states.data());
        m_shapeBuffer->upload(0, shapes.size() * sizeof(GpuBodyShape), shapes.data());
        m_indexMapBuffer->upload(0, m_gpuToBodyId.size() * sizeof(uint32_t), m_gpuToBodyId.data());
    }
}

void GpuPhysicsBackend::uploadParams(const IntegratorSettings& settings, float dt) {
    GpuShaderParams p{};
    p.gravity[0]  = settings.gravity.x();
    p.gravity[1]  = settings.gravity.y();
    p.gravity[2]  = settings.gravity.z();
    p.dt          = dt;
    p.bodyCount_f = static_cast<float>(m_gpuToBodyId.size());
    p.maxPairs_f  = static_cast<float>(m_maxPairs);
    p.compliance  = kCompliance;
    m_paramsBuffer->upload(0, sizeof(GpuShaderParams), &p);
}

void GpuPhysicsBackend::downloadBodies(BodyPool& pool) {
    uint32_t bodyCount = static_cast<uint32_t>(m_gpuToBodyId.size());
    if (bodyCount == 0) return;

    std::vector<GpuBodyState> states(bodyCount);
    m_stateStagingBuffer->download(0, bodyCount * sizeof(GpuBodyState), states.data());

    for (uint32_t gpuIdx = 0; gpuIdx < bodyCount; ++gpuIdx) {
        uint32_t bodyId = m_gpuToBodyId[gpuIdx];
        if (!pool.isValid(bodyId)) continue;

        BodyData& d = pool.get(bodyId);
        if (d.type == BodyType::Static) continue;  // GPU never moves statics

        const GpuBodyState& s = states[gpuIdx];
        d.transform.position = vm::Vector3<float>(s.pos[0], s.pos[1], s.pos[2]);
        d.transform.rotation = vm::Quaternion<float>(s.rot[0], s.rot[1], s.rot[2], s.rot[3]);
        d.linearVelocity     = vm::Vector3<float>(s.vel[0], s.vel[1], s.vel[2]);
        d.angularVelocity    = vm::Vector3<float>(s.angVel[0], s.angVel[1], s.angVel[2]);
    }
}

// ── step ─────────────────────────────────────────────────────────────────────

void GpuPhysicsBackend::step(BodyPool& pool, const IntegratorSettings& settings, float dt) {
    uint32_t bodyCount = pool.activeCount();
    if (bodyCount == 0) return;

    ensureCapacity(bodyCount);
    ensureRenderCapacity(pool.capacity());
    // Rebuild render bind group if render or index map buffers were just allocated
    if (m_renderBuffer && m_indexMapBuffer && !m_renderWriteBindGroup)
        rebuildBindGroups();
    uploadBodies(pool);
    uploadParams(settings, dt);

    const uint32_t actualCount = static_cast<uint32_t>(m_gpuToBodyId.size());
    if (actualCount == 0) return;

    const uint64_t wg64 = 64;
    auto wg = [&](uint64_t n) { return (n + wg64 - 1) / wg64; };

    // Pair check threads: N*(N-1)/2 (each potential unique pair (i<j))
    uint64_t pairThreads = static_cast<uint64_t>(actualCount) *
                           (static_cast<uint64_t>(actualCount) - 1) / 2;

    // Zero pair count before broadphase
    auto encoder = m_device->createCommandEncoder();
    encoder->clearBuffer(m_pairsBuffer, 0, sizeof(uint32_t) * kPairsHeaderUints);

    {   // broadphase_aabb: update world AABBs
        auto pass = encoder->beginComputePass();
        pass->setPipeline(m_aabbPipeline);
        pass->setBindGroup(0, m_aabbBindGroup, {}, 0, 0);
        pass->dispatchWorkgroups(wg(actualCount));
        pass->end();
    }
    {   // broadphase_pairs: detect overlapping AABB pairs (O(N²/2))
        auto pass = encoder->beginComputePass();
        pass->setPipeline(m_pairsPipeline);
        pass->setBindGroup(0, m_pairsBindGroup, {}, 0, 0);
        pass->dispatchWorkgroups(wg(pairThreads ? pairThreads : 1));
        pass->end();
    }
    {   // xpbd_predict: semi-implicit Euler prediction
        auto pass = encoder->beginComputePass();
        pass->setPipeline(m_predictPipeline);
        pass->setBindGroup(0, m_predictBindGroup, {}, 0, 0);
        pass->dispatchWorkgroups(wg(actualCount));
        pass->end();
    }
    // xpbd_contacts: Jacobi contact resolution (kXpbdIterations passes)
    for (int i = 0; i < kXpbdIterations; ++i) {
        auto pass = encoder->beginComputePass();
        pass->setPipeline(m_contactsPipeline);
        pass->setBindGroup(0, m_contactsBindGroup, {}, 0, 0);
        pass->dispatchWorkgroups(wg(m_maxPairs));
        pass->end();
    }
    {   // xpbd_finalize: commit predicted positions, update velocities
        auto pass = encoder->beginComputePass();
        pass->setPipeline(m_finalizePipeline);
        pass->setBindGroup(0, m_finalizeBindGroup, {}, 0, 0);
        pass->dispatchWorkgroups(wg(actualCount));
        pass->end();
    }
    if (m_renderWriteBindGroup) {
        // write_render_transforms: scatter finalized pos+rot into pool-slot render buffer
        auto pass = encoder->beginComputePass();
        pass->setPipeline(m_renderWritePipeline);
        pass->setBindGroup(0, m_renderWriteBindGroup, {}, 0, 0);
        pass->dispatchWorkgroups(wg(actualCount));
        pass->end();
    }

    // Copy state buffer → staging buffer so CPU can read back results.
    // On macOS, storage buffers use MTLStorageModeManaged (separate GPU copy);
    // the staging buffer uses MTLStorageModeShared and is always CPU-visible.
    {
        uint64_t stateSz = static_cast<uint64_t>(actualCount) * sizeof(GpuBodyState);
        encoder->copyBufferToBuffer(m_stateBuffer, 0, m_stateStagingBuffer, 0, stateSz);
    }

    m_device->submit(encoder->finish());
    m_device->waitForIdle();

    downloadBodies(pool);
}

} // namespace campello::physics

#endif // CAMPELLO_PHYSICS_GPU_ENABLED
