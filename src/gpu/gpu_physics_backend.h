#pragma once

#ifdef CAMPELLO_PHYSICS_GPU_ENABLED

#include "gpu_backend.h"
#include <campello_gpu/device.hpp>
#include <campello_gpu/buffer.hpp>
#include <campello_gpu/compute_pipeline.hpp>
#include <campello_gpu/bind_group.hpp>
#include <campello_gpu/bind_group_layout.hpp>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace cg = systems::leal::campello_gpu;

namespace campello::physics {

class GpuPhysicsBackend final : public IGpuBackend {
public:
    static constexpr int   kXpbdIterations = 10;
    static constexpr float kCompliance     = 0.f;   // rigid bodies
    static constexpr float kJacobiScale    = 0.4f;  // Jacobi stability factor

    bool initialize(std::shared_ptr<void> device = nullptr) override;
    void step(BodyPool& pool, const IntegratorSettings& settings, float dt) override;
    std::shared_ptr<void> renderBuffer() const override { return m_renderBuffer; }

private:
    std::shared_ptr<cg::Device> m_device;

    // Per-step GPU buffers
    std::shared_ptr<cg::Buffer> m_stateBuffer;        // GpuBodyState[] — managed (GPU compute)
    std::shared_ptr<cg::Buffer> m_stateStagingBuffer; // GpuBodyState[] — shared (CPU readback)
    std::shared_ptr<cg::Buffer> m_shapeBuffer;        // GpuBodyShape[]
    std::shared_ptr<cg::Buffer> m_paramsBuffer;       // GpuShaderParams
    std::shared_ptr<cg::Buffer> m_pairsBuffer;        // uint32 pairs data
    std::shared_ptr<cg::Buffer> m_indexMapBuffer;     // uint32[] gpuIdx→poolSlot (uploaded each step)
    std::shared_ptr<cg::Buffer> m_renderBuffer;       // GpuRenderTransform[] pool-slot indexed

    // Compute pipelines (one per kernel)
    std::shared_ptr<cg::ComputePipeline> m_aabbPipeline;
    std::shared_ptr<cg::ComputePipeline> m_pairsPipeline;
    std::shared_ptr<cg::ComputePipeline> m_predictPipeline;
    std::shared_ptr<cg::ComputePipeline> m_contactsPipeline;
    std::shared_ptr<cg::ComputePipeline> m_finalizePipeline;
    std::shared_ptr<cg::ComputePipeline> m_renderWritePipeline;

    // Bind groups (rebuilt when capacity changes)
    std::shared_ptr<cg::BindGroup> m_aabbBindGroup;
    std::shared_ptr<cg::BindGroup> m_pairsBindGroup;
    std::shared_ptr<cg::BindGroup> m_predictBindGroup;
    std::shared_ptr<cg::BindGroup> m_contactsBindGroup;
    std::shared_ptr<cg::BindGroup> m_finalizeBindGroup;
    std::shared_ptr<cg::BindGroup> m_renderWriteBindGroup;

    // Bind group layouts
    std::shared_ptr<cg::BindGroupLayout> m_aabbBGL;
    std::shared_ptr<cg::BindGroupLayout> m_pairsBGL;
    std::shared_ptr<cg::BindGroupLayout> m_predictBGL;
    std::shared_ptr<cg::BindGroupLayout> m_contactsBGL;
    std::shared_ptr<cg::BindGroupLayout> m_finalizeBGL;
    std::shared_ptr<cg::BindGroupLayout> m_renderWriteBGL;

    // Body index mapping
    uint32_t                    m_capacity = 0;       // physics buffer capacity (compact)
    uint32_t                    m_maxPairs = 0;
    uint32_t                    m_renderCapacity = 0; // render buffer capacity (pool slots)
    std::vector<uint32_t>       m_gpuToBodyId;     // GPU index → pool body ID
    std::unordered_map<uint32_t,uint32_t> m_bodyIdToGpu; // pool ID → GPU index

    bool loadPipelines();
    void ensureCapacity(uint32_t bodyCount);
    void ensureRenderCapacity(uint32_t poolCapacity);
    void rebuildBindGroups();
    void uploadBodies(const BodyPool& pool);
    void uploadParams(const IntegratorSettings& settings, float dt);
    void downloadBodies(BodyPool& pool);

    std::shared_ptr<cg::ShaderModule> loadShaderFile(const std::string& kernelName);
    std::shared_ptr<cg::ComputePipeline> buildKernelPipeline(
        const std::string& kernelName,
        const std::shared_ptr<cg::BindGroupLayout>& bgl);
    std::shared_ptr<cg::BindGroupLayout> makeStorageBGL(
        const std::vector<std::pair<uint32_t,bool>>& bindings,  // {slot, readonly}
        uint32_t uniformSlot = ~0u);
};

} // namespace campello::physics

#endif // CAMPELLO_PHYSICS_GPU_ENABLED
