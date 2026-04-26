#pragma once

#include <campello_physics/body_pool.h>
#include <campello_physics/integrator.h>
#include <cstdint>

namespace campello::physics {

// ── GPU buffer layouts ────────────────────────────────────────────────────────
// All structs use float4 fields (16-byte alignment) for cross-platform
// compatibility across GLSL std430, MSL, and HLSL structured buffers.

struct alignas(16) GpuBodyState {
    float pos[3];     float _p0;      // current position
    float rot[4];                      // current rotation (xyzw quaternion)
    float predPos[3]; float _p1;      // predicted position (XPBD)
    float predRot[4];                  // predicted rotation (XPBD)
    float vel[3];     float _p2;      // linear velocity
    float angVel[3];  float _p3;      // angular velocity
};
static_assert(sizeof(GpuBodyState) == 96, "GpuBodyState must be 96 bytes");

struct alignas(16) GpuBodyShape {
    float aabbMin[3]; float invMass;      // world AABB min + inverse mass
    float aabbMax[3]; float invInertiaX;  // world AABB max + inv inertia X
    float invInertiaY; float invInertiaZ; float shapeType; float param0;
    float param1;      float param2;      float param3;    float _p;
    // shapeType: 0=sphere (param0=radius), 1=box (param0-2=halfExtents), 2=capsule
    // invMass==0 means static/kinematic — GPU kernels skip position update
};
static_assert(sizeof(GpuBodyShape) == 64, "GpuBodyShape must be 64 bytes");

struct alignas(16) GpuShaderParams {
    float gravity[3];  float dt;
    float bodyCount_f; float maxPairs_f; float compliance; float _p;
};
static_assert(sizeof(GpuShaderParams) == 32, "GpuShaderParams must be 32 bytes");

// Pairs buffer layout (raw uint32 array):
//   [0]          = pair count (written atomically by broadphase_pairs)
//   [1]          = padding
//   [2 + k*2]    = bodyA index of pair k
//   [2 + k*2 + 1]= bodyB index of pair k
static constexpr uint32_t kPairsHeaderUints = 2;

// ── Render transform buffer layout ────────────────────────────────────────────
// One entry per pool slot (body ID), so render[bodyId] = that body's transform.
// Inactive/static slots are written once at creation and never moved.
// Size = pool.capacity() * sizeof(GpuRenderTransform).
// Stride: 32 bytes.
struct alignas(16) GpuRenderTransform {
    float position[3]; float _w;   // xyz = world position, w = 1
    float rotation[4];             // xyzw quaternion
};
static_assert(sizeof(GpuRenderTransform) == 32, "GpuRenderTransform must be 32 bytes");

// ── IGpuBackend ───────────────────────────────────────────────────────────────

class IGpuBackend {
public:
    virtual ~IGpuBackend() = default;
    // device: type-erased shared_ptr<cg::Device>. If null, the backend creates its own.
    virtual bool initialize(std::shared_ptr<void> device = nullptr) = 0;
    virtual void step(BodyPool& pool, const IntegratorSettings& settings, float dt) = 0;

    // Returns the render transform buffer as a type-erased shared_ptr<cg::Buffer>.
    // Layout: GpuRenderTransform[pool.capacity()], indexed by body pool ID.
    // Valid after the first step(); null before initialization or on CPU fallback.
    virtual std::shared_ptr<void> renderBuffer() const = 0;
};

} // namespace campello::physics
