#pragma once

#include <campello_physics/body.h>
#include <campello_physics/narrow_phase.h>  // for ShapeInstance
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace campello::physics {

// ── Per-body storage (internal) ───────────────────────────────────────────────

struct BodyData {
    // Identity
    BodyType  type   = BodyType::Dynamic;
    bool      active = false;   // false → slot is free

    // Shape (optional; may be null for point masses)
    std::shared_ptr<Shape> shape;

    // Mass / inertia
    float              invMass               = 0.f;
    vm::Vector3<float> invInertiaTensorLocal = { 0.f, 0.f, 0.f }; // diagonal I^-1 in local space

    // Kinematics
    Transform          transform;
    vm::Vector3<float> linearVelocity  = { 0.f, 0.f, 0.f };
    vm::Vector3<float> angularVelocity = { 0.f, 0.f, 0.f };

    // Accumulators (reset to zero after each integrate step)
    vm::Vector3<float> forceAccum  = { 0.f, 0.f, 0.f };
    vm::Vector3<float> torqueAccum = { 0.f, 0.f, 0.f };

    // Damping coefficients (fraction of velocity removed per second, 0 = none)
    float linearDamping  = 0.01f;
    float angularDamping = 0.05f;

    // Surface properties
    float restitution = 0.3f;
    float friction    = 0.5f;

    // Collision filtering
    uint32_t layer = 0xFFFFFFFF;
    uint32_t mask  = 0xFFFFFFFF;

    // Sleep state
    bool isSleeping = false;
    int  sleepFrames = 0;  // consecutive frames below sleep threshold

    // CCD flag
    bool ccdEnabled = false;
};

// ── Body pool ─────────────────────────────────────────────────────────────────
//
// Owns all body storage in a flat contiguous vector.  Bodies are identified
// by their integer ID (index into the vector).  Destroyed slots go on a free
// list for reuse.

class BodyPool {
public:
    static constexpr uint32_t kInvalidId = ~uint32_t(0);

    // Create a new body.  Returns a handle to it.
    [[nodiscard]] Body createBody(const BodyDescriptor& desc);

    // Destroy a body.  The handle becomes invalid.
    void destroyBody(uint32_t id);
    void destroyBody(Body body) { destroyBody(body.id()); }

    // Direct data access (for the integrator and constraint solver)
    [[nodiscard]] BodyData&       get(uint32_t id)       noexcept { return m_data[id]; }
    [[nodiscard]] const BodyData& get(uint32_t id) const noexcept { return m_data[id]; }

    [[nodiscard]] bool isValid(uint32_t id) const noexcept {
        return id < m_data.size() && m_data[id].active;
    }

    // Creates a Body handle pointing into this pool (for use by the pipeline).
    [[nodiscard]] Body makeHandle(uint32_t id) noexcept { return Body(id, this); }
    [[nodiscard]] Body makeHandle(uint32_t id) const noexcept {
        return Body(id, const_cast<BodyPool*>(this));
    }

    // Returns a ShapeInstance for use with the narrow phase.
    [[nodiscard]] ShapeInstance getShapeInstance(uint32_t id) const noexcept {
        const auto& d = m_data[id];
        return { d.shape.get(), d.transform };
    }

    // Iterate over all active bodies.
    void forEach(const std::function<void(uint32_t id, BodyData&)>& fn);
    void forEach(const std::function<void(uint32_t id, const BodyData&)>& fn) const;

    [[nodiscard]] uint32_t activeCount() const noexcept { return m_activeCount; }
    [[nodiscard]] uint32_t capacity()    const noexcept {
        return static_cast<uint32_t>(m_data.size());
    }

private:
    std::vector<BodyData> m_data;
    std::vector<uint32_t> m_freeList;
    uint32_t              m_activeCount = 0;
};

} // namespace campello::physics
