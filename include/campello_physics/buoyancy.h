#pragma once

#include <campello_physics/body.h>
#include <campello_physics/shapes.h>
#include <memory>

namespace campello::physics {

// ── BuoyancyDescriptor ────────────────────────────────────────────────────────
//
// Defines a fluid volume that applies Archimedes buoyancy and viscous drag
// to any Dynamic body overlapping with it.
//
// The fluid surface is taken as the AABB max Y of the volume shape in world
// space after applying volumeTransform.  Only the upward component of gravity
// is used for the buoyant force (gravity must point down for buoyancy to work).

struct BuoyancyDescriptor {
    std::shared_ptr<Shape> shape;           // volume geometry (box or sphere recommended)
    Transform              transform       = Transform::identity();
    float                  fluidDensity   = 1000.f;  // kg/m³ (water ≈ 1000)
    float                  linearDrag     = 1.5f;    // velocity damping inside fluid
    float                  angularDrag    = 1.0f;    // angular velocity damping inside fluid
};

// ── BuoyancyVolume ────────────────────────────────────────────────────────────
//
// Thin copyable handle returned by PhysicsWorld::addBuoyancyVolume().
// Equality is ID equality.

class BuoyancyVolume {
public:
    BuoyancyVolume() = default;

    [[nodiscard]] bool     isValid() const noexcept { return m_id != kInvalid; }
    [[nodiscard]] uint32_t id()      const noexcept { return m_id; }

    bool operator==(const BuoyancyVolume& o) const noexcept { return m_id == o.m_id; }
    bool operator!=(const BuoyancyVolume& o) const noexcept { return m_id != o.m_id; }

private:
    static constexpr uint32_t kInvalid = ~uint32_t(0);
    explicit BuoyancyVolume(uint32_t id) noexcept : m_id(id) {}
    friend class BuoyancySystem;
    uint32_t m_id = kInvalid;
};

} // namespace campello::physics
