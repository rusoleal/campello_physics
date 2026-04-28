#pragma once

#include <campello_physics/defines.h>
#include <campello_physics/transform.h>
#include <campello_physics/shape.h>
#include <cstdint>
#include <memory>

namespace campello::physics {

class BodyPool; // forward declaration

// ── Body type ──────────────────────────────────────────────────────────────────

enum class BodyType : uint8_t {
    Static,     // zero inverse mass; never moved by the integrator
    Kinematic,  // zero inverse mass; moved via velocity directly (no forces)
    Dynamic,    // full physics: mass, forces, constraints
    Sensor      // like Static but only generates contact events; no impulse response
};

// ── Body descriptor (creation parameters) ────────────────────────────────────

struct BodyDescriptor {
    BodyType                   type            = BodyType::Dynamic;
    Transform                  transform       = Transform::identity();
    vm::Vector3<float>         linearVelocity  = { 0.f, 0.f, 0.f };
    vm::Vector3<float>         angularVelocity = { 0.f, 0.f, 0.f };
    float                      mass            = 1.f;     // ignored for Static/Kinematic/Sensor
    std::shared_ptr<Shape>     shape;                     // may be null
    float                      linearDamping   = 0.01f;
    float                      angularDamping  = 0.05f;
    float                      restitution     = 0.3f;   // bounciness [0,1]
    float                      friction        = 0.5f;   // Coulomb friction coefficient
    uint32_t                   layer           = 0xFFFFFFFF;
    uint32_t                   mask            = 0xFFFFFFFF;
    bool                       ccdEnabled      = false;   // continuous collision detection
};

// ── Body handle ───────────────────────────────────────────────────────────────
//
// A thin copyable value that wraps a uint32_t ID.  All mutation goes through
// the owning BodyPool.  Copy freely; equality is ID equality.

class Body {
public:
    Body() = default;

    [[nodiscard]] bool     isValid() const noexcept;
    [[nodiscard]] uint32_t id()         const noexcept { return m_id; }
    [[nodiscard]] uint32_t generation() const noexcept { return m_generation; }

    // ── State getters ─────────────────────────────────────────────────────────
    [[nodiscard]] BodyType           type()            const noexcept;
    [[nodiscard]] Transform          transform()       const noexcept;
    [[nodiscard]] vm::Vector3<float> linearVelocity()  const noexcept;
    [[nodiscard]] vm::Vector3<float> angularVelocity() const noexcept;
    [[nodiscard]] float              mass()            const noexcept;
    [[nodiscard]] float              invMass()         const noexcept;
    [[nodiscard]] bool               isSleeping()      const noexcept;
    [[nodiscard]] float              restitution()     const noexcept;
    [[nodiscard]] float              friction()        const noexcept;

    // ── State setters (also wakes the body) ───────────────────────────────────
    void setTransform      (const Transform&          t) noexcept;
    void setLinearVelocity (const vm::Vector3<float>& v) noexcept;
    void setAngularVelocity(const vm::Vector3<float>& w) noexcept;
    void setLinearDamping  (float d) noexcept;
    void setAngularDamping (float d) noexcept;
    void setRestitution    (float r) noexcept;
    void setFriction       (float f) noexcept;

    // ── Force / impulse ───────────────────────────────────────────────────────
    // Forces are accumulated and consumed by the integrator each step.
    void applyForce          (const vm::Vector3<float>& force)       noexcept;
    void applyForceAt        (const vm::Vector3<float>& force,
                              const vm::Vector3<float>& worldPoint)  noexcept;
    void applyTorque         (const vm::Vector3<float>& torque)      noexcept;
    // Impulses change velocity immediately.
    void applyLinearImpulse  (const vm::Vector3<float>& impulse)     noexcept;
    void applyAngularImpulse (const vm::Vector3<float>& impulse)     noexcept;

    // ── Sleep ─────────────────────────────────────────────────────────────────
    void wake()  noexcept;
    void sleep() noexcept;

    // ── Equality ──────────────────────────────────────────────────────────────
    bool operator==(const Body& o) const noexcept {
        return m_id == o.m_id && m_generation == o.m_generation && m_pool == o.m_pool;
    }
    bool operator!=(const Body& o) const noexcept { return !(*this == o); }

private:
    explicit Body(uint32_t id, uint32_t generation, BodyPool* pool) noexcept
        : m_id(id), m_generation(generation), m_pool(pool) {}
    friend class BodyPool;

    uint32_t  m_id         = ~uint32_t(0);
    uint32_t  m_generation = 0;
    BodyPool* m_pool       = nullptr;
};

} // namespace campello::physics
