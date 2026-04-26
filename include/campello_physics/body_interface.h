#pragma once

#include <campello_physics/body_pool.h>
#include <shared_mutex>

namespace campello::physics {

class PhysicsWorld;  // forward-declaration; full definition in physics_world.h

// ── BodyInterface ─────────────────────────────────────────────────────────────
//
// Thread-safe interface for interacting with physics bodies from any thread.
//
// Structural operations (createBody, destroyBody) and state mutations
// (setTransform, applyForce, …) acquire an exclusive lock and therefore
// block while PhysicsWorld::step() is running — and vice versa.
//
// Read operations (getTransform, getLinearVelocity, …) are LOCK-FREE.
// They are safe to call from any thread between physics steps.
// Calling reads concurrently with step() produces a data race (undefined
// behaviour); use an external synchronisation point or double-buffering if
// you need to read while the simulation is running.
//
// Usage:
//   PhysicsWorld world;
//   Body b = world.bodyInterface().createBody(desc);   // from any thread
//   Transform t = world.bodyInterface().getTransform(b); // lock-free read

class BodyInterface {
public:
    BodyInterface(PhysicsWorld& world, BodyPool& pool,
                  std::shared_mutex& mutex) noexcept
        : m_world(world), m_pool(pool), m_mutex(mutex) {}

    // Non-copyable / non-movable — the PhysicsWorld owns the single instance.
    BodyInterface(const BodyInterface&)            = delete;
    BodyInterface& operator=(const BodyInterface&) = delete;
    BodyInterface(BodyInterface&&)                 = delete;
    BodyInterface& operator=(BodyInterface&&)      = delete;

    // ── Structural changes — exclusive lock ───────────────────────────────────
    // Safe to call from any thread.  Blocks while step() is running.

    [[nodiscard]] Body createBody (const BodyDescriptor& desc);
    void               destroyBody(Body body);

    // ── Lock-free reads ───────────────────────────────────────────────────────
    // No synchronisation cost.  Do NOT call while step() may be running.

    [[nodiscard]] bool               isValid          (Body body) const noexcept;
    [[nodiscard]] BodyType           getBodyType      (Body body) const noexcept;
    [[nodiscard]] float              getMass          (Body body) const noexcept;
    [[nodiscard]] Transform          getTransform     (Body body) const noexcept;
    [[nodiscard]] vm::Vector3<float> getLinearVelocity (Body body) const noexcept;
    [[nodiscard]] vm::Vector3<float> getAngularVelocity(Body body) const noexcept;
    [[nodiscard]] bool               isSleeping       (Body body) const noexcept;

    // ── State mutations — exclusive lock ──────────────────────────────────────
    // Safe to call from any thread.  Blocks while step() is running.

    void setTransform        (Body body, const Transform&          t) noexcept;
    void setLinearVelocity   (Body body, const vm::Vector3<float>& v) noexcept;
    void setAngularVelocity  (Body body, const vm::Vector3<float>& w) noexcept;
    void applyForce          (Body body, const vm::Vector3<float>& f) noexcept;
    void applyTorque         (Body body, const vm::Vector3<float>& t) noexcept;
    void applyLinearImpulse  (Body body, const vm::Vector3<float>& i) noexcept;
    void applyAngularImpulse (Body body, const vm::Vector3<float>& i) noexcept;
    void wake                (Body body) noexcept;

private:
    PhysicsWorld&      m_world;
    BodyPool&          m_pool;
    std::shared_mutex& m_mutex;
};

} // namespace campello::physics
