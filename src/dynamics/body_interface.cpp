#include <campello_physics/body_interface.h>
#include <campello_physics/physics_world.h>

namespace campello::physics {

// ── Structural changes ────────────────────────────────────────────────────────

Body BodyInterface::createBody(const BodyDescriptor& desc) {
    std::unique_lock lock(m_mutex);
    return m_world.createBody(desc);
}

void BodyInterface::destroyBody(Body body) {
    std::unique_lock lock(m_mutex);
    m_world.destroyBody(body);
}

// ── Lock-free reads ───────────────────────────────────────────────────────────

bool BodyInterface::isValid(Body body) const noexcept {
    return body.isValid();
}

BodyType BodyInterface::getBodyType(Body body) const noexcept {
    return m_pool.get(body.id()).type;
}

float BodyInterface::getMass(Body body) const noexcept {
    float inv = m_pool.get(body.id()).invMass;
    return inv > 0.f ? 1.f / inv : 0.f;
}

Transform BodyInterface::getTransform(Body body) const noexcept {
    return m_pool.get(body.id()).transform;
}

vm::Vector3<float> BodyInterface::getLinearVelocity(Body body) const noexcept {
    return m_pool.get(body.id()).linearVelocity;
}

vm::Vector3<float> BodyInterface::getAngularVelocity(Body body) const noexcept {
    return m_pool.get(body.id()).angularVelocity;
}

bool BodyInterface::isSleeping(Body body) const noexcept {
    return m_pool.get(body.id()).isSleeping;
}

// ── State mutations ───────────────────────────────────────────────────────────

void BodyInterface::setTransform(Body body, const Transform& t) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    d.transform   = t;
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void BodyInterface::setLinearVelocity(Body body, const vm::Vector3<float>& v) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    d.linearVelocity = v;
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void BodyInterface::setAngularVelocity(Body body, const vm::Vector3<float>& w) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    d.angularVelocity = w;
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void BodyInterface::applyForce(Body body, const vm::Vector3<float>& f) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    if (d.type != BodyType::Dynamic) return;
    d.forceAccum  = d.forceAccum + f;
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void BodyInterface::applyTorque(Body body, const vm::Vector3<float>& t) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    if (d.type != BodyType::Dynamic) return;
    d.torqueAccum = d.torqueAccum + t;
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void BodyInterface::applyLinearImpulse(Body body, const vm::Vector3<float>& imp) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    if (d.type != BodyType::Dynamic) return;
    d.linearVelocity = d.linearVelocity + imp * d.invMass;
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void BodyInterface::applyAngularImpulse(Body body, const vm::Vector3<float>& imp) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    if (d.type != BodyType::Dynamic) return;
    const auto& R   = d.transform.rotation;
    const auto  loc = R.conjugated().rotated(imp);
    const auto& inv = d.invInertiaTensorLocal;
    d.angularVelocity = d.angularVelocity +
        R.rotated(vm::Vector3<float>(loc.x()*inv.x(), loc.y()*inv.y(), loc.z()*inv.z()));
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void BodyInterface::wake(Body body) noexcept {
    std::unique_lock lock(m_mutex);
    auto& d = m_pool.get(body.id());
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

} // namespace campello::physics
