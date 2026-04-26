#include <campello_physics/body_pool.h>
#include <cassert>
#include <cmath>
#include <stdexcept>

namespace campello::physics {

// ── BodyPool ──────────────────────────────────────────────────────────────────

Body BodyPool::createBody(const BodyDescriptor& desc) {
    uint32_t id;
    if (!m_freeList.empty()) {
        id = m_freeList.back();
        m_freeList.pop_back();
        m_data[id] = BodyData{};
    } else {
        id = static_cast<uint32_t>(m_data.size());
        m_data.emplace_back();
    }

    BodyData& d = m_data[id];
    d.active          = true;
    d.type            = desc.type;
    d.shape           = desc.shape;
    d.transform       = desc.transform;
    d.linearVelocity  = desc.linearVelocity;
    d.angularVelocity = desc.angularVelocity;
    d.linearDamping   = desc.linearDamping;
    d.angularDamping  = desc.angularDamping;
    d.restitution     = desc.restitution;
    d.friction        = desc.friction;
    d.layer           = desc.layer;
    d.mask            = desc.mask;
    d.ccdEnabled      = desc.ccdEnabled;

    if (desc.type == BodyType::Dynamic && desc.mass > 0.f) {
        d.invMass = 1.f / desc.mass;
        if (desc.shape) {
            const auto iDiag = desc.shape->computeLocalInertiaDiagonal(desc.mass);
            d.invInertiaTensorLocal = vm::Vector3<float>(
                iDiag.x() > 1e-30f ? 1.f / iDiag.x() : 0.f,
                iDiag.y() > 1e-30f ? 1.f / iDiag.y() : 0.f,
                iDiag.z() > 1e-30f ? 1.f / iDiag.z() : 0.f
            );
        }
    }

    ++m_activeCount;
    return Body(id, this);
}

void BodyPool::destroyBody(uint32_t id) {
    assert(id < m_data.size() && m_data[id].active);
    m_data[id].active = false;
    m_data[id].shape.reset();
    m_freeList.push_back(id);
    --m_activeCount;
}

void BodyPool::forEach(const std::function<void(uint32_t, BodyData&)>& fn) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_data.size()); ++i)
        if (m_data[i].active) fn(i, m_data[i]);
}

void BodyPool::forEach(const std::function<void(uint32_t, const BodyData&)>& fn) const {
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_data.size()); ++i)
        if (m_data[i].active) fn(i, m_data[i]);
}

// ── Body method implementations ───────────────────────────────────────────────

bool Body::isValid() const noexcept {
    return m_pool && m_pool->isValid(m_id);
}

BodyType Body::type() const noexcept {
    return m_pool->get(m_id).type;
}

Transform Body::transform() const noexcept {
    return m_pool->get(m_id).transform;
}

vm::Vector3<float> Body::linearVelocity() const noexcept {
    return m_pool->get(m_id).linearVelocity;
}

vm::Vector3<float> Body::angularVelocity() const noexcept {
    return m_pool->get(m_id).angularVelocity;
}

float Body::mass() const noexcept {
    const float inv = m_pool->get(m_id).invMass;
    return inv > 0.f ? 1.f / inv : 0.f;
}

float Body::invMass() const noexcept {
    return m_pool->get(m_id).invMass;
}

bool Body::isSleeping() const noexcept {
    return m_pool->get(m_id).isSleeping;
}

void Body::setTransform(const Transform& t) noexcept {
    auto& d = m_pool->get(m_id);
    d.transform = t;
    d.isSleeping = false;
    d.sleepFrames = 0;
}

void Body::setLinearVelocity(const vm::Vector3<float>& v) noexcept {
    auto& d = m_pool->get(m_id);
    d.linearVelocity = v;
    d.isSleeping = false;
    d.sleepFrames = 0;
}

void Body::setAngularVelocity(const vm::Vector3<float>& w) noexcept {
    auto& d = m_pool->get(m_id);
    d.angularVelocity = w;
    d.isSleeping = false;
    d.sleepFrames = 0;
}

void Body::setLinearDamping(float damp) noexcept {
    m_pool->get(m_id).linearDamping = damp;
}

void Body::setAngularDamping(float damp) noexcept {
    m_pool->get(m_id).angularDamping = damp;
}

float Body::restitution() const noexcept {
    return m_pool->get(m_id).restitution;
}

float Body::friction() const noexcept {
    return m_pool->get(m_id).friction;
}

void Body::setRestitution(float r) noexcept {
    m_pool->get(m_id).restitution = r;
}

void Body::setFriction(float f) noexcept {
    m_pool->get(m_id).friction = f;
}

void Body::applyForce(const vm::Vector3<float>& force) noexcept {
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    d.forceAccum = d.forceAccum + force;
}

void Body::applyForceAt(const vm::Vector3<float>& force,
                        const vm::Vector3<float>& worldPoint) noexcept {
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    d.forceAccum  = d.forceAccum + force;
    // torque = r × F  where r = worldPoint - center
    const auto r  = worldPoint - d.transform.position;
    const auto rX = r.y() * force.z() - r.z() * force.y();
    const auto rY = r.z() * force.x() - r.x() * force.z();
    const auto rZ = r.x() * force.y() - r.y() * force.x();
    d.torqueAccum = d.torqueAccum + vm::Vector3<float>(rX, rY, rZ);
}

void Body::applyTorque(const vm::Vector3<float>& torque) noexcept {
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    d.torqueAccum = d.torqueAccum + torque;
}

void Body::applyLinearImpulse(const vm::Vector3<float>& imp) noexcept {
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    d.linearVelocity = d.linearVelocity + imp * d.invMass;
}

void Body::applyAngularImpulse(const vm::Vector3<float>& imp) noexcept {
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    // Δω = I_world^-1 * J  =  R * (invI_local ⊙ (R^T * J))
    const auto& R     = d.transform.rotation;
    const auto  local = R.conjugated().rotated(imp);
    const auto  inv   = d.invInertiaTensorLocal;
    const auto  scaled = vm::Vector3<float>(
        local.x() * inv.x(), local.y() * inv.y(), local.z() * inv.z());
    d.angularVelocity = d.angularVelocity + R.rotated(scaled);
}

void Body::wake() noexcept {
    auto& d = m_pool->get(m_id);
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void Body::sleep() noexcept {
    auto& d = m_pool->get(m_id);
    d.isSleeping      = true;
    d.sleepFrames     = 0;
    d.linearVelocity  = { 0.f, 0.f, 0.f };
    d.angularVelocity = { 0.f, 0.f, 0.f };
}

} // namespace campello::physics
