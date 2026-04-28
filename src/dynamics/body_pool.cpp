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
        uint32_t preservedGeneration = m_data[id].generation;
        m_data[id] = BodyData{};
        m_data[id].generation = preservedGeneration;
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
        // Pre-compute world inverse inertia from initial orientation
        const auto& q = d.transform.rotation;
        float x = q.x(), y = q.y(), z = q.z(), w = q.w();
        float xx = x*x, yy = y*y, zz = z*z;
        float xy = x*y, xz = x*z, yz = y*z;
        float wx = w*x, wy = w*y, wz = w*z;
        float r00 = 1.f - 2.f*(yy + zz);
        float r01 = 2.f*(xy - wz);
        float r02 = 2.f*(xz + wy);
        float r10 = 2.f*(xy + wz);
        float r11 = 1.f - 2.f*(xx + zz);
        float r12 = 2.f*(yz - wx);
        float r20 = 2.f*(xz - wy);
        float r21 = 2.f*(yz + wx);
        float r22 = 1.f - 2.f*(xx + yy);
        float ix = d.invInertiaTensorLocal.x();
        float iy = d.invInertiaTensorLocal.y();
        float iz = d.invInertiaTensorLocal.z();
        auto& m = d.invInertiaTensorWorld.data;
        m[0] = r00*r00*ix + r01*r01*iy + r02*r02*iz;
        m[1] = r00*r10*ix + r01*r11*iy + r02*r12*iz;
        m[2] = r00*r20*ix + r01*r21*iy + r02*r22*iz;
        m[3] = r10*r00*ix + r11*r01*iy + r12*r02*iz;
        m[4] = r10*r10*ix + r11*r11*iy + r12*r12*iz;
        m[5] = r10*r20*ix + r11*r21*iy + r12*r22*iz;
        m[6] = r20*r00*ix + r21*r01*iy + r22*r02*iz;
        m[7] = r20*r10*ix + r21*r11*iy + r22*r12*iz;
        m[8] = r20*r20*ix + r21*r21*iy + r22*r22*iz;
    }

    if (desc.type == BodyType::Dynamic)
        m_activeDynamicIds.push_back(id);
    if (desc.ccdEnabled)
        ++m_ccdEnabledCount;

    ++m_activeCount;
    return Body(id, m_data[id].generation, this);
}

void BodyPool::destroyBody(uint32_t id) {
    assert(id < m_data.size() && m_data[id].active);
    if (m_data[id].type == BodyType::Dynamic) {
        for (size_t i = 0; i < m_activeDynamicIds.size(); ++i) {
            if (m_activeDynamicIds[i] == id) {
                m_activeDynamicIds[i] = m_activeDynamicIds.back();
                m_activeDynamicIds.pop_back();
                break;
            }
        }
    }
    if (m_data[id].ccdEnabled)
        --m_ccdEnabledCount;
    m_data[id].active = false;
    m_data[id].shape.reset();
    ++m_data[id].generation;   // invalidate existing handles
    m_freeList.push_back(id);
    --m_activeCount;
}

// ── Body method implementations ───────────────────────────────────────────────

bool Body::isValid() const noexcept {
    return m_pool && m_pool->isValid(m_id, m_generation);
}

BodyType Body::type() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).type;
}

Transform Body::transform() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).transform;
}

vm::Vector3<float> Body::linearVelocity() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).linearVelocity;
}

vm::Vector3<float> Body::angularVelocity() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).angularVelocity;
}

float Body::mass() const noexcept {
    assert(isValid());
    const float inv = m_pool->get(m_id).invMass;
    return inv > 0.f ? 1.f / inv : 0.f;
}

float Body::invMass() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).invMass;
}

bool Body::isSleeping() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).isSleeping;
}

void Body::setTransform(const Transform& t) noexcept {
    assert(isValid());
    auto& d = m_pool->get(m_id);
    d.transform = t;
    d.isSleeping = false;
    d.sleepFrames = 0;
}

void Body::setLinearVelocity(const vm::Vector3<float>& v) noexcept {
    assert(isValid());
    auto& d = m_pool->get(m_id);
    d.linearVelocity = v;
    d.isSleeping = false;
    d.sleepFrames = 0;
}

void Body::setAngularVelocity(const vm::Vector3<float>& w) noexcept {
    assert(isValid());
    auto& d = m_pool->get(m_id);
    d.angularVelocity = w;
    d.isSleeping = false;
    d.sleepFrames = 0;
}

void Body::setLinearDamping(float damp) noexcept {
    assert(isValid());
    m_pool->get(m_id).linearDamping = damp;
}

void Body::setAngularDamping(float damp) noexcept {
    assert(isValid());
    m_pool->get(m_id).angularDamping = damp;
}

float Body::restitution() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).restitution;
}

float Body::friction() const noexcept {
    assert(isValid());
    return m_pool->get(m_id).friction;
}

void Body::setRestitution(float r) noexcept {
    assert(isValid());
    m_pool->get(m_id).restitution = r;
}

void Body::setFriction(float f) noexcept {
    assert(isValid());
    m_pool->get(m_id).friction = f;
}

void Body::applyForce(const vm::Vector3<float>& force) noexcept {
    assert(isValid());
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    d.forceAccum = d.forceAccum + force;
}

void Body::applyForceAt(const vm::Vector3<float>& force,
                        const vm::Vector3<float>& worldPoint) noexcept {
    assert(isValid());
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
    assert(isValid());
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    d.torqueAccum = d.torqueAccum + torque;
}

void Body::applyLinearImpulse(const vm::Vector3<float>& imp) noexcept {
    assert(isValid());
    auto& d = m_pool->get(m_id);
    if (d.type != BodyType::Dynamic) return;
    d.isSleeping = false;
    d.sleepFrames = 0;
    d.linearVelocity = d.linearVelocity + imp * d.invMass;
}

void Body::applyAngularImpulse(const vm::Vector3<float>& imp) noexcept {
    assert(isValid());
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
    assert(isValid());
    auto& d = m_pool->get(m_id);
    d.isSleeping  = false;
    d.sleepFrames = 0;
}

void Body::sleep() noexcept {
    assert(isValid());
    auto& d = m_pool->get(m_id);
    d.isSleeping      = true;
    d.sleepFrames     = 0;
    d.linearVelocity  = { 0.f, 0.f, 0.f };
    d.angularVelocity = { 0.f, 0.f, 0.f };
}

} // namespace campello::physics
