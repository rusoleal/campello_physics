#include "vehicle_system.h"
#include <cmath>

namespace campello::physics {

namespace {

inline float dot3(const vm::Vector3<float>& a, const vm::Vector3<float>& b) {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
}

inline vm::Vector3<float> cross3(const vm::Vector3<float>& a, const vm::Vector3<float>& b) {
    return { a.y()*b.z()-a.z()*b.y(), a.z()*b.x()-a.x()*b.z(), a.x()*b.y()-a.y()*b.x() };
}

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

} // namespace

// ── VehicleSystem ─────────────────────────────────────────────────────────────

VehicleBody VehicleSystem::create(const VehicleDescriptor& desc, Body chassisBody) {
    uint32_t id;
    if (!m_freeList.empty()) {
        id = m_freeList.back();
        m_freeList.pop_back();
    } else {
        id = static_cast<uint32_t>(m_records.size());
        m_records.emplace_back();
    }

    auto& rec        = m_records[id];
    rec.desc         = desc;
    rec.chassis      = chassisBody;
    rec.wheelStates.assign(desc.wheels.size(), WheelState{});
    rec.throttle     = 0.f;
    rec.braking      = 0.f;
    rec.steering     = 0.f;
    rec.active       = true;
    ++m_activeCount;

    return VehicleBody(id, chassisBody);
}

void VehicleSystem::destroy(VehicleBody vb) {
    if (!vb.isValid() || vb.id() >= m_records.size()) return;
    auto& rec = m_records[vb.id()];
    if (!rec.active) return;
    rec.active = false;
    m_freeList.push_back(vb.id());
    --m_activeCount;
}

void VehicleSystem::syncControls(const VehicleBody& vb) {
    if (!vb.isValid() || vb.id() >= m_records.size()) return;
    auto& rec    = m_records[vb.id()];
    rec.throttle = vb.throttle;
    rec.braking  = vb.braking;
    rec.steering = vb.steering;
}

const WheelState& VehicleSystem::wheelState(const VehicleBody& vb, int i) const noexcept {
    static const WheelState kDefault{};
    if (!vb.isValid() || vb.id() >= m_records.size()) return kDefault;
    const auto& rec = m_records[vb.id()];
    if (!rec.active || i < 0 || i >= static_cast<int>(rec.wheelStates.size())) return kDefault;
    return rec.wheelStates[i];
}

int VehicleSystem::wheelCount(const VehicleBody& vb) const noexcept {
    if (!vb.isValid() || vb.id() >= m_records.size()) return 0;
    return static_cast<int>(m_records[vb.id()].wheelStates.size());
}

void VehicleSystem::update(BodyPool& pool,
                            const IntegratorSettings& settings,
                            float dt,
                            const VehicleRaycastFn& raycast) {
    if (m_activeCount == 0) return;

    for (auto& rec : m_records) {
        if (!rec.active || !rec.chassis.isValid()) continue;
        if (!pool.isValid(rec.chassis.id())) continue;

        BodyData& chassis = pool.get(rec.chassis.id());
        if (chassis.type != BodyType::Dynamic) continue;

        for (int wi = 0; wi < static_cast<int>(rec.desc.wheels.size()); ++wi) {
            processWheel(chassis, rec, wi, dt, raycast);
        }

        chassis.isSleeping  = false;
        chassis.sleepFrames = 0;
        (void)settings;
    }
}

void VehicleSystem::processWheel(BodyData& chassis,
                                  VehicleRecord& rec,
                                  int wi,
                                  float dt,
                                  const VehicleRaycastFn& raycast) {
    const WheelDescriptor& wd = rec.desc.wheels[wi];
    WheelState& ws = rec.wheelStates[wi];

    const auto& rot = chassis.transform.rotation;
    const auto& pos = chassis.transform.position;

    // Wheel attachment in world space
    const auto attachWorld = pos + rot.rotated(wd.attachmentLocal);

    // Ray cast straight down in chassis-local space
    const auto downWorld = rot.rotated(vm::Vector3<float>(0.f, -1.f, 0.f));
    const float rayLen   = wd.restLength + wd.radius;

    Ray ray;
    ray.origin    = attachWorld;
    ray.direction = downWorld;

    QueryFilter filter;
    filter.layer         = 0xFFFFFFFF;
    filter.mask          = 0xFFFFFFFF;
    filter.excludeBodyId = rec.chassis.id();  // never hit the chassis itself

    auto hit = raycast(ray, filter);

    ws.isGrounded = hit.has_value() && hit->fraction <= rayLen;

    if (!ws.isGrounded) {
        ws.suspensionLength = wd.restLength;
        ws.suspensionForce  = 0.f;
        return;
    }

    const float dist = hit->fraction;
    ws.suspensionLength  = clampf(dist - wd.radius, 0.f, wd.restLength);
    ws.contactPoint      = hit->point;
    ws.contactNormal     = hit->normal;

    const float compression = wd.restLength - ws.suspensionLength;

    // ── Suspension spring-damper ───────────────────────────────────────────────

    const auto rA = attachWorld - pos;
    const auto velAtWheel = chassis.linearVelocity + cross3(chassis.angularVelocity, rA);
    const float suspVel   = dot3(velAtWheel, downWorld * -1.f);

    const float springF   = wd.stiffness * compression - wd.damping * suspVel;
    ws.suspensionForce    = springF > 0.f ? springF : 0.f;

    const auto upWorld  = downWorld * -1.f;
    const auto suspForceVec = upWorld * ws.suspensionForce;
    chassis.forceAccum  = chassis.forceAccum  + suspForceVec;
    chassis.torqueAccum = chassis.torqueAccum + cross3(rA, suspForceVec);

    if (ws.suspensionForce <= 0.f) return;

    // ── Steering ──────────────────────────────────────────────────────────────

    const float steerAngle = wd.maxSteerAngle > 0.f ? rec.steering * wd.maxSteerAngle : 0.f;
    // Z-forward convention: at steerAngle=0 the local forward is (0,0,1).
    // Rotating by steerAngle around local Y: fwd = (-sinθ, 0, cosθ).
    const auto  fwdLocal   = vm::Vector3<float>(-std::sin(steerAngle), 0.f, std::cos(steerAngle));
    const auto  fwdWorld   = rot.rotated(fwdLocal);
    const auto  rightWorld = cross3(fwdWorld, upWorld);

    // ── Drive (engine) ────────────────────────────────────────────────────────

    if (rec.throttle > 0.f) {
        const float driveF    = rec.throttle * rec.desc.maxEngineForce
                                / static_cast<float>(rec.desc.wheels.size());
        const auto  driveVec  = fwdWorld * driveF;
        chassis.forceAccum    = chassis.forceAccum  + driveVec;
        chassis.torqueAccum   = chassis.torqueAccum + cross3(rA, driveVec);
    }

    // ── Braking ───────────────────────────────────────────────────────────────

    if (rec.braking > 0.f) {
        const float velFwd  = dot3(velAtWheel, fwdWorld);
        const float brakeF  = -clampf(velFwd, -1.f, 1.f)
                              * rec.braking * rec.desc.maxBrakeForce
                              / static_cast<float>(rec.desc.wheels.size());
        const auto  brakeVec = fwdWorld * brakeF;
        chassis.forceAccum   = chassis.forceAccum  + brakeVec;
        chassis.torqueAccum  = chassis.torqueAccum + cross3(rA, brakeVec);
    }

    // ── Lateral friction ──────────────────────────────────────────────────────

    const float lateralVel    = dot3(velAtWheel, rightWorld);
    const float mass          = chassis.invMass > 1e-20f ? 1.f / chassis.invMass : 0.f;
    const float maxLateral    = wd.lateralFriction * ws.suspensionForce * dt;
    // Divide by wheel count: each wheel contributes 1/N of the total correction
    // so the combined impulse from all grounded wheels equals exactly -lateralVel*mass
    // (no overcorrection → no lateral oscillation).
    const float nWheels = static_cast<float>(rec.desc.wheels.size());
    float lateralImpulse      = -lateralVel * mass / nWheels;
    lateralImpulse            = clampf(lateralImpulse, -maxLateral, maxLateral);

    const auto  latVec       = rightWorld * (lateralImpulse / (dt > 1e-10f ? dt : 1e-10f));
    chassis.forceAccum       = chassis.forceAccum  + latVec;
    chassis.torqueAccum      = chassis.torqueAccum + cross3(rA, latVec);
}

} // namespace campello::physics
