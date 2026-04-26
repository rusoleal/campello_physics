#include "buoyancy_system.h"
#include <campello_physics/shapes/sphere_shape.h>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace campello::physics {

namespace {

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Volume of a spherical cap of height h in a sphere of radius r.
// V = π × h² × (3r − h) / 3
inline float sphericalCapVolume(float r, float h) {
    h = clampf(h, 0.f, 2.f * r);
    return (std::numbers::pi_v<float> * h * h * (3.f * r - h)) / 3.f;
}

} // namespace

// ── BuoyancySystem ────────────────────────────────────────────────────────────

BuoyancyVolume BuoyancySystem::add(const BuoyancyDescriptor& desc) {
    uint32_t id;
    if (!m_freeList.empty()) {
        id = m_freeList.back();
        m_freeList.pop_back();
        m_volumes[id] = { desc, true };
    } else {
        id = static_cast<uint32_t>(m_volumes.size());
        m_volumes.push_back({ desc, true });
    }
    ++m_activeCount;
    return BuoyancyVolume(id);
}

void BuoyancySystem::remove(BuoyancyVolume vol) {
    if (!vol.isValid() || vol.id() >= m_volumes.size()) return;
    auto& v = m_volumes[vol.id()];
    if (!v.active) return;
    v.active = false;
    m_freeList.push_back(vol.id());
    --m_activeCount;
}

// Returns the fraction [0,1] of the body's shape volume that sits below fluidTopY.
float BuoyancySystem::submersionFraction(const BodyData& body,
                                          const BuoyancyVolumeData& /*vol*/,
                                          float fluidTopY) {
    if (!body.shape) return 0.f;

    const float cy = body.transform.position.y();

    if (body.shape->type() == ShapeType::Sphere) {
        const auto* sp = static_cast<const SphereShape*>(body.shape.get());
        const float r  = sp->radius();
        const float h  = clampf(fluidTopY - (cy - r), 0.f, 2.f * r);
        if (h <= 0.f) return 0.f;
        if (h >= 2.f * r) return 1.f;
        const float totalVol = (4.f / 3.f) * std::numbers::pi_v<float> * r * r * r;
        return sphericalCapVolume(r, h) / totalVol;
    }

    // Fallback: AABB intersection fraction for non-sphere shapes.
    const AABB bodyAABB = body.shape->computeAABB(body.transform);
    const float bodyMinY = bodyAABB.min.y();
    const float bodyMaxY = bodyAABB.max.y();
    if (fluidTopY <= bodyMinY) return 0.f;
    if (fluidTopY >= bodyMaxY) return 1.f;
    return (fluidTopY - bodyMinY) / (bodyMaxY - bodyMinY);
}

void BuoyancySystem::applyForces(BodyPool& pool,
                                  const IntegratorSettings& settings,
                                  float dt) const {
    if (m_activeCount == 0) return;

    const float gravY = settings.gravity.y();
    if (gravY >= 0.f) return;
    const float gravMag = std::abs(gravY);

    for (const auto& vol : m_volumes) {
        if (!vol.active || !vol.desc.shape) continue;

        const AABB volAABB    = vol.desc.shape->computeAABB(vol.desc.transform);
        const float fluidTopY = volAABB.max.y();
        const float fluidMinY = volAABB.min.y();
        const float fluidMinX = volAABB.min.x();
        const float fluidMaxX = volAABB.max.x();
        const float fluidMinZ = volAABB.min.z();
        const float fluidMaxZ = volAABB.max.z();

        pool.forEach([&](uint32_t /*id*/, BodyData& d) {
            if (d.type != BodyType::Dynamic || d.isSleeping) return;
            if (!d.shape) return;

            const AABB bodyAABB = d.shape->computeAABB(d.transform);
            if (bodyAABB.min.y() >= fluidTopY) return;
            if (bodyAABB.max.y() <= fluidMinY) return;
            if (bodyAABB.max.x() < fluidMinX || bodyAABB.min.x() > fluidMaxX) return;
            if (bodyAABB.max.z() < fluidMinZ || bodyAABB.min.z() > fluidMaxZ) return;

            const float frac = submersionFraction(d, vol, fluidTopY);
            if (frac <= 0.f) return;

            float bodyVol;
            if (d.shape->type() == ShapeType::Sphere) {
                const auto* sp = static_cast<const SphereShape*>(d.shape.get());
                const float r  = sp->radius();
                bodyVol = (4.f / 3.f) * std::numbers::pi_v<float> * r * r * r;
            } else {
                const float bExtX = (bodyAABB.max.x() - bodyAABB.min.x()) * 0.5f;
                const float bExtY = (bodyAABB.max.y() - bodyAABB.min.y()) * 0.5f;
                const float bExtZ = (bodyAABB.max.z() - bodyAABB.min.z()) * 0.5f;
                bodyVol = 8.f * bExtX * bExtY * bExtZ;
            }

            const float submergedVol    = bodyVol * frac;
            const float buoyantForceMag = vol.desc.fluidDensity * gravMag * submergedVol;
            d.forceAccum = d.forceAccum + vm::Vector3<float>(0.f, buoyantForceMag, 0.f);

            const float mass   = (d.invMass > 1e-20f) ? 1.f / d.invMass : 0.f;
            const float linDrg = vol.desc.linearDrag  * frac * mass;
            const float angDrg = vol.desc.angularDrag * frac * mass;
            d.forceAccum  = d.forceAccum  - d.linearVelocity  * linDrg;
            d.torqueAccum = d.torqueAccum - d.angularVelocity * angDrg;

            d.isSleeping  = false;
            d.sleepFrames = 0;

            (void)dt;
        });
    }
}

} // namespace campello::physics
