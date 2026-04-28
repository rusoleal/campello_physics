#include <campello_physics/narrow_phase.h>
#include "gjk.h"
#include "sat.h"
#include "narrowphase_utils.h"
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <cstdint>
#include <optional>
#include <thread>
#include <vector>

namespace campello::physics {

// ── Dispatch table ────────────────────────────────────────────────────────────

std::optional<ContactManifold> collide(const ShapeInstance& a, const ShapeInstance& b)
{
    if (!a.shape || !b.shape) return std::nullopt;

    const ShapeType ta = a.shape->type();
    const ShapeType tb = b.shape->type();

    // ── Sphere vs. * ──────────────────────────────────────────────────────────
    if (ta == ShapeType::Sphere && tb == ShapeType::Sphere) {
        return detail::collideSpheres(
            static_cast<const SphereShape&>(*a.shape), a.transform,
            static_cast<const SphereShape&>(*b.shape), b.transform);
    }

    if (ta == ShapeType::Sphere && tb == ShapeType::Box) {
        return detail::collideSphereBox(
            static_cast<const SphereShape&>(*a.shape), a.transform,
            static_cast<const BoxShape&>(*b.shape),    b.transform,
            /*sphereIsA=*/true);
    }
    if (ta == ShapeType::Box && tb == ShapeType::Sphere) {
        return detail::collideSphereBox(
            static_cast<const SphereShape&>(*b.shape), b.transform,
            static_cast<const BoxShape&>(*a.shape),    a.transform,
            /*sphereIsA=*/false);
    }

    if (ta == ShapeType::Sphere && tb == ShapeType::Capsule) {
        return detail::collideSphereCapsule(
            static_cast<const SphereShape&>(*a.shape),  a.transform,
            static_cast<const CapsuleShape&>(*b.shape), b.transform,
            /*sphereIsA=*/true);
    }
    if (ta == ShapeType::Capsule && tb == ShapeType::Sphere) {
        return detail::collideSphereCapsule(
            static_cast<const SphereShape&>(*b.shape),  b.transform,
            static_cast<const CapsuleShape&>(*a.shape), a.transform,
            /*sphereIsA=*/false);
    }

    // ── Box vs. Box ───────────────────────────────────────────────────────────
    if (ta == ShapeType::Box && tb == ShapeType::Box) {
        return detail::collideBoxBox(
            static_cast<const BoxShape&>(*a.shape), a.transform,
            static_cast<const BoxShape&>(*b.shape), b.transform);
    }

    // ── Capsule vs. Capsule ───────────────────────────────────────────────────
    if (ta == ShapeType::Capsule && tb == ShapeType::Capsule) {
        return detail::collideCapsuleCapsule(
            static_cast<const CapsuleShape&>(*a.shape), a.transform,
            static_cast<const CapsuleShape&>(*b.shape), b.transform);
    }

    // ── Non-convex shapes: not directly supported with GJK ────────────────────
    if (ta == ShapeType::TriangleMesh || ta == ShapeType::HeightField ||
        tb == ShapeType::TriangleMesh || tb == ShapeType::HeightField)
    {
        return std::nullopt; // narrowphase for triangle mesh / height field in Phase 8
    }

    // ── Fall-back: GJK + EPA for any remaining convex pair ───────────────────
    return detail::collideConvex(a, b);
}

// ── NarrowPhase batch processing ─────────────────────────────────────────────

void NarrowPhase::copyWarmStart(ContactManifold& newM, const ContactManifold& oldM) const
{
    for (int i = 0; i < newM.count; ++i) {
        auto& np       = newM.points[i];
        float bestDist2 = kCacheThreshold * kCacheThreshold;
        for (int j = 0; j < oldM.count; ++j) {
            const auto& op = oldM.points[j];
            const detail::V3 d = np.position - op.position;
            const float dist2  = detail::lenSq3(d);
            if (dist2 < bestDist2) {
                bestDist2              = dist2;
                np.warmStartImpulse   = op.warmStartImpulse;
                np.warmStartFriction0 = op.warmStartFriction0;
                np.warmStartFriction1 = op.warmStartFriction1;
            }
        }
    }
}

} // namespace campello::physics
