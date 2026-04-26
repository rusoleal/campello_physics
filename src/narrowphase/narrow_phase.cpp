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

void NarrowPhase::process(
    const std::vector<CollisionPair>& pairs,
    const std::function<ShapeInstance(uint32_t)>& getShape,
    int numThreads)
{
    m_manifolds.clear();
    std::unordered_map<uint64_t, ContactManifold> newCache;

    // Minimum pair count to justify thread overhead
    static constexpr int kParallelThreshold = 32;

    const int n = static_cast<int>(pairs.size());
    const bool useParallel = numThreads > 1 && n >= kParallelThreshold;

    // Pre-allocate result slots (one per pair, nullopt = no contact)
    std::vector<std::optional<ContactManifold>> results(
        useParallel ? static_cast<size_t>(n) : 0);

    if (useParallel) {
        // Distribute pairs across worker threads.
        // Each thread writes only to its own result slots — no locks needed.
        auto worker = [&](int first, int last) {
            for (int i = first; i < last; ++i) {
                auto r = collide(getShape(pairs[i].bodyA), getShape(pairs[i].bodyB));
                if (r) { r->bodyA = pairs[i].bodyA; r->bodyB = pairs[i].bodyB; }
                results[static_cast<size_t>(i)] = std::move(r);
            }
        };

        const int threads = std::min(numThreads, n);
        const int chunk   = (n + threads - 1) / threads;

        std::vector<std::thread> workers;
        workers.reserve(static_cast<size_t>(threads - 1));
        for (int t = 0; t < threads - 1; ++t)
            workers.emplace_back(worker, t * chunk, std::min((t + 1) * chunk, n));
        worker((threads - 1) * chunk, n);  // last chunk on calling thread
        for (auto& th : workers) th.join();

        // Collect results serially (warm-start and cache update must be sequential)
        for (int i = 0; i < n; ++i) {
            if (!results[static_cast<size_t>(i)]) continue;
            ContactManifold& m = *results[static_cast<size_t>(i)];
            const uint64_t key = (static_cast<uint64_t>(m.bodyA) << 32) | m.bodyB;
            const auto it = m_cache.find(key);
            if (it != m_cache.end()) copyWarmStart(m, it->second);
            newCache[key] = m;
            m_manifolds.push_back(std::move(m));
        }
    } else {
        // Serial path (original behaviour)
        for (const auto& pair : pairs) {
            auto result = collide(getShape(pair.bodyA), getShape(pair.bodyB));
            if (!result) continue;
            result->bodyA = pair.bodyA;
            result->bodyB = pair.bodyB;
            const uint64_t key = (static_cast<uint64_t>(pair.bodyA) << 32) | pair.bodyB;
            const auto it = m_cache.find(key);
            if (it != m_cache.end()) copyWarmStart(*result, it->second);
            newCache[key] = *result;
            m_manifolds.push_back(std::move(*result));
        }
    }

    m_cache = std::move(newCache);
}

} // namespace campello::physics
