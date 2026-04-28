#pragma once

#include <campello_physics/contact.h>
#include <campello_physics/shape.h>
#include <campello_physics/transform.h>
#include <campello_physics/broad_phase.h>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

namespace campello::physics {

// A shape placed in the world with a specific transform.
struct ShapeInstance {
    const Shape* shape     = nullptr;
    Transform    transform = Transform::identity();
};

// One-shot collision detection dispatch.
// Returns a manifold if the shapes overlap, nullopt if they are separated.
// The manifold normal always points from B toward A.
// bodyA/bodyB in the returned manifold are left at 0; the caller sets them.
[[nodiscard]] std::optional<ContactManifold>
collide(const ShapeInstance& a, const ShapeInstance& b);

// Batch narrow phase with persistent contact caching.
// Feed it the broad-phase pair list every step; it fills a manifold list and
// copies warm-start impulses from the previous frame's matching contacts.
class NarrowPhase {
public:
    // Process all collision pairs.
    // getShape: given a body ID, returns the ShapeInstance for that body.
    // numThreads: 1 = serial (default); >1 = collide() calls run in parallel.
    // parallelFor: optional callback that executes a parallel-for loop. When
    // provided, numThreads is ignored and the caller's thread pool is used
    // instead of spawning temporary std::thread objects every step.
    template<typename GetShapeFn>
    void process(
        const std::vector<CollisionPair>& pairs,
        GetShapeFn&& getShape,
        int numThreads = 1);

    using ParallelForFn = std::function<void(int total, const std::function<void(int first, int last)>&)>;
    template<typename GetShapeFn>
    void process(
        const std::vector<CollisionPair>& pairs,
        GetShapeFn&& getShape,
        int numThreads,
        const ParallelForFn& parallelFor);

    [[nodiscard]] const std::vector<ContactManifold>& manifolds() const noexcept {
        return m_manifolds;
    }
    [[nodiscard]] std::vector<ContactManifold>& manifolds() noexcept {
        return m_manifolds;
    }

    // Call at the start of each step before process().
    void clearManifolds() { m_manifolds.clear(); }

private:
    // Contact points within this distance are considered "the same" for warm starting.
    static constexpr float kCacheThreshold = 0.04f;

    std::unordered_map<uint64_t, ContactManifold> m_cache;
    std::vector<ContactManifold>                   m_manifolds;

    void copyWarmStart(ContactManifold& newM, const ContactManifold& oldM) const;
};

// ── Templated process implementations (must be in header for inlining) ─────────

template<typename GetShapeFn>
inline void NarrowPhase::process(
    const std::vector<CollisionPair>& pairs,
    GetShapeFn&& getShape,
    int numThreads)
{
    process(pairs, std::forward<GetShapeFn>(getShape), numThreads, nullptr);
}

template<typename GetShapeFn>
inline void NarrowPhase::process(
    const std::vector<CollisionPair>& pairs,
    GetShapeFn&& getShape,
    int /*numThreads*/,
    const ParallelForFn& parallelFor)
{
    m_manifolds.clear();
    if (pairs.empty()) {
        m_cache.clear();
        return;
    }

    std::unordered_map<uint64_t, ContactManifold> newCache;

    const int n = static_cast<int>(pairs.size());
    const bool useParallel = parallelFor && n >= 32;

    if (useParallel) {
        std::vector<std::optional<ContactManifold>> results(static_cast<size_t>(n));

        auto worker = [&](int first, int last) {
            for (int i = first; i < last; ++i) {
                auto r = collide(getShape(pairs[i].bodyA), getShape(pairs[i].bodyB));
                if (r) { r->bodyA = pairs[i].bodyA; r->bodyB = pairs[i].bodyB; }
                results[static_cast<size_t>(i)] = std::move(r);
            }
        };

        parallelFor(n, worker);

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
