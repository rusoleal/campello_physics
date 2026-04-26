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
    void process(
        const std::vector<CollisionPair>& pairs,
        const std::function<ShapeInstance(uint32_t bodyId)>& getShape,
        int numThreads = 1);

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

} // namespace campello::physics
