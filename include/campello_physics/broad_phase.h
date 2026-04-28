#pragma once

#include <campello_physics/aabb.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace campello::physics {

struct CollisionPair {
    uint32_t bodyA;  // always < bodyB
    uint32_t bodyB;

    bool operator==(const CollisionPair& o) const noexcept {
        return bodyA == o.bodyA && bodyB == o.bodyB;
    }
    bool operator<(const CollisionPair& o) const noexcept {
        return bodyA != o.bodyA ? bodyA < o.bodyA : bodyB < o.bodyB;
    }
};

// Manages broad-phase collision detection via two dynamic AABB BVHs
// (one for static bodies, one for dynamic/kinematic/sensor bodies).
// Tracks the pair lifecycle: added, persisted, removed each frame.
class BroadPhase {
public:
    BroadPhase();
    ~BroadPhase();  // defined in .cpp where BVHPair is complete

    // Register a body. isStatic = true means it never moves (static bucket).
    void insertBody(uint32_t id, const AABB& aabb, uint32_t layer, uint32_t mask, bool isStatic);

    // Unregister a body.
    void removeBody(uint32_t id);

    // Notify that a dynamic body's AABB changed. No-op for static bodies.
    void updateBody(uint32_t id, const AABB& aabb);

    // Recompute overlapping pairs and update lifecycle lists.
    // Call once per simulation step, before narrowphase.
    void computePairs();

    [[nodiscard]] const std::vector<CollisionPair>& currentPairs() const noexcept { return m_current; }
    [[nodiscard]] const std::vector<CollisionPair>& addedPairs()   const noexcept { return m_added; }
    [[nodiscard]] const std::vector<CollisionPair>& removedPairs() const noexcept { return m_removed; }

    // Query bodies (from both static and dynamic trees) whose AABB is hit by the ray.
    // Only bodies passing (filter.layer & body.mask) && (body.layer & filter.mask) are reported.
    void queryRay(const vm::Vector3<float>& origin,
                  const vm::Vector3<float>& invDir,
                  float maxT,
                  uint32_t filterLayer, uint32_t filterMask,
                  const std::function<void(uint32_t bodyId)>& cb) const;

    // Query bodies (from both trees) whose AABB overlaps the given AABB.
    void queryAABB(const AABB& aabb,
                   uint32_t filterLayer, uint32_t filterMask,
                   const std::function<void(uint32_t bodyId)>& cb) const;

private:
    // Forward-declare the BVH types to avoid exposing the internal header.
    struct Impl;
    struct ProxyData {
        int      nodeValue = -1;  // DynamicBVH::NodeId.value, -1 = invalid
        uint32_t layer     = 0xFFFFFFFF;
        uint32_t mask      = 0xFFFFFFFF;
        bool     isStatic  = false;
        bool     valid     = false;
    };

    std::vector<ProxyData> m_proxies;
    std::vector<CollisionPair>              m_previous;
    std::vector<CollisionPair>              m_current;
    std::vector<CollisionPair>              m_added;
    std::vector<CollisionPair>              m_removed;

    // Opaque BVH storage — allocated in the .cpp.
    struct BVHPair;
    std::unique_ptr<BVHPair> m_trees;

    void addPairIfFiltered(uint32_t a, uint32_t b);
};

} // namespace campello::physics
