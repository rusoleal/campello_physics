#pragma once

#include <campello_physics/aabb.h>
#include <vector>
#include <functional>

namespace campello::physics {

// Internal AABB BVH with dynamic insert/remove/update.
// Uses fat AABBs (tight AABB + margin) to amortize re-insertions.
// Sibling selection uses branch-and-bound (Bittner et al.) to minimize SA cost.
class DynamicBVH {
public:
    static constexpr float kFatMargin = 0.1f;
    static constexpr int   kNullNode  = -1;

    struct NodeId { int value = kNullNode; };

    // Insert a body. Returns a handle needed for remove/update.
    NodeId insert(int bodyId, const AABB& tightAABB);

    // Remove a body by its node handle.
    void remove(NodeId node);

    // Update AABB. Re-inserts only if tight AABB escapes the fat AABB. Returns true if re-inserted.
    bool update(NodeId& node, int bodyId, const AABB& tightAABB);

    // Call cb(bodyId) for every leaf whose fat AABB overlaps aabb.
    // excludeBodyId is skipped (pass -1 to skip nothing).
    void query(const AABB& aabb, int excludeBodyId, const std::function<void(int)>& cb) const;

    // Call cb(bodyId, fatAABB) for every leaf.
    void forEachLeaf(const std::function<void(int, const AABB&)>& cb) const;

    // Call cb(bodyId) for every leaf whose fat AABB is intersected by the ray.
    // invDir = componentwise 1/dir (precomputed by caller).
    void queryRay(const vm::Vector3<float>& origin,
                  const vm::Vector3<float>& invDir,
                  float maxT,
                  int excludeBodyId,
                  const std::function<void(int)>& cb) const;

    bool empty() const noexcept { return m_root == kNullNode; }
    int  nodeCount() const noexcept { return static_cast<int>(m_nodes.size()) - static_cast<int>(m_freeList.size()); }

private:
    struct Node {
        AABB fatAABB;
        int  parent  = kNullNode;
        int  left    = kNullNode;
        int  right   = kNullNode;
        int  bodyId  = -1;        // >= 0 only for leaves
        bool isLeaf() const noexcept { return bodyId >= 0; }
    };

    std::vector<Node> m_nodes;
    std::vector<int>  m_freeList;
    int               m_root = kNullNode;

    int  allocNode();
    void freeNode(int idx);
    int  findBestSibling(const AABB& leafAABB) const;
    void insertLeaf(int leafIdx);
    void removeLeaf(int leafIdx);
    void refitAncestors(int startIdx);

    static AABB merged(const AABB& a, const AABB& b) noexcept { return a.merged(b); }
    static AABB fatten(const AABB& a) noexcept {
        vm::Vector3<float> m(kFatMargin, kFatMargin, kFatMargin);
        return AABB::fromMinMax(a.min - m, a.max + m);
    }
};

} // namespace campello::physics
