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
    static constexpr float kFatMargin = 0.5f;
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
    template<typename Fn>
    void query(const AABB& aabb, int excludeBodyId, Fn&& cb) const;

    // Call cb(bodyId, fatAABB) for every leaf.
    template<typename Fn>
    void forEachLeaf(Fn&& cb) const;

    // Call cb(bodyId) for every leaf whose fat AABB is intersected by the ray.
    // invDir = componentwise 1/dir (precomputed by caller).
    template<typename Fn>
    void queryRay(const vm::Vector3<float>& origin,
                  const vm::Vector3<float>& invDir,
                  float maxT,
                  int excludeBodyId,
                  Fn&& cb) const;

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

// ── Templated query implementations (inlined for callback specialization) ─────

template<typename Fn>
inline void DynamicBVH::query(const AABB& aabb, int excludeBodyId, Fn&& cb) const {
    if (m_root == kNullNode) return;
    std::vector<int> stack;
    stack.reserve(32);
    stack.push_back(m_root);
    while (!stack.empty()) {
        int idx = stack.back(); stack.pop_back();
        const Node& n = m_nodes[idx];
        if (!n.fatAABB.intersects(aabb)) continue;
        if (n.isLeaf()) {
            if (n.bodyId != excludeBodyId)
                cb(n.bodyId);
        } else {
            stack.push_back(n.left);
            stack.push_back(n.right);
        }
    }
}

template<typename Fn>
inline void DynamicBVH::forEachLeaf(Fn&& cb) const {
    if (m_root == kNullNode) return;
    std::vector<int> stack;
    stack.reserve(32);
    stack.push_back(m_root);
    while (!stack.empty()) {
        int idx = stack.back(); stack.pop_back();
        const Node& n = m_nodes[idx];
        if (n.isLeaf())
            cb(n.bodyId, n.fatAABB);
        else {
            stack.push_back(n.left);
            stack.push_back(n.right);
        }
    }
}

namespace detail {
inline bool rayAABB(const vm::Vector3<float>& origin,
                    const vm::Vector3<float>& invDir,
                    float maxT,
                    const AABB& aabb) noexcept {
    float tNear = 0.f, tFar = maxT;
    for (int i = 0; i < 3; ++i) {
        float o = (i == 0) ? origin.x() : (i == 1 ? origin.y() : origin.z());
        float d = (i == 0) ? invDir.x()  : (i == 1 ? invDir.y()  : invDir.z());
        float mn = (i == 0) ? aabb.min.x() : (i == 1 ? aabb.min.y() : aabb.min.z());
        float mx = (i == 0) ? aabb.max.x() : (i == 1 ? aabb.max.y() : aabb.max.z());
        float t1 = (mn - o) * d;
        float t2 = (mx - o) * d;
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
        tNear = tNear > t1 ? tNear : t1;
        tFar  = tFar  < t2 ? tFar  : t2;
        if (tFar < tNear) return false;
    }
    return tFar >= 0.f && tNear <= maxT;
}
} // namespace detail

template<typename Fn>
inline void DynamicBVH::queryRay(const vm::Vector3<float>& origin,
                                  const vm::Vector3<float>& invDir,
                                  float maxT,
                                  int excludeBodyId,
                                  Fn&& cb) const {
    if (m_root == kNullNode) return;
    std::vector<int> stack;
    stack.reserve(32);
    stack.push_back(m_root);
    while (!stack.empty()) {
        int idx = stack.back(); stack.pop_back();
        const Node& n = m_nodes[idx];
        if (!detail::rayAABB(origin, invDir, maxT, n.fatAABB)) continue;
        if (n.isLeaf()) {
            if (n.bodyId != excludeBodyId)
                cb(n.bodyId);
        } else {
            stack.push_back(n.left);
            stack.push_back(n.right);
        }
    }
}

} // namespace campello::physics
