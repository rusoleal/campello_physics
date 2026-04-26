#include <campello_physics/broad_phase.h>
#include "dynamic_bvh.h"

#include <algorithm>
#include <cassert>

namespace campello::physics {

// ── PIMPL storage ─────────────────────────────────────────────────────────────

struct BroadPhase::BVHPair {
    DynamicBVH staticTree;
    DynamicBVH dynamicTree;
};

BroadPhase::BroadPhase()  : m_trees(std::make_unique<BVHPair>()) {}
BroadPhase::~BroadPhase() = default;

// ── Body management ───────────────────────────────────────────────────────────

void BroadPhase::insertBody(uint32_t id, const AABB& aabb,
                             uint32_t layer, uint32_t mask, bool isStatic)
{
    assert(m_proxies.find(id) == m_proxies.end() && "body already inserted");

    DynamicBVH& tree = isStatic ? m_trees->staticTree : m_trees->dynamicTree;
    auto nodeId = tree.insert(static_cast<int>(id), aabb);
    m_proxies[id] = ProxyData{nodeId.value, layer, mask, isStatic};
}

void BroadPhase::removeBody(uint32_t id) {
    auto it = m_proxies.find(id);
    assert(it != m_proxies.end() && "body not found");

    DynamicBVH& tree = it->second.isStatic ? m_trees->staticTree : m_trees->dynamicTree;
    tree.remove(DynamicBVH::NodeId{it->second.nodeValue});
    m_proxies.erase(it);
}

void BroadPhase::updateBody(uint32_t id, const AABB& aabb) {
    auto it = m_proxies.find(id);
    assert(it != m_proxies.end() && "body not found");
    if (it->second.isStatic) return;  // static bodies never move

    DynamicBVH& tree = m_trees->dynamicTree;
    DynamicBVH::NodeId node{it->second.nodeValue};
    if (tree.update(node, static_cast<int>(id), aabb))
        it->second.nodeValue = node.value;
}

// ── Pair helpers ──────────────────────────────────────────────────────────────

void BroadPhase::addPairIfFiltered(uint32_t a, uint32_t b) {
    if (a == b) return;
    const ProxyData& pa = m_proxies.at(a);
    const ProxyData& pb = m_proxies.at(b);
    // Layer/mask filter: both bodies must accept each other's layer.
    if (!(pa.layer & pb.mask) || !(pb.layer & pa.mask)) return;
    // Canonical ordering
    if (a > b) std::swap(a, b);
    m_current.push_back({a, b});
}

// ── Main update ───────────────────────────────────────────────────────────────

void BroadPhase::computePairs() {
    m_previous = std::move(m_current);
    m_current.clear();
    m_added.clear();
    m_removed.clear();

    // Dynamic vs Static
    m_trees->dynamicTree.forEachLeaf([&](int dynId, const AABB& dynAABB) {
        m_trees->staticTree.query(dynAABB, -1, [&](int statId) {
            addPairIfFiltered(static_cast<uint32_t>(dynId), static_cast<uint32_t>(statId));
        });
    });

    // Dynamic vs Dynamic: each leaf queries the tree, only record bodyId > self to avoid dups.
    m_trees->dynamicTree.forEachLeaf([&](int idA, const AABB& aabbA) {
        m_trees->dynamicTree.query(aabbA, idA, [&](int idB) {
            if (idB > idA)
                addPairIfFiltered(static_cast<uint32_t>(idA), static_cast<uint32_t>(idB));
        });
    });

    // Deduplicate (a body can appear in multiple leaves after re-insertion edge cases).
    std::sort(m_current.begin(), m_current.end());
    m_current.erase(std::unique(m_current.begin(), m_current.end()), m_current.end());

    // Compute added / removed by diffing sorted lists.
    std::set_difference(m_current.begin(),  m_current.end(),
                        m_previous.begin(), m_previous.end(),
                        std::back_inserter(m_added));

    std::set_difference(m_previous.begin(), m_previous.end(),
                        m_current.begin(),  m_current.end(),
                        std::back_inserter(m_removed));
}

void BroadPhase::queryRay(
    const vm::Vector3<float>& origin,
    const vm::Vector3<float>& invDir,
    float maxT,
    uint32_t filterLayer, uint32_t filterMask,
    const std::function<void(uint32_t)>& cb) const
{
    auto visit = [&](int bodyId) {
        auto it = m_proxies.find(static_cast<uint32_t>(bodyId));
        if (it == m_proxies.end()) return;
        const auto& p = it->second;
        if ((filterLayer & p.mask) && (p.layer & filterMask))
            cb(static_cast<uint32_t>(bodyId));
    };
    m_trees->staticTree.queryRay(origin, invDir, maxT, -1, visit);
    m_trees->dynamicTree.queryRay(origin, invDir, maxT, -1, visit);
}

void BroadPhase::queryAABB(
    const AABB& aabb,
    uint32_t filterLayer, uint32_t filterMask,
    const std::function<void(uint32_t)>& cb) const
{
    auto visit = [&](int bodyId) {
        auto it = m_proxies.find(static_cast<uint32_t>(bodyId));
        if (it == m_proxies.end()) return;
        const auto& p = it->second;
        if ((filterLayer & p.mask) && (p.layer & filterMask))
            cb(static_cast<uint32_t>(bodyId));
    };
    m_trees->staticTree.query(aabb, -1, visit);
    m_trees->dynamicTree.query(aabb, -1, visit);
}

} // namespace campello::physics
