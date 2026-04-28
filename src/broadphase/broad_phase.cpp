#include <campello_physics/broad_phase.h>
#include "dynamic_bvh.h"

#include <algorithm>
#include <cassert>
#include <iterator>

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
    if (id >= m_proxies.size()) m_proxies.resize(id + 1);
    assert(!m_proxies[id].valid && "body already inserted");

    DynamicBVH& tree = isStatic ? m_trees->staticTree : m_trees->dynamicTree;
    auto nodeId = tree.insert(static_cast<int>(id), aabb);
    m_proxies[id] = ProxyData{nodeId.value, layer, mask, isStatic, true};
}

void BroadPhase::removeBody(uint32_t id) {
    assert(id < m_proxies.size() && m_proxies[id].valid && "body not found");

    DynamicBVH& tree = m_proxies[id].isStatic ? m_trees->staticTree : m_trees->dynamicTree;
    tree.remove(DynamicBVH::NodeId{m_proxies[id].nodeValue});
    m_proxies[id].valid = false;
}

void BroadPhase::updateBody(uint32_t id, const AABB& aabb) {
    assert(id < m_proxies.size() && m_proxies[id].valid && "body not found");
    if (m_proxies[id].isStatic) return;  // static bodies never move

    DynamicBVH& tree = m_trees->dynamicTree;
    DynamicBVH::NodeId node{m_proxies[id].nodeValue};
    if (tree.update(node, static_cast<int>(id), aabb))
        m_proxies[id].nodeValue = node.value;
}

// ── Pair helpers ──────────────────────────────────────────────────────────────

void BroadPhase::addPairIfFiltered(uint32_t a, uint32_t b) {
    if (a == b) return;
    if (a >= m_proxies.size() || b >= m_proxies.size()) return;
    const ProxyData& pa = m_proxies[a];
    const ProxyData& pb = m_proxies[b];
    if (!pa.valid || !pb.valid) return;
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

    // Sort for set_difference. Skip unique — BVH queries generate each pair
    // exactly once (dynamic-dynamic uses idB > idA, dynamic-static are unique).
    std::sort(m_current.begin(), m_current.end());

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
        uint32_t id = static_cast<uint32_t>(bodyId);
        if (id >= m_proxies.size() || !m_proxies[id].valid) return;
        const auto& p = m_proxies[id];
        if ((filterLayer & p.mask) && (p.layer & filterMask))
            cb(id);
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
        uint32_t id = static_cast<uint32_t>(bodyId);
        if (id >= m_proxies.size() || !m_proxies[id].valid) return;
        const auto& p = m_proxies[id];
        if ((filterLayer & p.mask) && (p.layer & filterMask))
            cb(id);
    };
    m_trees->staticTree.query(aabb, -1, visit);
    m_trees->dynamicTree.query(aabb, -1, visit);
}

} // namespace campello::physics
