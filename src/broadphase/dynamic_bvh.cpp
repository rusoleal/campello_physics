#include "dynamic_bvh.h"
#include <cassert>
#include <cmath>
#include <vector>

namespace campello::physics {

int DynamicBVH::allocNode() {
    if (!m_freeList.empty()) {
        int idx = m_freeList.back();
        m_freeList.pop_back();
        m_nodes[idx] = Node{};
        return idx;
    }
    m_nodes.emplace_back();
    return static_cast<int>(m_nodes.size()) - 1;
}

void DynamicBVH::freeNode(int idx) {
    assert(idx >= 0 && idx < static_cast<int>(m_nodes.size()));
    m_nodes[idx].bodyId = -1;
    m_freeList.push_back(idx);
}

// Branch-and-bound sibling selection — minimises total SA cost of insertion.
int DynamicBVH::findBestSibling(const AABB& leafAABB) const {
    assert(m_root != kNullNode);

    struct Candidate { int idx; float inheritedCost; };
    std::vector<Candidate> stack;
    stack.reserve(32);
    stack.push_back({m_root, 0.f});

    float bestCost = merged(m_nodes[m_root].fatAABB, leafAABB).halfSurfaceArea();
    int   best     = m_root;

    while (!stack.empty()) {
        auto [idx, iCost] = stack.back();
        stack.pop_back();

        const Node& n  = m_nodes[idx];
        float direct   = merged(n.fatAABB, leafAABB).halfSurfaceArea();
        float total    = direct + iCost;

        if (total < bestCost) {
            bestCost = total;
            best     = idx;
        }

        if (!n.isLeaf()) {
            // Lower bound for any descendant: leaf SA + inherited cost after this node.
            float childInherited = iCost + (direct - n.fatAABB.halfSurfaceArea());
            float lowerBound     = leafAABB.halfSurfaceArea() + childInherited;
            if (lowerBound < bestCost) {
                stack.push_back({n.left,  childInherited});
                stack.push_back({n.right, childInherited});
            }
        }
    }

    return best;
}

void DynamicBVH::insertLeaf(int leaf) {
    if (m_root == kNullNode) {
        m_root = leaf;
        m_nodes[leaf].parent = kNullNode;
        return;
    }

    int sibling   = findBestSibling(m_nodes[leaf].fatAABB);
    int oldParent = m_nodes[sibling].parent;
    int newParent = allocNode();

    m_nodes[newParent].parent  = oldParent;
    m_nodes[newParent].fatAABB = merged(m_nodes[sibling].fatAABB, m_nodes[leaf].fatAABB);
    m_nodes[newParent].left    = sibling;
    m_nodes[newParent].right   = leaf;

    if (oldParent == kNullNode) {
        m_root = newParent;
    } else {
        if (m_nodes[oldParent].left == sibling)
            m_nodes[oldParent].left = newParent;
        else
            m_nodes[oldParent].right = newParent;
    }

    m_nodes[sibling].parent = newParent;
    m_nodes[leaf].parent    = newParent;

    refitAncestors(newParent);
}

void DynamicBVH::removeLeaf(int leaf) {
    if (leaf == m_root) {
        m_root = kNullNode;
        freeNode(leaf);
        return;
    }

    int parent   = m_nodes[leaf].parent;
    int sibling  = (m_nodes[parent].left == leaf) ? m_nodes[parent].right : m_nodes[parent].left;
    int grandParent = m_nodes[parent].parent;

    if (grandParent == kNullNode) {
        m_root = sibling;
        m_nodes[sibling].parent = kNullNode;
    } else {
        if (m_nodes[grandParent].left == parent)
            m_nodes[grandParent].left = sibling;
        else
            m_nodes[grandParent].right = sibling;
        m_nodes[sibling].parent = grandParent;
        refitAncestors(grandParent);
    }

    freeNode(parent);
    freeNode(leaf);
}

void DynamicBVH::refitAncestors(int startIdx) {
    int idx = startIdx;
    while (idx != kNullNode) {
        Node& n = m_nodes[idx];
        if (!n.isLeaf())
            n.fatAABB = merged(m_nodes[n.left].fatAABB, m_nodes[n.right].fatAABB);
        idx = n.parent;
    }
}

DynamicBVH::NodeId DynamicBVH::insert(int bodyId, const AABB& tightAABB) {
    int leaf = allocNode();
    m_nodes[leaf].fatAABB = fatten(tightAABB);
    m_nodes[leaf].bodyId  = bodyId;
    insertLeaf(leaf);
    return NodeId{leaf};
}

void DynamicBVH::remove(NodeId node) {
    assert(node.value != kNullNode);
    removeLeaf(node.value);
}

bool DynamicBVH::update(NodeId& node, int bodyId, const AABB& tightAABB) {
    assert(node.value != kNullNode);
    // Only re-insert if body AABB has escaped its fat AABB.
    if (m_nodes[node.value].fatAABB.contains(tightAABB))
        return false;

    // remove() frees node.value onto the free list. Do NOT reuse that slot directly:
    // insertLeaf() calls allocNode() which would pop the same slot as the new internal
    // parent, creating a self-referencing cycle. Use a fresh insert instead.
    remove(node);
    node = insert(bodyId, tightAABB);
    return true;
}

} // namespace campello::physics
