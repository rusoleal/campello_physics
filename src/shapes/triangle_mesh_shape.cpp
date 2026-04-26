#include <campello_physics/shapes/triangle_mesh_shape.h>
#include <algorithm>
#include <cassert>
#include <cmath>

namespace campello::physics {

TriangleMeshShape::TriangleMeshShape(std::vector<vm::Vector3<float>> vertices,
                                     std::vector<std::array<uint32_t, 3>> indices)
{
    assert(!indices.empty());
    m_tris.reserve(indices.size());
    m_localAABB = AABB::empty();

    for (const auto& tri : indices) {
        Triangle t;
        t.v[0] = vertices[tri[0]];
        t.v[1] = vertices[tri[1]];
        t.v[2] = vertices[tri[2]];
        m_tris.push_back(t);
        m_localAABB.expand(t.v[0]);
        m_localAABB.expand(t.v[1]);
        m_localAABB.expand(t.v[2]);
    }

    // Build BVH root over all triangles.
    m_bvh.emplace_back();
    buildBVH(0, 0, static_cast<int>(m_tris.size()));
}

AABB TriangleMeshShape::computeAABB(const Transform& t) const noexcept {
    // Transform the precomputed local AABB corners into world space.
    // Use the OBB→AABB formula on the local AABB.
    auto he = m_localAABB.halfExtents();
    auto center = t.transformPoint(m_localAABB.center());
    auto ax = t.rotation.rotated(vm::Vector3<float>(he.x(), 0.f, 0.f));
    auto ay = t.rotation.rotated(vm::Vector3<float>(0.f, he.y(), 0.f));
    auto az = t.rotation.rotated(vm::Vector3<float>(0.f, 0.f, he.z()));
    vm::Vector3<float> worldHe(
        std::fabs(ax.x()) + std::fabs(ay.x()) + std::fabs(az.x()),
        std::fabs(ax.y()) + std::fabs(ay.y()) + std::fabs(az.y()),
        std::fabs(ax.z()) + std::fabs(ay.z()) + std::fabs(az.z())
    );
    return AABB::fromCenterHalfExtents(center, worldHe);
}

void TriangleMeshShape::buildBVH(int nodeIdx, int triStart, int triCount) {
    // Compute node AABB.
    AABB nodeAABB = AABB::empty();
    for (int i = triStart; i < triStart + triCount; ++i) {
        nodeAABB.expand(m_tris[i].v[0]);
        nodeAABB.expand(m_tris[i].v[1]);
        nodeAABB.expand(m_tris[i].v[2]);
    }
    m_bvh[nodeIdx].aabb = nodeAABB;

    if (triCount <= 4) {
        // Leaf node.
        m_bvh[nodeIdx].triStart = triStart;
        m_bvh[nodeIdx].triCount = triCount;
        return;
    }

    // Split along the longest AABB axis by centroid median.
    auto extents = nodeAABB.extents();
    int  axis    = 0;
    if (extents.y() > extents.x()) axis = 1;
    if (extents.z() > (axis == 0 ? extents.x() : extents.y())) axis = 2;

    auto centroid = [axis](const Triangle& t) {
        float c = (t.v[0].data[axis] + t.v[1].data[axis] + t.v[2].data[axis]) / 3.f;
        return c;
    };
    int mid = triStart + triCount / 2;
    std::nth_element(m_tris.begin() + triStart,
                     m_tris.begin() + mid,
                     m_tris.begin() + triStart + triCount,
                     [&](const Triangle& a, const Triangle& b) {
                         return centroid(a) < centroid(b);
                     });

    int leftIdx  = static_cast<int>(m_bvh.size());
    m_bvh.emplace_back();
    int rightIdx = static_cast<int>(m_bvh.size());
    m_bvh.emplace_back();
    m_bvh[nodeIdx].left  = leftIdx;
    m_bvh[nodeIdx].right = rightIdx;

    buildBVH(leftIdx,  triStart,       mid - triStart);
    buildBVH(rightIdx, mid,            triStart + triCount - mid);
}

} // namespace campello::physics
