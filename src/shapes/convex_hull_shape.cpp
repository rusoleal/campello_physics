#include <campello_physics/shapes/convex_hull_shape.h>
#include <cassert>
#include <cmath>

namespace campello::physics {

ConvexHullShape::ConvexHullShape(std::vector<vm::Vector3<float>> points)
    : m_points(std::move(points))
{
    assert(!m_points.empty());
    m_localAABB = AABB::empty();
    for (const auto& p : m_points)
        m_localAABB.expand(p);
}

AABB ConvexHullShape::computeAABB(const Transform& t) const noexcept {
    AABB result = AABB::empty();
    for (const auto& p : m_points)
        result.expand(t.transformPoint(p));
    return result;
}

vm::Vector3<float> ConvexHullShape::computeLocalInertiaDiagonal(float mass) const noexcept {
    // Approximate using the AABB half-extents (conservative overestimate).
    auto he = m_localAABB.halfExtents();
    const float hx = he.x(), hy = he.y(), hz = he.z();
    return vm::Vector3<float>(
        (mass / 3.f) * (hy * hy + hz * hz),
        (mass / 3.f) * (hx * hx + hz * hz),
        (mass / 3.f) * (hx * hx + hy * hy)
    );
}

vm::Vector3<float> ConvexHullShape::support(const vm::Vector3<float>& dir) const noexcept {
    float best = -1e30f;
    vm::Vector3<float> result(0.f, 0.f, 0.f);
    for (const auto& p : m_points) {
        float d = p.x()*dir.x() + p.y()*dir.y() + p.z()*dir.z();
        if (d > best) { best = d; result = p; }
    }
    return result;
}

} // namespace campello::physics
