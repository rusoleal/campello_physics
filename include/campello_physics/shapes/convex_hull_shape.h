#pragma once

#include <campello_physics/shape.h>
#include <vector>

namespace campello::physics {

// Convex hull built from a point cloud.
// Stores the raw points; a support-function query (for GJK, Phase 4) iterates them.
class ConvexHullShape final : public Shape {
public:
    explicit ConvexHullShape(std::vector<vm::Vector3<float>> points);

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::ConvexHull; }

    [[nodiscard]] AABB computeAABB(const Transform& t) const noexcept override;

    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float mass) const noexcept override;

    [[nodiscard]] const std::vector<vm::Vector3<float>>& points() const noexcept { return m_points; }

    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& dir) const noexcept override;

private:
    std::vector<vm::Vector3<float>> m_points;
    AABB m_localAABB;
};

} // namespace campello::physics
