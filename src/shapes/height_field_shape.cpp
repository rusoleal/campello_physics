#include <campello_physics/shapes/height_field_shape.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace campello::physics {

HeightFieldShape::HeightFieldShape(int rows, int cols,
                                   float cellSizeX, float cellSizeZ,
                                   float heightScale,
                                   std::span<const float> heights)
    : m_rows(rows)
    , m_cols(cols)
    , m_cellSizeX(cellSizeX)
    , m_cellSizeZ(cellSizeZ)
    , m_heightScale(heightScale)
    , m_heights(heights.begin(), heights.end())
{
    assert(rows > 1 && cols > 1);
    assert(static_cast<int>(heights.size()) == rows * cols);

    float minH = std::numeric_limits<float>::infinity();
    float maxH = -std::numeric_limits<float>::infinity();
    for (float h : m_heights) {
        minH = std::min(minH, h);
        maxH = std::max(maxH, h);
    }

    m_localAABB = AABB::fromMinMax(
        vm::Vector3<float>(0.f,                   minH * m_heightScale, 0.f),
        vm::Vector3<float>((cols - 1) * cellSizeX, maxH * m_heightScale, (rows - 1) * cellSizeZ)
    );
}

AABB HeightFieldShape::computeAABB(const Transform& t) const noexcept {
    auto he     = m_localAABB.halfExtents();
    auto center = t.transformPoint(m_localAABB.center());
    auto ax     = t.rotation.rotated(vm::Vector3<float>(he.x(), 0.f, 0.f));
    auto ay     = t.rotation.rotated(vm::Vector3<float>(0.f, he.y(), 0.f));
    auto az     = t.rotation.rotated(vm::Vector3<float>(0.f, 0.f, he.z()));
    vm::Vector3<float> worldHe(
        std::fabs(ax.x()) + std::fabs(ay.x()) + std::fabs(az.x()),
        std::fabs(ax.y()) + std::fabs(ay.y()) + std::fabs(az.y()),
        std::fabs(ax.z()) + std::fabs(ay.z()) + std::fabs(az.z())
    );
    return AABB::fromCenterHalfExtents(center, worldHe);
}

} // namespace campello::physics
