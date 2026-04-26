#pragma once

#include <campello_physics/shape.h>
#include <vector>
#include <span>

namespace campello::physics {

// Terrain height field: a rows×cols grid of heights.
// Only valid as a Static body shape — inertia is zero.
class HeightFieldShape final : public Shape {
public:
    HeightFieldShape(int rows, int cols,
                     float cellSizeX, float cellSizeZ,
                     float heightScale,
                     std::span<const float> heights);

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::HeightField; }

    [[nodiscard]] AABB computeAABB(const Transform& t) const noexcept override;

    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float /*mass*/) const noexcept override {
        return vm::Vector3<float>(0.f, 0.f, 0.f);
    }

    // GJK is not applicable to non-convex height fields.
    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& /*dir*/) const noexcept override {
        return vm::Vector3<float>(0.f, 0.f, 0.f);
    }

    [[nodiscard]] int   rows()        const noexcept { return m_rows; }
    [[nodiscard]] int   cols()        const noexcept { return m_cols; }
    [[nodiscard]] float cellSizeX()   const noexcept { return m_cellSizeX; }
    [[nodiscard]] float cellSizeZ()   const noexcept { return m_cellSizeZ; }
    [[nodiscard]] float heightScale() const noexcept { return m_heightScale; }
    [[nodiscard]] float heightAt(int row, int col) const noexcept {
        return m_heights[row * m_cols + col] * m_heightScale;
    }
    [[nodiscard]] const AABB& localAABB() const noexcept { return m_localAABB; }

private:
    int                m_rows;
    int                m_cols;
    float              m_cellSizeX;
    float              m_cellSizeZ;
    float              m_heightScale;
    std::vector<float> m_heights;
    AABB               m_localAABB;
};

} // namespace campello::physics
