#pragma once

#include <campello_physics/shape.h>
#include <cmath>

namespace campello::physics {

// Cylinder aligned along the Y axis.
class CylinderShape final : public Shape {
public:
    CylinderShape(float radius, float halfHeight) noexcept
        : m_radius(radius), m_halfHeight(halfHeight) {}

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::Cylinder; }

    [[nodiscard]] float radius()     const noexcept { return m_radius; }
    [[nodiscard]] float halfHeight() const noexcept { return m_halfHeight; }

    [[nodiscard]] AABB computeAABB(const Transform& t) const noexcept override {
        // Treat as OBB with half-extents (r, h, r) — same as BoxShape formula.
        const float r = m_radius;
        const float h = m_halfHeight;
        auto ax = t.rotation.rotated(vm::Vector3<float>(r, 0.f, 0.f));
        auto ay = t.rotation.rotated(vm::Vector3<float>(0.f, h, 0.f));
        auto az = t.rotation.rotated(vm::Vector3<float>(0.f, 0.f, r));
        vm::Vector3<float> he(
            std::fabs(ax.x()) + std::fabs(ay.x()) + std::fabs(az.x()),
            std::fabs(ax.y()) + std::fabs(ay.y()) + std::fabs(az.y()),
            std::fabs(ax.z()) + std::fabs(ay.z()) + std::fabs(az.z())
        );
        return AABB::fromCenterHalfExtents(t.position, he);
    }

    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float mass) const noexcept override {
        const float r = m_radius;
        const float h = m_halfHeight;  // half-height
        // Ixx = Izz = m * (r²/4 + h²/3)  [h here is half-height]
        // Iyy = m * r²/2
        const float ixz = mass * (r * r / 4.f + h * h / 3.f);
        const float iy  = mass * r * r / 2.f;
        return vm::Vector3<float>(ixz, iy, ixz);
    }

    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& dir) const noexcept override {
        // Cylinder along Y axis.  XZ disc support + ±halfHeight on Y.
        float xzLenSq = dir.x()*dir.x() + dir.z()*dir.z();
        float y = (dir.y() >= 0.f) ? m_halfHeight : -m_halfHeight;
        if (xzLenSq < 1e-20f) return vm::Vector3<float>(0.f, y, 0.f);
        float inv = m_radius / std::sqrt(xzLenSq);
        return vm::Vector3<float>(dir.x()*inv, y, dir.z()*inv);
    }

private:
    float m_radius;
    float m_halfHeight;
};

} // namespace campello::physics
