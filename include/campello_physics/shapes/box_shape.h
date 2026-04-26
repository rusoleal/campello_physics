#pragma once

#include <campello_physics/shape.h>
#include <cmath>

namespace campello::physics {

class BoxShape final : public Shape {
public:
    explicit BoxShape(const vm::Vector3<float>& halfExtents) noexcept
        : m_halfExtents(halfExtents) {}

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::Box; }

    [[nodiscard]] const vm::Vector3<float>& halfExtents() const noexcept { return m_halfExtents; }

    [[nodiscard]] AABB computeAABB(const Transform& t) const noexcept override {
        // OBB → AABB: project rotated half-extents onto world axes.
        const float hx = m_halfExtents.x();
        const float hy = m_halfExtents.y();
        const float hz = m_halfExtents.z();
        auto ax = t.rotation.rotated(vm::Vector3<float>(hx, 0.f, 0.f));
        auto ay = t.rotation.rotated(vm::Vector3<float>(0.f, hy, 0.f));
        auto az = t.rotation.rotated(vm::Vector3<float>(0.f, 0.f, hz));
        vm::Vector3<float> he(
            std::fabs(ax.x()) + std::fabs(ay.x()) + std::fabs(az.x()),
            std::fabs(ax.y()) + std::fabs(ay.y()) + std::fabs(az.y()),
            std::fabs(ax.z()) + std::fabs(ay.z()) + std::fabs(az.z())
        );
        return AABB::fromCenterHalfExtents(t.position, he);
    }

    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float mass) const noexcept override {
        const float hx = m_halfExtents.x();
        const float hy = m_halfExtents.y();
        const float hz = m_halfExtents.z();
        // I = (mass/3) * (b² + c²) for each axis pair
        return vm::Vector3<float>(
            (mass / 3.f) * (hy * hy + hz * hz),
            (mass / 3.f) * (hx * hx + hz * hz),
            (mass / 3.f) * (hx * hx + hy * hy)
        );
    }

    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& dir) const noexcept override {
        return vm::Vector3<float>(
            dir.x() >= 0.f ? m_halfExtents.x() : -m_halfExtents.x(),
            dir.y() >= 0.f ? m_halfExtents.y() : -m_halfExtents.y(),
            dir.z() >= 0.f ? m_halfExtents.z() : -m_halfExtents.z()
        );
    }

private:
    vm::Vector3<float> m_halfExtents;
};

} // namespace campello::physics
