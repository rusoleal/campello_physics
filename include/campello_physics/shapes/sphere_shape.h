#pragma once

#include <campello_physics/shape.h>

namespace campello::physics {

class SphereShape final : public Shape {
public:
    explicit SphereShape(float radius) noexcept : m_radius(radius) {}

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::Sphere; }

    [[nodiscard]] float radius() const noexcept { return m_radius; }

    [[nodiscard]] AABB computeAABB(const Transform& t) const noexcept override {
        return AABB::fromCenterHalfExtents(t.position, vm::Vector3<float>(m_radius, m_radius, m_radius));
    }

    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float mass) const noexcept override {
        const float i = (2.f / 5.f) * mass * m_radius * m_radius;
        return vm::Vector3<float>(i, i, i);
    }

    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& dir) const noexcept override {
        float lenSq = dir.x()*dir.x() + dir.y()*dir.y() + dir.z()*dir.z();
        if (lenSq < 1e-20f) return vm::Vector3<float>(m_radius, 0.f, 0.f);
        float inv = m_radius / std::sqrt(lenSq);
        return dir * inv;
    }

private:
    float m_radius;
};

} // namespace campello::physics
