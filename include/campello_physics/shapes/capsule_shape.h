#pragma once

#include <campello_physics/shape.h>
#include <cmath>
#include <numbers>

namespace campello::physics {

// Capsule aligned along the Y axis: cylinder of `halfHeight` capped by two hemispheres of `radius`.
class CapsuleShape final : public Shape {
public:
    CapsuleShape(float radius, float halfHeight) noexcept
        : m_radius(radius), m_halfHeight(halfHeight) {}

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::Capsule; }

    [[nodiscard]] float radius()     const noexcept { return m_radius; }
    [[nodiscard]] float halfHeight() const noexcept { return m_halfHeight; }

    [[nodiscard]] AABB computeAABB(const Transform& t) const noexcept override {
        // Transform the two sphere-cap centres, then expand each by radius.
        auto top    = t.transformPoint(vm::Vector3<float>(0.f,  m_halfHeight, 0.f));
        auto bottom = t.transformPoint(vm::Vector3<float>(0.f, -m_halfHeight, 0.f));
        AABB b = AABB::empty();
        b.expand(top);
        b.expand(bottom);
        vm::Vector3<float> r(m_radius, m_radius, m_radius);
        return AABB::fromMinMax(b.min - r, b.max + r);
    }

    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& dir) const noexcept override {
        // Capsule = cylinder along Y capped with hemispheres.
        // The support sphere centre is at the top or bottom depending on dir.y.
        float cy = (dir.y() >= 0.f) ? m_halfHeight : -m_halfHeight;
        float lenSq = dir.x()*dir.x() + dir.y()*dir.y() + dir.z()*dir.z();
        if (lenSq < 1e-20f) return vm::Vector3<float>(m_radius, cy, 0.f);
        float inv = m_radius / std::sqrt(lenSq);
        return vm::Vector3<float>(dir.x()*inv, cy + dir.y()*inv, dir.z()*inv);
    }

    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float mass) const noexcept override {
        const float r  = m_radius;
        const float h  = 2.f * m_halfHeight;
        const float pi = std::numbers::pi_v<float>;

        const float vCyl  = pi * r * r * h;
        const float vHemi = (2.f / 3.f) * pi * r * r * r;
        const float vTot  = vCyl + 2.f * vHemi;

        const float mCyl  = mass * vCyl  / vTot;
        const float mHemi = mass * vHemi / vTot;  // each hemisphere

        // Y axis (symmetry axis)
        const float iy = mCyl * r * r / 2.f + 2.f * mHemi * (2.f * r * r / 5.f);

        // X/Z axes — hemisphere COM is at (3/8)*r from its flat face
        const float d   = m_halfHeight + 3.f * r / 8.f;
        const float ix  = mCyl * (r * r / 4.f + h * h / 12.f)
                        + 2.f * (mHemi * 2.f * r * r / 5.f + mHemi * d * d);

        return vm::Vector3<float>(ix, iy, ix);
    }

private:
    float m_radius;
    float m_halfHeight;
};

} // namespace campello::physics
