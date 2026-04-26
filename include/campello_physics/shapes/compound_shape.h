#pragma once

#include <campello_physics/shape.h>
#include <memory>
#include <vector>

namespace campello::physics {

struct CompoundChild {
    std::shared_ptr<Shape> shape;
    Transform              localTransform;
};

class CompoundShape final : public Shape {
public:
    void addChild(std::shared_ptr<Shape> shape, const Transform& localTransform) {
        m_children.push_back({std::move(shape), localTransform});
    }

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::Compound; }

    [[nodiscard]] AABB computeAABB(const Transform& worldTransform) const noexcept override {
        AABB result = AABB::empty();
        for (const auto& child : m_children) {
            Transform childWorld = worldTransform.combined(child.localTransform);
            result.expand(child.shape->computeAABB(childWorld));
        }
        return result;
    }

    // Approximate: treats sub-shape inertias as aligned with compound axes.
    // Uses parallel axis theorem from compound local origin.
    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float mass) const noexcept override {
        if (m_children.empty())
            return vm::Vector3<float>(0.f, 0.f, 0.f);

        float totalVol = static_cast<float>(m_children.size()); // uniform mass split
        vm::Vector3<float> total(0.f, 0.f, 0.f);

        for (const auto& child : m_children) {
            float childMass = mass / totalVol;
            auto  diag      = child.shape->computeLocalInertiaDiagonal(childMass);
            auto& d         = child.localTransform.position;
            float dx = d.x(), dy = d.y(), dz = d.z();
            // Parallel axis theorem (diagonal components only)
            total = vm::Vector3<float>(
                total.x() + diag.x() + childMass * (dy * dy + dz * dz),
                total.y() + diag.y() + childMass * (dx * dx + dz * dz),
                total.z() + diag.z() + childMass * (dx * dx + dy * dy)
            );
        }
        return total;
    }

    [[nodiscard]] const std::vector<CompoundChild>& children() const noexcept { return m_children; }

    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& dir) const noexcept override {
        float bestDot = -1e30f;
        vm::Vector3<float> best(0.f, 0.f, 0.f);
        for (const auto& child : m_children) {
            auto localDir   = child.localTransform.inverseTransformVector(dir);
            auto localSupp  = child.shape->support(localDir);
            auto worldSupp  = child.localTransform.transformPoint(localSupp);
            float d = worldSupp.x()*dir.x() + worldSupp.y()*dir.y() + worldSupp.z()*dir.z();
            if (d > bestDot) { bestDot = d; best = worldSupp; }
        }
        return best;
    }

private:
    std::vector<CompoundChild> m_children;
};

} // namespace campello::physics
