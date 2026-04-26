#pragma once
#include <campello_physics/constraint.h>

namespace campello::physics {

// Maintains the distance between two anchor points within [minDist, maxDist].
// When minDist == maxDist it acts as a rigid rod (equality constraint).

class DistanceConstraint : public Constraint {
public:
    [[nodiscard]] static std::shared_ptr<DistanceConstraint> create(
        Body a, const vm::Vector3<float>& anchorLocalA,
        Body b, const vm::Vector3<float>& anchorLocalB,
        float minDist, float maxDist);

    void prepare(BodyPool& pool, float dt, float baumgarte, float slop) override;
    void warmStart(BodyPool& pool) override;
    void solveVelocity(BodyPool& pool) override;
    void solvePosition(BodyPool& pool, float dt, float alpha) override;

private:
    DistanceConstraint() = default;

    vm::Vector3<float> m_anchorA{0.f, 0.f, 0.f};
    vm::Vector3<float> m_anchorB{0.f, 0.f, 0.f};
    float              m_minDist = 0.f;
    float              m_maxDist = 0.f;

    ConstraintRow m_row;
    bool          m_active = false;
};

} // namespace campello::physics
