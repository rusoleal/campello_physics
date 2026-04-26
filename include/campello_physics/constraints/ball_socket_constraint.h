#pragma once
#include <campello_physics/constraint.h>

namespace campello::physics {

// Locks the relative position of two anchor points (3 translational DOF constrained).
// Both bodies can still rotate freely relative to each other.

class BallSocketConstraint : public Constraint {
public:
    [[nodiscard]] static std::shared_ptr<BallSocketConstraint> create(
        Body a, const vm::Vector3<float>& anchorLocalA,
        Body b, const vm::Vector3<float>& anchorLocalB);

    void prepare(BodyPool& pool, float dt, float baumgarte, float slop) override;
    void warmStart(BodyPool& pool) override;
    void solveVelocity(BodyPool& pool) override;
    void solvePosition(BodyPool& pool, float dt, float alpha) override;

private:
    BallSocketConstraint() = default;

    vm::Vector3<float> m_anchorA{0.f, 0.f, 0.f};
    vm::Vector3<float> m_anchorB{0.f, 0.f, 0.f};
    ConstraintRow      m_rows[3];
};

} // namespace campello::physics
