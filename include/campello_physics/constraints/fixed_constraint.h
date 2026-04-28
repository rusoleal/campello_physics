#pragma once
#include <campello_physics/constraint.h>

namespace campello::physics {

// Locks all 6 DOF — the two bodies move as a rigid unit.
// The relative position and orientation at creation time are preserved.

class FixedConstraint : public Constraint {
public:
    [[nodiscard]] static std::shared_ptr<FixedConstraint> create(Body a, Body b);

    void prepare(BodyPool& pool, float dt, float baumgarte, float slop) override;
    void warmStart(BodyPool& pool) override;
    void solveVelocity(BodyPool& pool) override;
    void solvePosition(BodyPool& pool, float dt, float alpha) override;

    [[nodiscard]] int rowCount() const override { return 6; }
    [[nodiscard]] const ConstraintRow* rows() const override { return m_rows; }
    ConstraintRow* rows() override { return m_rows; }

private:
    FixedConstraint() = default;

    vm::Vector3<float>    m_anchorA{0.f, 0.f, 0.f};
    vm::Vector3<float>    m_anchorB{0.f, 0.f, 0.f};
    vm::Quaternion<float> m_qRefInA;   // q_A0_conj * q_B0

    ConstraintRow m_rows[6];   // [0-2] linear, [3-5] angular
};

} // namespace campello::physics
