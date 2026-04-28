#pragma once
#include <campello_physics/constraint.h>

namespace campello::physics {

// One rotational DOF: bodies rotate freely about the hinge axis.
// 3 linear + 2 angular rows constrained; optional rotation limits and motor.

class HingeConstraint : public Constraint {
public:
    [[nodiscard]] static std::shared_ptr<HingeConstraint> create(
        Body a, const vm::Vector3<float>& anchorLocalA, const vm::Vector3<float>& axisLocalA,
        Body b, const vm::Vector3<float>& anchorLocalB, const vm::Vector3<float>& axisLocalB);

    // Rotation limits (radians). Only active when min < max.
    void setLimits(float minAngle, float maxAngle);
    void clearLimits();

    // Motor: drives angular velocity about the hinge axis.
    void setMotor(float targetSpeed, float maxImpulse);
    void clearMotor();

    void prepare(BodyPool& pool, float dt, float baumgarte, float slop) override;
    void warmStart(BodyPool& pool) override;
    void solveVelocity(BodyPool& pool) override;
    void solvePosition(BodyPool& pool, float dt, float alpha) override;

    [[nodiscard]] int rowCount() const override { return m_rowCount; }
    [[nodiscard]] const ConstraintRow* rows() const override { return m_rows; }
    ConstraintRow* rows() override { return m_rows; }

private:
    HingeConstraint() = default;

    vm::Vector3<float> m_anchorA{0.f, 0.f, 0.f};
    vm::Vector3<float> m_anchorB{0.f, 0.f, 0.f};
    vm::Vector3<float> m_axisLocalA{0.f, 1.f, 0.f};
    vm::Vector3<float> m_axisLocalB{0.f, 1.f, 0.f};

    // Reference perp vector (in body A local) for angle measurement
    vm::Vector3<float> m_refPerpLocalA{1.f, 0.f, 0.f};
    // Matching perp vector stored in body B local frame at creation
    vm::Vector3<float> m_refPerpLocalB{1.f, 0.f, 0.f};

    bool  m_limitsActive = false;
    float m_limitMin     = 0.f;
    float m_limitMax     = 0.f;

    bool  m_motorActive   = false;
    float m_motorSpeed    = 0.f;
    float m_motorMaxImpulse = 0.f;

    // Rows: [0-2] ball-socket linear, [3-4] angular perp, [5] limit or motor
    ConstraintRow m_rows[6];
    int           m_rowCount = 5;
};

} // namespace campello::physics
