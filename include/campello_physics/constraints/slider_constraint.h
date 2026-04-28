#pragma once
#include <campello_physics/constraint.h>

namespace campello::physics {

// One translational DOF: bodies slide freely along the slide axis.
// 2 linear + 3 angular rows constrained; optional translation limits and motor.

class SliderConstraint : public Constraint {
public:
    [[nodiscard]] static std::shared_ptr<SliderConstraint> create(
        Body a, const vm::Vector3<float>& anchorLocalA, const vm::Vector3<float>& axisLocalA,
        Body b, const vm::Vector3<float>& anchorLocalB, const vm::Vector3<float>& axisLocalB);

    // Translation limits (metres along slide axis). Only active when min < max.
    void setLimits(float minDist, float maxDist);
    void clearLimits();

    // Motor: drives linear velocity along the slide axis.
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
    SliderConstraint() = default;

    vm::Vector3<float>    m_anchorA{0.f, 0.f, 0.f};
    vm::Vector3<float>    m_anchorB{0.f, 0.f, 0.f};
    vm::Vector3<float>    m_axisLocalA{0.f, 1.f, 0.f};
    vm::Vector3<float>    m_axisLocalB{0.f, 1.f, 0.f};
    vm::Quaternion<float> m_qRefInA;   // q_A0_conj * q_B0

    bool  m_limitsActive = false;
    float m_limitMin     = 0.f;
    float m_limitMax     = 0.f;

    bool  m_motorActive    = false;
    float m_motorSpeed     = 0.f;
    float m_motorMaxImpulse = 0.f;

    // Rows: [0-1] perp linear, [2-4] angular, [5] limit or motor
    ConstraintRow m_rows[6];
    int           m_rowCount = 5;
};

} // namespace campello::physics
