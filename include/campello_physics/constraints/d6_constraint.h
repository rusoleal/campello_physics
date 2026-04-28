#pragma once
#include <campello_physics/constraint.h>
#include <campello_physics/transform.h>

namespace campello::physics {

enum class D6Motion { Free, Limited, Locked };

// Full 6-DOF constraint with per-axis motion type, limits, and velocity drives.
//
// DOF indices:
//   D6Constraint::LinX/LinY/LinZ  (0-2) — translational, measured in metres
//   D6Constraint::AngX/AngY/AngZ  (3-5) — rotational,    measured in radians
//
// Motion types:
//   Locked  — DOF is fully fixed (no relative motion allowed).
//   Limited — DOF is free within [lower, upper]; clamped at the boundary.
//   Free    — DOF is unconstrained.
//
// A velocity drive can be attached to any DOF independently of its motion type.
// The drive targets a relative velocity along/about that DOF's axis.
//
// Default: all 6 DOF Locked — equivalent to FixedConstraint.

class D6Constraint : public Constraint {
public:
    static constexpr int LinX = 0, LinY = 1, LinZ = 2;
    static constexpr int AngX = 3, AngY = 4, AngZ = 5;

    [[nodiscard]] static std::shared_ptr<D6Constraint> create(
        Body a, const Transform& frameLocalA,
        Body b, const Transform& frameLocalB);

    // Per-DOF motion type (index 0-5 as above).
    void     setMotion(int dof, D6Motion motion) noexcept;
    D6Motion getMotion(int dof)            const noexcept;

    // Limits for a Limited DOF.  Linear in metres, angular in radians.
    void setLimit(int dof, float lower, float upper) noexcept;

    // Velocity drive: drives the relative velocity of DOF dof to targetVelocity.
    // maxImpulse == 0 disables the drive.
    void setDrive (int dof, float targetVelocity, float maxImpulse) noexcept;
    void clearDrive(int dof) noexcept;

    void prepare      (BodyPool& pool, float dt, float baumgarte, float slop) override;
    void warmStart    (BodyPool& pool) override;
    void solveVelocity(BodyPool& pool) override;
    void solvePosition(BodyPool& pool, float dt, float alpha) override;

    [[nodiscard]] int rowCount() const override { return 6; }
    [[nodiscard]] const ConstraintRow* rows() const override { return m_rows; }
    ConstraintRow* rows() override { return m_rows; }

private:
    D6Constraint() = default;

    Transform m_frameA;   // constraint frame in body A local space
    Transform m_frameB;   // constraint frame in body B local space

    D6Motion m_motion[6] = {
        D6Motion::Locked, D6Motion::Locked, D6Motion::Locked,
        D6Motion::Locked, D6Motion::Locked, D6Motion::Locked
    };

    float m_limitLower[6] = {};
    float m_limitUpper[6] = {};

    struct DriveDesc {
        float targetVelocity = 0.f;
        float maxImpulse     = 0.f;  // 0 = disabled
    };
    DriveDesc m_drive[6];

    // Reference relative quaternion (frame A space). Lazy-initialised on first prepare().
    vm::Quaternion<float> m_qRef;
    bool                  m_qRefInit = false;

    // One row per DOF (fixed layout). Inactive rows carry effMass=0, lambda=0.
    ConstraintRow m_rows[6];
};

} // namespace campello::physics
