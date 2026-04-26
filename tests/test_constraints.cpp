#include <gtest/gtest.h>
#include <campello_physics/constraint.h>
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <campello_physics/constraints/fixed_constraint.h>
#include <campello_physics/constraints/hinge_constraint.h>
#include <campello_physics/constraints/slider_constraint.h>
#include <campello_physics/constraints/distance_constraint.h>
#include <campello_physics/constraints/d6_constraint.h>
#include <campello_physics/integrator.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <cmath>

using namespace campello::physics;

namespace {

using V3 = vm::Vector3<float>;
using Q  = vm::Quaternion<float>;

inline float dot3(const V3& a, const V3& b) {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
}
inline float len3(const V3& v) {
    return std::sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z());
}

// Creates a dynamic body with a sphere shape (so inertia is non-trivial).
Body makeDynamic(BodyPool& pool, V3 pos, float mass = 1.f) {
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.mass  = mass;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.transform.position = pos;
    d.linearDamping = d.angularDamping = 0.f;
    return pool.createBody(d);
}

IntegratorSettings noGrav() {
    IntegratorSettings s;
    s.gravity = V3(0.f, 0.f, 0.f);
    s.sleepFramesRequired = 1000000;
    return s;
}

// Simulate N steps of constraints + integration
void simulate(BodyPool& pool, ConstraintSolver& solver, int steps, float dt = 1.f/60.f) {
    IntegratorSettings s = noGrav();
    for (int i = 0; i < steps; ++i) {
        solver.solve(pool, dt);
        integrate(pool, s, dt);
    }
}

} // namespace

// ── BallSocketConstraint ──────────────────────────────────────────────────────

TEST(Constraints, BallSocketKeepsAnchorsTogether) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(2.f, 0.f, 0.f));

    // Connect at world origin: A's right edge, B's left edge
    auto c = BallSocketConstraint::create(
        a, V3(1.f, 0.f, 0.f),   // local anchor on A
        b, V3(-1.f, 0.f, 0.f)); // local anchor on B

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Give them opposite velocities to try to pull apart
    a.setLinearVelocity(V3(-5.f, 0.f, 0.f));
    b.setLinearVelocity(V3( 5.f, 0.f, 0.f));

    simulate(pool, solver, 120);

    // Anchors in world space should converge
    const auto wA = a.transform().position + V3(1.f, 0.f, 0.f);
    const auto wB = b.transform().position + V3(-1.f, 0.f, 0.f);
    const auto diff = wA - wB;
    float dist = len3(diff);
    EXPECT_LT(dist, 0.05f) << "Anchors separated by " << dist << " m";
}

TEST(Constraints, BallSocketRelativeVelocityConverges) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(1.f, 0.f, 0.f));

    auto c = BallSocketConstraint::create(a, V3(0.f,0.f,0.f), b, V3(0.f,0.f,0.f));
    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    a.setLinearVelocity(V3(3.f, 0.f, 0.f));
    simulate(pool, solver, 60);

    // Relative velocity at the anchors should be near zero
    const auto rv = a.linearVelocity() - b.linearVelocity();
    EXPECT_LT(len3(rv), 0.1f);
}

// ── FixedConstraint ───────────────────────────────────────────────────────────

TEST(Constraints, FixedConstraintMaintainsRelativePosition) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(1.f, 0.f, 0.f));

    auto c = FixedConstraint::create(a, b);
    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Push both in the same direction — they should stay together
    a.applyLinearImpulse(V3(5.f, 0.f, 0.f));

    simulate(pool, solver, 60);

    // Distance should remain ≈ 1 m
    const auto diff = a.transform().position - b.transform().position;
    float dist = len3(diff);
    EXPECT_NEAR(dist, 1.f, 0.05f);
}

TEST(Constraints, FixedConstraintMaintainsRelativeOrientation) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    auto c = FixedConstraint::create(a, b);
    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Apply torque to A — B should follow
    a.applyAngularImpulse(V3(0.f, 1.f, 0.f));

    simulate(pool, solver, 60);

    // Relative quaternion should be close to identity
    const auto qA = a.transform().rotation;
    const auto qB = b.transform().rotation;
    auto qRel = qA.conjugated() * qB;
    if (qRel.data[3] < 0.f) {
        qRel.data[0]=-qRel.data[0]; qRel.data[1]=-qRel.data[1];
        qRel.data[2]=-qRel.data[2]; qRel.data[3]=-qRel.data[3];
    }
    float angErr = std::sqrt(qRel.data[0]*qRel.data[0] +
                             qRel.data[1]*qRel.data[1] +
                             qRel.data[2]*qRel.data[2]) * 2.f;
    EXPECT_LT(angErr, 0.15f) << "Angular error = " << angErr << " rad";
}

// ── HingeConstraint ───────────────────────────────────────────────────────────

TEST(Constraints, HingeAllowsRotationAboutAxis) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(1.f, 0.f, 0.f));

    // Hinge on Y axis
    auto c = HingeConstraint::create(
        a, V3(0.f,0.f,0.f), V3(0.f,1.f,0.f),
        b, V3(-1.f,0.f,0.f), V3(0.f,1.f,0.f));

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Apply angular impulse about hinge axis — should be allowed
    a.applyAngularImpulse(V3(0.f, 2.f, 0.f));
    simulate(pool, solver, 60);

    // Angular velocity about Y should still be substantial (not killed)
    float wy = a.angularVelocity().y();
    EXPECT_GT(std::abs(wy), 0.1f) << "Hinge killed rotation about its own axis";
}

TEST(Constraints, HingeConstrainsPerpendicularRotation) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(1.f, 0.f, 0.f));

    auto c = HingeConstraint::create(
        a, V3(0.f,0.f,0.f), V3(0.f,1.f,0.f),
        b, V3(-1.f,0.f,0.f), V3(0.f,1.f,0.f));

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Apply angular impulse perpendicular to hinge — should be strongly damped
    a.applyAngularImpulse(V3(5.f, 0.f, 0.f));
    simulate(pool, solver, 120);

    // After many steps, relative perpendicular angular velocity should be small
    const auto wRel = a.angularVelocity() - b.angularVelocity();
    float wx = wRel.x(), wz = wRel.z();
    EXPECT_LT(std::abs(wx) + std::abs(wz), 0.3f);
}

TEST(Constraints, HingeLimitPreventsExceedingMaxAngle) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(1.f, 0.f, 0.f));

    auto c = HingeConstraint::create(
        a, V3(0.f,0.f,0.f), V3(0.f,1.f,0.f),
        b, V3(-1.f,0.f,0.f), V3(0.f,1.f,0.f));
    c->setLimits(-0.1f, 0.1f);

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Spin both bodies — limits should keep them in range
    a.applyAngularImpulse(V3(0.f, 10.f, 0.f));
    simulate(pool, solver, 120);

    // After convergence, relative angular velocity about Y should be low
    float wyRel = a.angularVelocity().y() - b.angularVelocity().y();
    EXPECT_LT(std::abs(wyRel), 2.f) << "Limit not constraining: wyRel = " << wyRel;
}

// ── SliderConstraint ──────────────────────────────────────────────────────────

TEST(Constraints, SliderAllowsTranslationAlongAxis) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    // Slide along X
    auto c = SliderConstraint::create(
        a, V3(0.f,0.f,0.f), V3(1.f,0.f,0.f),
        b, V3(0.f,0.f,0.f), V3(1.f,0.f,0.f));

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Push B along slide axis — should be allowed
    b.applyLinearImpulse(V3(3.f, 0.f, 0.f));
    simulate(pool, solver, 60);

    float vx = b.linearVelocity().x();
    EXPECT_GT(vx, 0.1f) << "Slider killed motion along its own axis";
}

TEST(Constraints, SliderConstrainsPerpendicularTranslation) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    auto c = SliderConstraint::create(
        a, V3(0.f,0.f,0.f), V3(1.f,0.f,0.f),
        b, V3(0.f,0.f,0.f), V3(1.f,0.f,0.f));

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Push B perpendicular to slide — should be constrained
    b.applyLinearImpulse(V3(0.f, 5.f, 0.f));
    simulate(pool, solver, 60);

    // Momentum is conserved so both bodies end up at COM velocity ≈ 2.5 ĵ.
    // What matters is that the *relative* perpendicular velocity is near zero.
    const auto relV = b.linearVelocity() - a.linearVelocity();
    EXPECT_LT(std::abs(relV.y()), 0.1f) << "Slider relative vy = " << relV.y();
}

// ── DistanceConstraint ────────────────────────────────────────────────────────

TEST(Constraints, DistanceEqualityMaintainsSeparation) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(3.f, 0.f, 0.f));

    // Rod of fixed length 3
    auto c = DistanceConstraint::create(
        a, V3(0.f,0.f,0.f), b, V3(0.f,0.f,0.f), 3.f, 3.f);

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Yank both toward each other
    a.applyLinearImpulse(V3( 5.f, 0.f, 0.f));
    b.applyLinearImpulse(V3(-5.f, 0.f, 0.f));

    simulate(pool, solver, 120);

    const auto diff = a.transform().position - b.transform().position;
    float dist = len3(diff);
    EXPECT_NEAR(dist, 3.f, 0.15f) << "Distance constraint off by " << std::abs(dist - 3.f);
}

TEST(Constraints, DistanceMinLimitPreventsCompression) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(2.f, 0.f, 0.f));

    // Min 2, max 10 — bodies start at 2, push them together
    auto c = DistanceConstraint::create(
        a, V3(0.f,0.f,0.f), b, V3(0.f,0.f,0.f), 2.f, 10.f);

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    a.applyLinearImpulse(V3( 4.f, 0.f, 0.f));
    b.applyLinearImpulse(V3(-4.f, 0.f, 0.f));

    simulate(pool, solver, 120);

    const auto diff = a.transform().position - b.transform().position;
    float dist = len3(diff);
    EXPECT_GE(dist, 1.9f) << "Min limit violated: dist = " << dist;
}

TEST(Constraints, DistanceMaxLimitPreventsExtension) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(1.f, 0.f, 0.f));

    // Max 2 — bodies start at 1, pull them apart
    auto c = DistanceConstraint::create(
        a, V3(0.f,0.f,0.f), b, V3(0.f,0.f,0.f), 0.f, 2.f);

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    a.applyLinearImpulse(V3(-5.f, 0.f, 0.f));
    b.applyLinearImpulse(V3( 5.f, 0.f, 0.f));

    simulate(pool, solver, 120);

    const auto diff = a.transform().position - b.transform().position;
    float dist = len3(diff);
    EXPECT_LE(dist, 2.1f) << "Max limit violated: dist = " << dist;
}

// ── D6Constraint ─────────────────────────────────────────────────────────────

TEST(Constraints, D6AllLocked_ActsLikeFixed) {
    BodyPool pool;
    // Bodies at the same origin: identity frames weld them at their origins.
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    auto c = D6Constraint::create(a, Transform::identity(), b, Transform::identity());
    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    a.applyLinearImpulse(V3(5.f, 3.f, 1.f));
    simulate(pool, solver, 60);

    // Both bodies should stay co-located (welded).
    float dist = len3(a.transform().position - b.transform().position);
    EXPECT_LT(dist, 0.05f) << "D6 locked: bodies separated by " << dist << " m";
}

TEST(Constraints, D6LinearFree_AllowsTranslation) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    auto c = D6Constraint::create(a, Transform::identity(), b, Transform::identity());
    c->setMotion(D6Constraint::LinX, D6Motion::Free);  // X translation free

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    b.applyLinearImpulse(V3(3.f, 0.f, 0.f));
    simulate(pool, solver, 60);

    // B should be moving along X
    EXPECT_GT(b.linearVelocity().x(), 0.1f) << "D6: free LinX killed X velocity";

    // Perpendicular relative velocities should be constrained
    const auto relV = b.linearVelocity() - a.linearVelocity();
    EXPECT_LT(std::abs(relV.y()), 0.05f) << "D6: LinY not constrained";
    EXPECT_LT(std::abs(relV.z()), 0.05f) << "D6: LinZ not constrained";
}

TEST(Constraints, D6AngularFree_AllowsRotation) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    auto c = D6Constraint::create(a, Transform::identity(), b, Transform::identity());
    c->setMotion(D6Constraint::AngX, D6Motion::Free);  // rotation about X free

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    b.applyAngularImpulse(V3(2.f, 0.f, 0.f));
    simulate(pool, solver, 60);

    // B should still have angular velocity about X
    EXPECT_GT(b.angularVelocity().x(), 0.1f) << "D6: free AngX killed X angular velocity";

    // Relative angular velocity about Y and Z should be near zero
    const auto relW = b.angularVelocity() - a.angularVelocity();
    EXPECT_LT(std::abs(relW.y()), 0.1f) << "D6: AngY not constrained";
    EXPECT_LT(std::abs(relW.z()), 0.1f) << "D6: AngZ not constrained";
}

TEST(Constraints, D6LinearLimited_ClampsBoundary) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.5f, 0.f, 0.f));  // start inside [-1, 1]

    auto c = D6Constraint::create(a, Transform::identity(), b, Transform::identity());
    c->setMotion(D6Constraint::LinX, D6Motion::Limited);
    c->setLimit(D6Constraint::LinX, -1.f, 1.f);
    // Y and Z translation free for this test; angular all locked
    c->setMotion(D6Constraint::LinY, D6Motion::Free);
    c->setMotion(D6Constraint::LinZ, D6Motion::Free);

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    // Push B beyond the upper limit (small enough for Baumgarte to contain)
    b.applyLinearImpulse(V3(2.f, 0.f, 0.f));
    simulate(pool, solver, 120);

    // Relative X displacement should remain near the limit
    const auto relPos = b.transform().position - a.transform().position;
    EXPECT_LE(relPos.x(), 1.5f) << "D6 LinX limit violated: relX = " << relPos.x();
}

TEST(Constraints, D6LinearDrive_ReachesTargetVelocity) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    auto c = D6Constraint::create(a, Transform::identity(), b, Transform::identity());
    // Free all linear so the drive is the only force on LinX
    c->setMotion(D6Constraint::LinX, D6Motion::Free);
    c->setMotion(D6Constraint::LinY, D6Motion::Free);
    c->setMotion(D6Constraint::LinZ, D6Motion::Free);
    // Angular all locked (default)
    const float target = 2.f;
    c->setDrive(D6Constraint::LinX, target, 10000.f);

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    simulate(pool, solver, 120);

    // Relative velocity along X should converge to target
    const float relVx = b.linearVelocity().x() - a.linearVelocity().x();
    EXPECT_NEAR(relVx, target, 0.2f) << "D6 drive: relVx = " << relVx;
}

TEST(Constraints, D6AngularDrive_ReachesTargetAngularVelocity) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f, 0.f, 0.f));
    auto b = makeDynamic(pool, V3(0.f, 0.f, 0.f));

    auto c = D6Constraint::create(a, Transform::identity(), b, Transform::identity());
    c->setMotion(D6Constraint::AngX, D6Motion::Free);
    const float target = 1.f;  // 1 rad/s about X
    c->setDrive(D6Constraint::AngX, target, 10000.f);

    ConstraintSolver solver;
    solver.iterations = 20;
    solver.add(c);

    simulate(pool, solver, 120);

    // Relative angular velocity about X should converge to target
    const float relWx = b.angularVelocity().x() - a.angularVelocity().x();
    EXPECT_NEAR(relWx, target, 0.2f) << "D6 angular drive: relWx = " << relWx;
}

// ── Solver infrastructure ─────────────────────────────────────────────────────

TEST(Constraints, SolverAddRemoveClear) {
    BodyPool pool;
    auto a = makeDynamic(pool, V3(0.f,0.f,0.f));
    auto b = makeDynamic(pool, V3(1.f,0.f,0.f));

    ConstraintSolver solver;
    auto c = BallSocketConstraint::create(a, V3(0.f,0.f,0.f), b, V3(0.f,0.f,0.f));
    solver.add(c);
    EXPECT_EQ(solver.constraints().size(), 1u);

    solver.remove(c);
    EXPECT_EQ(solver.constraints().size(), 0u);

    solver.add(c);
    solver.clear();
    EXPECT_EQ(solver.constraints().size(), 0u);
}
