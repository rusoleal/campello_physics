#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <campello_physics/constraints/hinge_constraint.h>
#include <campello_physics/constraints/fixed_constraint.h>
#include <cmath>
#include <random>
#include <string>
#include <vector>

using namespace campello::physics;
namespace {

using V3 = vm::Vector3<float>;
using Q  = vm::Quaternion<float>;

constexpr float kDt = 1.f / 60.f;

// ── Sanity helpers ─────────────────────────────────────────────────────────────

bool isFinite3(const V3& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool isUnitQuat(const Q& q, float tol = 1e-3f) {
    float len = std::sqrt(q.data[0]*q.data[0] + q.data[1]*q.data[1] +
                          q.data[2]*q.data[2] + q.data[3]*q.data[3]);
    return std::abs(len - 1.f) < tol;
}

void assertBodySane(const BodyData& d, const std::string& tag) {
    ASSERT_TRUE(isFinite3(d.transform.position))   << tag << ": position NaN/inf";
    ASSERT_TRUE(isFinite3(d.linearVelocity))       << tag << ": linVel NaN/inf";
    ASSERT_TRUE(isFinite3(d.angularVelocity))      << tag << ": angVel NaN/inf";
    ASSERT_TRUE(isUnitQuat(d.transform.rotation))  << tag << ": rotation not unit";
}

void runAndAssertSane(PhysicsWorld& w, int steps, float dt = kDt,
                      float posLimit = 1000.f) {
    for (int i = 0; i < steps; ++i) {
        w.step(dt);
        const BodyPool& pool = w.bodyPool();
        for (uint32_t id = 0; id < pool.capacity(); ++id) {
            if (!pool.isValid(id)) continue;
            const BodyData& d = pool.get(id);
            if (d.type == BodyType::Static || d.isSleeping) continue;
            std::string tag = "step " + std::to_string(i) + " body " + std::to_string(id);
            assertBodySane(d, tag);
            ASSERT_LT(std::abs(d.transform.position.x()), posLimit) << tag << ": x out of bounds";
            ASSERT_LT(std::abs(d.transform.position.y()), posLimit) << tag << ": y out of bounds";
            ASSERT_LT(std::abs(d.transform.position.z()), posLimit) << tag << ": z out of bounds";
        }
    }
}

Body makeStaticFloor(PhysicsWorld& w, V3 pos = {0.f, -0.5f, 0.f}) {
    BodyDescriptor d;
    d.type      = BodyType::Static;
    d.mass      = 0.f;
    d.transform = { pos, Q::identity() };
    d.shape     = std::make_shared<BoxShape>(V3(50.f, 0.5f, 50.f));
    return w.createBody(d);
}

Body makeSphere(PhysicsWorld& w, float radius, float mass, V3 pos) {
    BodyDescriptor d;
    d.type      = BodyType::Dynamic;
    d.mass      = mass;
    d.transform = { pos, Q::identity() };
    d.shape     = std::make_shared<SphereShape>(radius);
    return w.createBody(d);
}

Body makeBox(PhysicsWorld& w, V3 half, float mass, V3 pos, Q rot = Q::identity()) {
    BodyDescriptor d;
    d.type      = BodyType::Dynamic;
    d.mass      = mass;
    d.transform = { pos, rot };
    d.shape     = std::make_shared<BoxShape>(half);
    return w.createBody(d);
}

} // namespace

// ── 1. Box tower: 10 boxes stacked, 300 steps, no explosion ───────────────────

TEST(Stress, BoxTowerStability) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    makeStaticFloor(w);

    const int kN = 10;
    for (int i = 0; i < kN; ++i)
        makeBox(w, {0.5f, 0.5f, 0.5f}, 1.f, {0.f, 0.5f + float(i) * 1.1f, 0.f});

    runAndAssertSane(w, 300);

    // Boxes should settle near the floor, not explode
    const BodyPool& pool = w.bodyPool();
    for (uint32_t id = 0; id < pool.capacity(); ++id) {
        if (!pool.isValid(id)) continue;
        const BodyData& d = pool.get(id);
        if (d.type == BodyType::Static) continue;
        EXPECT_GT(d.transform.position.y(), -1.f) << "box " << id << " fell through floor";
        EXPECT_LT(d.transform.position.y(), float(kN) * 2.f) << "box " << id << " exploded upward";
    }
}

// ── 2. Sphere tower: 8 spheres, 300 steps ─────────────────────────────────────

TEST(Stress, SphereTowerStability) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    makeStaticFloor(w);

    const int kN = 8;
    for (int i = 0; i < kN; ++i)
        makeSphere(w, 0.5f, 1.f, {0.f, 0.5f + float(i) * 1.05f, 0.f});

    runAndAssertSane(w, 300);
}

// ── 3. Extreme mass ratio: 1000 kg box on 1 kg box ────────────────────────────

TEST(Stress, ExtremeMassRatio) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    makeStaticFloor(w);

    // Heavy on top, light on bottom
    makeBox(w, {0.5f, 0.5f, 0.5f},    1.f, {0.f, 0.5f, 0.f});
    makeBox(w, {0.5f, 0.5f, 0.5f}, 1000.f, {0.f, 1.6f, 0.f});

    runAndAssertSane(w, 200);

    // Bottom box should not fall below floor
    const BodyPool& pool = w.bodyPool();
    for (uint32_t id = 0; id < pool.capacity(); ++id) {
        if (!pool.isValid(id)) continue;
        const BodyData& d = pool.get(id);
        if (d.type == BodyType::Static) continue;
        EXPECT_GT(d.transform.position.y(), -0.5f) << "body " << id << " fell through floor";
    }
}

// ── 4. Deep penetration recovery: two spheres start 50% overlapping ───────────

TEST(Stress, DeepPenetrationRecovery) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    // Two spheres placed so they overlap by half their diameter
    Body a = makeSphere(w, 0.5f, 1.f, {0.f,  0.25f, 0.f});
    Body b = makeSphere(w, 0.5f, 1.f, {0.f, -0.25f, 0.f});

    runAndAssertSane(w, 120, kDt, 100.f);

    // After recovery they should be separated
    const auto& da = w.bodyPool().get(a.id());
    const auto& db = w.bodyPool().get(b.id());
    float sep = std::abs(da.transform.position.y() - db.transform.position.y());
    EXPECT_GE(sep, 0.8f) << "Spheres did not separate after deep penetration";
}

// ── 5. High-velocity collision: sphere at 50 m/s hits static wall ─────────────

TEST(Stress, HighVelocityCollision) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    // Static wall
    BodyDescriptor wd;
    wd.type      = BodyType::Static;
    wd.mass      = 0.f;
    wd.transform = { {10.f, 0.f, 0.f}, Q::identity() };
    wd.shape     = std::make_shared<BoxShape>(V3(0.5f, 5.f, 5.f));
    w.createBody(wd);

    // Fast sphere
    Body s = makeSphere(w, 0.5f, 1.f, {0.f, 0.f, 0.f});
    w.bodyPool().get(s.id()).linearVelocity = {50.f, 0.f, 0.f};

    w.setFixedTimestep(kDt);
    runAndAssertSane(w, 120, kDt, 500.f);

    // Sphere must have stopped or bounced — not tunneled through
    const auto& d = w.bodyPool().get(s.id());
    EXPECT_LT(d.transform.position.x(), 12.f) << "Sphere tunneled through wall";
}

// ── 6. Ball-socket chain: 15 links, 300 steps under gravity ───────────────────

TEST(Stress, BallSocketChainStability) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    const int kLinks = 15;
    const float kLinkLen = 0.5f;
    const float kRadius  = 0.15f;

    // Root: kinematic anchor at origin
    BodyDescriptor rd;
    rd.type      = BodyType::Kinematic;
    rd.mass      = 1.f;
    rd.transform = { {0.f, 0.f, 0.f}, Q::identity() };
    rd.shape     = std::make_shared<SphereShape>(kRadius);
    Body prev = w.createBody(rd);

    for (int i = 1; i <= kLinks; ++i) {
        Body cur = makeSphere(w, kRadius, 1.f, {0.f, -float(i) * kLinkLen, 0.f});
        V3 anchor = {0.f, kLinkLen * 0.5f, 0.f};
        w.addConstraint(BallSocketConstraint::create(prev, anchor, cur, {0.f, -kLinkLen * 0.5f, 0.f}));
        prev = cur;
    }

    runAndAssertSane(w, 300, kDt, 500.f);
}

// ── 7. Hinge chain: 8 hinges in series, under gravity ─────────────────────────

TEST(Stress, HingeChainStability) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    const int  kLinks   = 8;
    const float kLen    = 0.6f;
    const V3    kAxis   = {1.f, 0.f, 0.f};

    BodyDescriptor rd;
    rd.type      = BodyType::Static;
    rd.mass      = 0.f;
    rd.transform = { {0.f, 0.f, 0.f}, Q::identity() };
    rd.shape     = std::make_shared<SphereShape>(0.1f);
    Body prev = w.createBody(rd);

    for (int i = 1; i <= kLinks; ++i) {
        Body cur = makeBox(w, {0.05f, kLen * 0.5f, 0.05f}, 1.f,
                           {0.f, -float(i) * kLen, 0.f});
        V3 ancA = {0.f, -kLen * 0.5f, 0.f};
        V3 ancB = {0.f,  kLen * 0.5f, 0.f};
        w.addConstraint(HingeConstraint::create(prev, ancA, kAxis, cur, ancB, kAxis));
        prev = cur;
    }

    runAndAssertSane(w, 300, kDt, 500.f);
}

// ── 8. High angular velocity: spinning bodies, quaternion stays unit ───────────

TEST(Stress, HighAngularVelocityNoNaN) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    const float kSpin = 500.f; // rad/s
    const int   kN    = 6;

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1.f, 1.f);

    std::vector<Body> bodies;
    for (int i = 0; i < kN; ++i) {
        Body b = makeSphere(w, 0.3f, 1.f, {float(i) * 3.f, 0.f, 0.f});
        V3 spin = {dist(rng), dist(rng), dist(rng)};
        float len = std::sqrt(spin.x()*spin.x() + spin.y()*spin.y() + spin.z()*spin.z());
        if (len > 1e-6f) spin = spin * (kSpin / len);
        w.bodyPool().get(b.id()).angularVelocity = spin;
        bodies.push_back(b);
    }

    for (int i = 0; i < 300; ++i) {
        w.step(kDt);
        for (auto& b : bodies) {
            const auto& d = w.bodyPool().get(b.id());
            ASSERT_TRUE(isFinite3(d.angularVelocity))
                << "angVel NaN at step " << i;
            ASSERT_TRUE(isUnitQuat(d.transform.rotation))
                << "rotation not unit at step " << i;
        }
    }
}

// ── 9. Random large impulses: 20 bodies, random impulses every 10 steps ────────

TEST(Stress, RandomImpulseRobustness) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    makeStaticFloor(w);

    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> dist(-20.f, 20.f);

    std::vector<Body> bodies;
    for (int i = 0; i < 20; ++i) {
        float x = float(i % 5) * 2.f - 4.f;
        float z = float(i / 5) * 2.f;
        bodies.push_back(makeSphere(w, 0.3f, 1.f, {x, 2.f + float(i) * 0.5f, z}));
    }

    for (int step = 0; step < 240; ++step) {
        if (step % 10 == 0) {
            for (auto& b : bodies) {
                BodyData& d = w.bodyPool().get(b.id());
                if (d.isSleeping) continue;
                d.linearVelocity  = d.linearVelocity  + V3(dist(rng), dist(rng), dist(rng));
                d.angularVelocity = d.angularVelocity + V3(dist(rng), dist(rng), dist(rng));
            }
        }
        w.step(kDt);
        for (auto& b : bodies) {
            const auto& d = w.bodyPool().get(b.id());
            std::string tag = "step " + std::to_string(step);
            ASSERT_TRUE(isFinite3(d.transform.position))  << tag << ": pos NaN";
            ASSERT_TRUE(isFinite3(d.linearVelocity))      << tag << ": linVel NaN";
            ASSERT_TRUE(isUnitQuat(d.transform.rotation)) << tag << ": rot not unit";
        }
    }
}

// ── 10. Constraint drift: ball socket, measure anchor error after 300 steps ────

TEST(Stress, BallSocketConstraintDrift) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    // Static anchor at origin
    BodyDescriptor sd;
    sd.type  = BodyType::Static;
    sd.mass  = 0.f;
    sd.shape = std::make_shared<SphereShape>(0.1f);
    Body anchor = w.createBody(sd);

    // Dynamic pendulum bob: hangs 1 m below
    Body bob = makeSphere(w, 0.2f, 1.f, {0.f, -1.f, 0.f});
    w.addConstraint(BallSocketConstraint::create(
        anchor, {0.f, 0.f, 0.f},
        bob,    {0.f, 0.5f, 0.f}));

    for (int i = 0; i < 300; ++i) w.step(kDt);

    // Measure anchor error
    const auto& da  = w.bodyPool().get(anchor.id());
    const auto& db  = w.bodyPool().get(bob.id());
    const auto  wAnchorA = da.transform.position; // anchor offset is {0,0,0}
    const auto  rB = db.transform.rotation.rotated(V3(0.f, 0.5f, 0.f));
    const auto  wAnchorB = db.transform.position + rB;
    float err = std::sqrt(
        (wAnchorA.x()-wAnchorB.x())*(wAnchorA.x()-wAnchorB.x()) +
        (wAnchorA.y()-wAnchorB.y())*(wAnchorA.y()-wAnchorB.y()) +
        (wAnchorA.z()-wAnchorB.z())*(wAnchorA.z()-wAnchorB.z()));

    EXPECT_LT(err, 0.02f) << "Ball socket anchor error too large after 300 steps: " << err;
}

// ── 11. Fixed constraint drift: two heavy bodies after 300 steps ───────────────

TEST(Stress, FixedConstraintDrift) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    Body a = makeBox(w, {0.5f, 0.5f, 0.5f}, 10.f, {0.f,  5.f, 0.f});
    Body b = makeBox(w, {0.5f, 0.5f, 0.5f}, 10.f, {0.f, 3.5f, 0.f});
    w.addConstraint(FixedConstraint::create(a, b));

    for (int i = 0; i < 300; ++i) w.step(kDt);

    const auto& da = w.bodyPool().get(a.id());
    const auto& db = w.bodyPool().get(b.id());
    float dist = std::sqrt(
        (da.transform.position.x()-db.transform.position.x())*(da.transform.position.x()-db.transform.position.x()) +
        (da.transform.position.y()-db.transform.position.y())*(da.transform.position.y()-db.transform.position.y()) +
        (da.transform.position.z()-db.transform.position.z())*(da.transform.position.z()-db.transform.position.z()));

    EXPECT_NEAR(dist, 1.5f, 0.05f) << "Fixed constraint separation drifted: " << dist;
}

// ── 12. Position solve convergence: body starts with 5 cm penetration ─────────

TEST(Stress, ContactPositionSolveConverges) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    makeStaticFloor(w);

    // Sphere placed 5 cm below its natural resting height (into the floor)
    Body s = makeSphere(w, 0.5f, 1.f, {0.f, 0.45f, 0.f});  // should be at 0.5

    // Run briefly — position solve should push it out and it should settle
    runAndAssertSane(w, 120);

    const auto& d = w.bodyPool().get(s.id());
    EXPECT_GT(d.transform.position.y(), 0.3f)  << "Sphere still inside floor";
    EXPECT_LT(d.transform.position.y(), 1.5f)  << "Sphere shot upward";
}

// ── 13. Many simultaneous contacts: 5×5 grid of spheres on floor ──────────────

TEST(Stress, ManySimultaneousContacts) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    makeStaticFloor(w);

    for (int x = 0; x < 5; ++x)
        for (int z = 0; z < 5; ++z)
            makeSphere(w, 0.4f, 1.f,
                {float(x) * 1.0f - 2.f, 0.4f + 0.01f * float(x + z), float(z) * 1.0f - 2.f});

    runAndAssertSane(w, 200);
}

// ── 14. Pendulum with constraint: angle stays bounded ─────────────────────────

TEST(Stress, PendulumAngleBounded) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    BodyDescriptor sd;
    sd.type  = BodyType::Static;
    sd.mass  = 0.f;
    sd.shape = std::make_shared<SphereShape>(0.1f);
    Body pivot = w.createBody(sd);

    Body bob = makeSphere(w, 0.3f, 2.f, {1.f, 0.f, 0.f}); // released from horizontal
    w.addConstraint(BallSocketConstraint::create(
        pivot, {0.f, 0.f, 0.f},
        bob,   {0.f, 0.f, 0.f}));

    for (int i = 0; i < 600; ++i) {
        w.step(kDt);
        const auto& d = w.bodyPool().get(bob.id());
        // Bob should remain within 1.5 m of pivot (rope length = 1 m + tolerance)
        float r = std::sqrt(d.transform.position.x()*d.transform.position.x() +
                            d.transform.position.y()*d.transform.position.y() +
                            d.transform.position.z()*d.transform.position.z());
        ASSERT_LT(r, 1.5f) << "Pendulum drifted too far at step " << i;
    }
}

// ── 15. Kinematic body collision: fast kinematic pushes dynamic body ───────────

TEST(Stress, KinematicBodyPushStaysFinite) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    // Fast kinematic "paddle"
    BodyDescriptor kd;
    kd.type      = BodyType::Kinematic;
    kd.mass      = 1.f;
    kd.transform = { {-3.f, 0.f, 0.f}, Q::identity() };
    kd.shape     = std::make_shared<BoxShape>(V3(0.5f, 1.f, 1.f));
    Body paddle = w.createBody(kd);

    // Dynamic target
    Body target = makeSphere(w, 0.5f, 1.f, {0.f, 0.f, 0.f});

    // Move kinematic body toward target each step
    for (int i = 0; i < 180; ++i) {
        BodyData& pd = w.bodyPool().get(paddle.id());
        if (pd.transform.position.x() < 5.f)
            pd.linearVelocity = {5.f, 0.f, 0.f};
        else
            pd.linearVelocity = {0.f, 0.f, 0.f};
        w.step(kDt);

        const auto& d = w.bodyPool().get(target.id());
        ASSERT_TRUE(isFinite3(d.transform.position)) << "target pos NaN at step " << i;
        ASSERT_TRUE(isFinite3(d.linearVelocity))     << "target vel NaN at step " << i;
    }
}
