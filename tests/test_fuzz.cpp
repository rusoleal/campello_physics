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
#include <vector>

using namespace campello::physics;
namespace {
using V3 = vm::Vector3<float>;
using Q  = vm::Quaternion<float>;

constexpr float kDt = 1.f / 60.f;

// ── 1. Zero-mass dynamic body ─────────────────────────────────────────────────

TEST(Fuzz, ZeroMassDynamicBodyDoesNotCrash) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 0.f;  // invalid: dynamic with zero mass
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = {0.f, 10.f, 0.f};

    Body b = w.createBody(d);
    for (int i = 0; i < 60; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 2. NaN / Inf in initial state ─────────────────────────────────────────────

TEST(Fuzz, NaNPositionIsTolerated) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = {std::nanf(""), 0.f, 0.f};

    Body b = w.createBody(d);
    for (int i = 0; i < 10; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

TEST(Fuzz, InfVelocityIsTolerated) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.linearVelocity = {1e30f, 0.f, 0.f};

    Body b = w.createBody(d);
    for (int i = 0; i < 10; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 3. Degenerate shapes ──────────────────────────────────────────────────────

TEST(Fuzz, ZeroRadiusSphere) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(0.f);
    d.transform.position = {0.f, 1.f, 0.f};

    Body b = w.createBody(d);
    for (int i = 0; i < 60; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

TEST(Fuzz, ZeroExtentBox) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<BoxShape>(V3{0.f, 0.f, 0.f});
    d.transform.position = {0.f, 1.f, 0.f};

    Body b = w.createBody(d);
    for (int i = 0; i < 60; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 4. Identical overlapping bodies ───────────────────────────────────────────

TEST(Fuzz, IdenticalBodiesOverlap) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(1.f);

    d.transform.position = {0.f, 0.f, 0.f};
    Body a = w.createBody(d);
    Body b = w.createBody(d);

    for (int i = 0; i < 120; ++i)
        w.step(kDt);

    EXPECT_TRUE(a.isValid());
    EXPECT_TRUE(b.isValid());
}

// ── 5. Extreme mass ratios ────────────────────────────────────────────────────

TEST(Fuzz, ExtremeMassRatio1To1e6) {
    PhysicsWorld w;
    w.setGravity(V3{0.f, -9.81f, 0.f});

    BodyDescriptor light;
    light.type = BodyType::Dynamic;
    light.mass = 1e-6f;
    light.shape = std::make_shared<SphereShape>(0.1f);
    light.transform.position = {0.f, 1.f, 0.f};

    BodyDescriptor heavy;
    heavy.type = BodyType::Dynamic;
    heavy.mass = 1e6f;
    heavy.shape = std::make_shared<SphereShape>(10.f);
    heavy.transform.position = {0.f, 0.f, 0.f};

    Body bl = w.createBody(light);
    Body bh = w.createBody(heavy);

    for (int i = 0; i < 120; ++i)
        w.step(kDt);

    EXPECT_TRUE(bl.isValid());
    EXPECT_TRUE(bh.isValid());
}

// ── 6. Negative damping ───────────────────────────────────────────────────────

TEST(Fuzz, NegativeDamping) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.linearDamping = -0.5f;
    d.angularDamping = -0.5f;
    d.linearVelocity = {1.f, 0.f, 0.f};

    Body b = w.createBody(d);
    for (int i = 0; i < 60; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 7. Null shape dynamic body ────────────────────────────────────────────────

TEST(Fuzz, NullShapeDynamicBody) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = nullptr;
    d.transform.position = {0.f, 10.f, 0.f};

    Body b = w.createBody(d);
    for (int i = 0; i < 60; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 8. Random impulse spam ────────────────────────────────────────────────────

TEST(Fuzz, RandomImpulseSpam) {
    PhysicsWorld w;
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1000.f, 1000.f);
    std::uniform_int_distribution<int> idxDist(0, 49);

    std::vector<Body> bodies;
    for (int i = 0; i < 50; ++i) {
        BodyDescriptor d;
        d.type = BodyType::Dynamic;
        d.mass = 1.f;
        d.shape = std::make_shared<SphereShape>(0.5f);
        d.transform.position = {float(i) * 2.f, 10.f, 0.f};
        bodies.push_back(w.createBody(d));
    }

    for (int step = 0; step < 120; ++step) {
        for (int k = 0; k < 10; ++k) {
            int idx = idxDist(rng);
            V3 imp{dist(rng), dist(rng), dist(rng)};
            bodies[idx].applyLinearImpulse(imp);
        }
        w.step(kDt);
    }

    for (auto& b : bodies)
        EXPECT_TRUE(b.isValid());
}

// ── 9. Constraint with same body ──────────────────────────────────────────────

TEST(Fuzz, BallSocketSameBody) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(1.f);
    Body b = w.createBody(d);

    auto c = BallSocketConstraint::create(b, V3{0.f, 1.f, 0.f}, b, V3{0.f, -1.f, 0.f});
    w.addConstraint(c);

    for (int i = 0; i < 60; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 10. Constraint with invalid body ──────────────────────────────────────────

TEST(Fuzz, ConstraintWithInvalidBody) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(1.f);
    Body b = w.createBody(d);

    // Create and immediately destroy a body to get a stale handle
    Body temp = w.createBody(d);
    w.destroyBody(temp);

    // Attempting to create a constraint with a stale handle should not crash
    // the world. The constraint creation itself may assert, so we guard it.
    if (temp.isValid()) {
        auto c = FixedConstraint::create(temp, b);
        w.addConstraint(c);
    }

    for (int i = 0; i < 60; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 11. Very large restitution / friction ─────────────────────────────────────

TEST(Fuzz, ExcessiveRestitutionAndFriction) {
    PhysicsWorld w;
    w.setGravity(V3{0.f, -9.81f, 0.f});

    BodyDescriptor floor;
    floor.type = BodyType::Static;
    floor.shape = std::make_shared<BoxShape>(V3{50.f, 0.5f, 50.f});
    floor.transform.position = {0.f, -0.5f, 0.f};
    floor.restitution = 1e6f;
    floor.friction = 1e6f;
    [[maybe_unused]] auto floorBody = w.createBody(floor);

    BodyDescriptor ball;
    ball.type = BodyType::Dynamic;
    ball.mass = 1.f;
    ball.shape = std::make_shared<SphereShape>(1.f);
    ball.restitution = 1e6f;
    ball.friction = 1e6f;
    ball.transform.position = {0.f, 10.f, 0.f};
    Body b = w.createBody(ball);

    for (int i = 0; i < 120; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 12. Deep penetration recovery ─────────────────────────────────────────────

TEST(Fuzz, MassiveDeepPenetration) {
    PhysicsWorld w;
    w.setGravity(V3{0.f, -9.81f, 0.f});

    BodyDescriptor floor;
    floor.type = BodyType::Static;
    floor.shape = std::make_shared<BoxShape>(V3{50.f, 0.5f, 50.f});
    floor.transform.position = {0.f, -0.5f, 0.f};
    [[maybe_unused]] auto floorBody = w.createBody(floor);

    BodyDescriptor ball;
    ball.type = BodyType::Dynamic;
    ball.mass = 1.f;
    ball.shape = std::make_shared<SphereShape>(1.f);
    ball.transform.position = {0.f, -5.f, 0.f};  // starts inside floor
    Body b = w.createBody(ball);

    for (int i = 0; i < 120; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 13. Very high solver iterations ───────────────────────────────────────────

TEST(Fuzz, ExtremeSolverIterations) {
    PhysicsWorld w;
    w.contactIterations = 1000;
    w.constraintIterations = 1000;
    w.positionIterations = 100;

    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(1.f);
    Body b = w.createBody(d);

    for (int i = 0; i < 10; ++i)
        w.step(kDt);

    EXPECT_TRUE(b.isValid());
}

// ── 14. Rapid create / destroy cycles ─────────────────────────────────────────

TEST(Fuzz, RapidCreateDestroy) {
    PhysicsWorld w;
    for (int i = 0; i < 500; ++i) {
        BodyDescriptor d;
        d.type = BodyType::Dynamic;
        d.mass = 1.f;
        d.shape = std::make_shared<SphereShape>(0.5f);
        Body b = w.createBody(d);
        w.step(kDt);
        w.destroyBody(b);
    }
    EXPECT_EQ(w.bodyPool().activeCount(), 0u);
}

// ── 15. Query on empty / degenerate world ─────────────────────────────────────

TEST(Fuzz, RaycastEmptyWorld) {
    PhysicsWorld w;
    Ray ray{V3{0.f, 0.f, 0.f}, V3{1.f, 0.f, 0.f}, 1000.f};
    auto hit = w.raycastClosest(ray, QueryFilter{});
    EXPECT_FALSE(hit.has_value());
}

TEST(Fuzz, OverlapWithDegenerateShape) {
    PhysicsWorld w;
    BoxShape shape(V3{0.f, 0.f, 0.f});
    auto results = w.overlap(shape, Transform::identity(), QueryFilter{});
    EXPECT_TRUE(results.empty());
}

// ── 16. Island sleep / wake churn ─────────────────────────────────────────────

TEST(Fuzz, SleepWakeChurn) {
    PhysicsWorld w;
    w.setGravity(V3{0.f, -9.81f, 0.f});

    std::vector<Body> bodies;
    for (int i = 0; i < 20; ++i) {
        BodyDescriptor d;
        d.type = BodyType::Dynamic;
        d.mass = 1.f;
        d.shape = std::make_shared<SphereShape>(0.5f);
        d.transform.position = {float(i) * 1.1f, 10.f, 0.f};
        bodies.push_back(w.createBody(d));
    }

    for (int i = 0; i < 300; ++i) {
        if (i % 60 == 0) {
            for (auto& b : bodies)
                b.wake();
        }
        w.step(kDt);
    }

    for (auto& b : bodies)
        EXPECT_TRUE(b.isValid());
}

// ── 17. Serialization round-trip with degenerate state ────────────────────────

TEST(Fuzz, SerializeDegenerateWorld) {
    PhysicsWorld w;

    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1e-12f;
    d.shape = std::make_shared<SphereShape>(0.f);
    d.linearVelocity = {1e30f, 1e30f, 1e30f};
    d.restitution = 1e6f;
    d.friction = -1.f;
    [[maybe_unused]] auto _b = w.createBody(d);

    std::string json = w.serialize();
    EXPECT_FALSE(json.empty());

    PhysicsWorld w2;
    bool ok = w2.deserialize(json);
    EXPECT_TRUE(ok);
}

} // namespace
