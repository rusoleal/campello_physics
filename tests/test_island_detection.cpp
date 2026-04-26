#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <campello_physics/constraints/fixed_constraint.h>
#include <cmath>

namespace campello::physics {

// ── helpers ───────────────────────────────────────────────────────────────────

static Body makeDynamic(PhysicsWorld& w, float radius = 0.5f, float mass = 1.f,
                        vm::Vector3<float> pos = {0.f, 0.f, 0.f}) {
    BodyDescriptor d;
    d.type            = BodyType::Dynamic;
    d.shape           = std::make_shared<SphereShape>(radius);
    d.mass            = mass;
    d.transform.position = pos;
    d.linearDamping   = 0.f;
    d.angularDamping  = 0.f;
    return w.createBody(d);
}

static Body makeStatic(PhysicsWorld& w, vm::Vector3<float> pos = {0.f, 0.f, 0.f}) {
    BodyDescriptor d;
    d.type            = BodyType::Static;
    d.shape           = std::make_shared<BoxShape>(vm::Vector3<float>(5.f, 0.5f, 5.f));
    d.transform.position = pos;
    return w.createBody(d);
}

static bool isSleeping(PhysicsWorld& w, Body b) {
    return w.bodyPool().get(b.id()).isSleeping;
}

static int sleepFrames(PhysicsWorld& w, Body b) {
    return w.bodyPool().get(b.id()).sleepFrames;
}

// ── 1. Single isolated body sleeps normally ───────────────────────────────────

TEST(IslandDetection, IsolatedBodySleepsNormally) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});
    Body b = makeDynamic(w);

    // No forces, no contacts → velocity stays zero → sleepFrames increments
    for (int i = 0; i < 80; ++i) w.step(1.f / 60.f);

    EXPECT_TRUE(isSleeping(w, b));
}

// ── 2. Two isolated islands sleep independently ───────────────────────────────

TEST(IslandDetection, IsolatedIslandsDontInterfere) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    // Island A: one body, far away, at rest — should sleep
    Body bA = makeDynamic(w, 0.5f, 1.f, {0.f, 0.f, 0.f});

    // Island B: one body with significant velocity — should stay awake
    Body bB = makeDynamic(w, 0.5f, 1.f, {100.f, 0.f, 0.f});
    w.bodyPool().get(bB.id()).linearVelocity = {5.f, 0.f, 0.f};

    for (int i = 0; i < 80; ++i) w.step(1.f / 60.f);

    EXPECT_TRUE(isSleeping(w, bA));
    EXPECT_FALSE(isSleeping(w, bB));
}

// ── 3. Wake propagation: active body wakes its sleeping island-mate ───────────

TEST(IslandDetection, WakePropagatesAcrossContact) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    // Static floor
    makeStatic(w, {0.f, -0.5f, 0.f});

    // A sphere resting on the floor — let it settle and sleep
    Body resting = makeDynamic(w, 0.5f, 1.f, {0.f, 0.5f, 0.f});
    for (int i = 0; i < 180; ++i) w.step(1.f / 60.f);
    ASSERT_TRUE(isSleeping(w, resting));

    // Drop another sphere from above — needs ~35 steps at 60Hz to travel 1.5m
    Body falling = makeDynamic(w, 0.5f, 1.f, {0.f, 3.f, 0.f});
    for (int i = 0; i < 60; ++i) w.step(1.f / 60.f);

    // Once the falling body contacts the resting one, resting must wake
    EXPECT_FALSE(isSleeping(w, resting));
}

// ── 4. Connected bodies sleep simultaneously ──────────────────────────────────

TEST(IslandDetection, ConnectedBodiesSleepTogether) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    // Two bodies linked by a fixed constraint — same island, no contact oscillation
    Body bA = makeDynamic(w, 0.5f, 1.f, { 0.f, 0.f, 0.f});
    Body bB = makeDynamic(w, 0.5f, 1.f, {10.f, 0.f, 0.f});
    w.addConstraint(FixedConstraint::create(bA, bB));

    // Both at rest with default damping → both drain to zero and sleep together
    for (int i = 0; i < 120; ++i) w.step(1.f / 60.f);

    EXPECT_TRUE(isSleeping(w, bA));
    EXPECT_TRUE(isSleeping(w, bB));
}

// ── 5. Constraint links two otherwise disconnected bodies into one island ──────

TEST(IslandDetection, ConstraintMergesIslands) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    // Two bodies far apart (no contact) but connected by a distance constraint
    Body bA = makeDynamic(w, 0.5f, 1.f, { 0.f, 0.f, 0.f});
    Body bB = makeDynamic(w, 0.5f, 1.f, {10.f, 0.f, 0.f});

    // Give B a velocity so it stays awake initially
    w.bodyPool().get(bB.id()).linearVelocity = {1.f, 0.f, 0.f};

    // Connect them — they must form one island
    auto c = FixedConstraint::create(bA, bB);
    w.addConstraint(c);

    // Run a few steps — bA would normally sleep quickly since it has zero velocity
    // but bB is awake, so bA must be kept awake too via island wake propagation
    for (int i = 0; i < 30; ++i) w.step(1.f / 60.f);

    // bA is in the same island as bB (which is moving) → bA must not be sleeping
    EXPECT_FALSE(isSleeping(w, bA));
}

// ── 6. Sleeping island is fully skipped by solver ────────────────────────────

TEST(IslandDetection, SleepingIslandDoesNotMove) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    // Static floor
    makeStatic(w, {0.f, -0.5f, 0.f});

    // Body resting on floor — settle and sleep
    Body b = makeDynamic(w, 0.5f, 1.f, {0.f, 0.5f, 0.f});
    for (int i = 0; i < 180; ++i) w.step(1.f / 60.f);
    ASSERT_TRUE(isSleeping(w, b));

    // Record position
    auto posBefore = w.bodyPool().get(b.id()).transform.position;

    // Step more — sleeping island must not move despite gravity
    for (int i = 0; i < 60; ++i) w.step(1.f / 60.f);
    auto posAfter = w.bodyPool().get(b.id()).transform.position;

    EXPECT_NEAR(posBefore.y(), posAfter.y(), 1e-6f);
    EXPECT_TRUE(isSleeping(w, b));
}

// ── 7. Stack of boxes: touching body wakes the whole stack ────────────────────

TEST(IslandDetection, StackWakesAsOneIsland) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});

    makeStatic(w, {0.f, -0.5f, 0.f});

    // Stack of 3 spheres
    Body b0 = makeDynamic(w, 0.5f, 1.f, {0.f, 0.5f, 0.f});
    Body b1 = makeDynamic(w, 0.5f, 1.f, {0.f, 1.5f, 0.f});
    Body b2 = makeDynamic(w, 0.5f, 1.f, {0.f, 2.5f, 0.f});

    // Let stack settle and sleep
    for (int i = 0; i < 300; ++i) w.step(1.f / 60.f);
    EXPECT_TRUE(isSleeping(w, b0));
    EXPECT_TRUE(isSleeping(w, b1));
    EXPECT_TRUE(isSleeping(w, b2));

    // Drop a fourth sphere from above — needs ~43 steps at 60Hz to travel 2.5m
    makeDynamic(w, 0.5f, 1.f, {0.f, 6.f, 0.f});

    // After impact, the top of the stack must wake
    for (int i = 0; i < 60; ++i) w.step(1.f / 60.f);
    EXPECT_FALSE(isSleeping(w, b2));
}

// ── 8. sleepFrames resets when body wakes up ─────────────────────────────────

TEST(IslandDetection, SleepFramesResetOnWake) {
    PhysicsWorld w;
    w.setGravity({0.f, 0.f, 0.f});

    Body b = makeDynamic(w, 0.5f, 1.f, {0.f, 0.f, 0.f});

    // Accumulate sleep frames
    for (int i = 0; i < 30; ++i) w.step(1.f / 60.f);
    EXPECT_GT(sleepFrames(w, b), 0);

    // Apply an impulse to wake it
    b.applyLinearImpulse({1.f, 0.f, 0.f});
    w.step(1.f / 60.f);

    EXPECT_EQ(sleepFrames(w, b), 0);
    EXPECT_FALSE(isSleeping(w, b));
}

} // namespace campello::physics
