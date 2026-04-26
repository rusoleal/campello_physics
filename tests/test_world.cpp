#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <campello_physics/listeners.h>
#include <cmath>
#include <vector>

using namespace campello::physics;

namespace {
using V3 = vm::Vector3<float>;
using Q  = vm::Quaternion<float>;
} // namespace

// ── Body lifecycle ─────────────────────────────────────────────────────────────

TEST(World, CreateDestroyBody) {
    PhysicsWorld world;
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;

    Body b = world.createBody(d);
    EXPECT_TRUE(b.isValid());

    world.destroyBody(b);
    EXPECT_FALSE(b.isValid());
}

TEST(World, GravityAccessors) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    EXPECT_NEAR(world.gravity().y(), -9.81f, 1e-5f);
}

// ── Free-fall through world step ───────────────────────────────────────────────

TEST(World, StepIntegratesGravity) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));

    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.linearDamping = d.angularDamping = 0.f;
    Body sphere = world.createBody(d);

    const float dt = 1.f / 60.f;
    for (int i = 0; i < 60; ++i)
        world.step(dt);

    // After 1 second of free-fall (semi-implicit Euler), velocity ≈ -g
    EXPECT_NEAR(sphere.linearVelocity().y(), -9.81f, 0.1f);
}

// ── Sphere bouncing on static plane ───────────────────────────────────────────

TEST(World, SphereBounceOnStaticPlane) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.contactIterations    = 20;
    world.constraintIterations = 20;

    // Static floor: box 10×0.2×10 centred at y = -0.1 (top surface at y = 0)
    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    floorDesc.restitution = 0.5f;
    [[maybe_unused]] auto floor = world.createBody(floorDesc);

    // Dynamic sphere radius 0.5, centre at y = 2 (1.5 m above floor surface)
    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 2.f, 0.f);
    sphereDesc.restitution = 0.5f;
    sphereDesc.linearDamping = sphereDesc.angularDamping = 0.f;
    Body sphere = world.createBody(sphereDesc);

    // Simulate 3 seconds
    const float dt = 1.f / 60.f;
    for (int i = 0; i < 180; ++i)
        world.step(dt);

    // Sphere should remain above the floor surface (y > -0.6 = -0.1 - 0.5)
    float y = sphere.transform().position.y();
    EXPECT_GT(y, -0.65f) << "Sphere fell through floor: y = " << y;

    // Sphere should have moved from its initial position (it was dropped)
    EXPECT_LT(y, 1.9f) << "Sphere didn't fall: y = " << y;
}

TEST(World, SphereRestingOnFloor) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.contactIterations    = 20;
    world.constraintIterations = 20;

    // Floor
    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    floorDesc.restitution = 0.f;  // no bounce → comes to rest
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 1.f, 0.f);
    sphereDesc.restitution = 0.f;
    sphereDesc.linearDamping  = 0.1f;
    sphereDesc.angularDamping = 0.1f;
    Body sphere = world.createBody(sphereDesc);

    // Simulate 5 seconds
    for (int i = 0; i < 300; ++i)
        world.step(1.f / 60.f);

    // Sphere should have settled near y = 0.5 (floor surface + radius)
    float y = sphere.transform().position.y();
    EXPECT_GT(y, -0.6f)  << "Sphere fell through floor: y = " << y;
    EXPECT_LT(y,  0.75f) << "Sphere not resting on floor: y = " << y;
}

// ── Contact listener ───────────────────────────────────────────────────────────

TEST(World, ContactListenerFires) {
    struct Listener : IContactListener {
        int added = 0, persisted = 0, removed = 0;
        void onContactAdded    (Body, Body, const ContactManifold&) override { ++added; }
        void onContactPersisted(Body, Body, const ContactManifold&) override { ++persisted; }
        void onContactRemoved  (Body, Body)                         override { ++removed; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));

    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 0.4f, 0.f);  // close to floor
    sphereDesc.restitution = 0.f;
    [[maybe_unused]] auto _sphere2 = world.createBody(sphereDesc);

    Listener listener;
    world.addContactListener(&listener);

    for (int i = 0; i < 60; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(listener.added + listener.persisted, 0)
        << "No contact events fired";

    world.removeContactListener(&listener);
}

// ── Trigger / sensor ───────────────────────────────────────────────────────────

TEST(World, TriggerListenerFires) {
    struct Listener : ITriggerListener {
        int entered = 0, exited = 0;
        void onTriggerEnter(Body, Body) override { ++entered; }
        void onTriggerExit (Body, Body) override { ++exited; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    // Sensor body at origin
    BodyDescriptor sensorDesc;
    sensorDesc.type  = BodyType::Sensor;
    sensorDesc.shape = std::make_shared<SphereShape>(1.f);
    sensorDesc.transform.position = V3(0.f, 0.f, 0.f);
    [[maybe_unused]] auto _sensor = world.createBody(sensorDesc);

    // Dynamic body starting far away, moving toward sensor
    BodyDescriptor bodyDesc;
    bodyDesc.type  = BodyType::Dynamic;
    bodyDesc.mass  = 1.f;
    bodyDesc.shape = std::make_shared<SphereShape>(0.5f);
    bodyDesc.transform.position = V3(5.f, 0.f, 0.f);
    bodyDesc.linearDamping = bodyDesc.angularDamping = 0.f;
    Body mover = world.createBody(bodyDesc);
    mover.setLinearVelocity(V3(-3.f, 0.f, 0.f));

    Listener listener;
    world.addTriggerListener(&listener);

    for (int i = 0; i < 120; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(listener.entered, 0) << "Trigger enter never fired";

    world.removeTriggerListener(&listener);
}

// ── Constraint round-trip through world ───────────────────────────────────────

TEST(World, ConstraintAddedThroughWorld) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.linearDamping = d.angularDamping = 0.f;

    d.transform.position = V3(0.f, 0.f, 0.f);
    Body a = world.createBody(d);
    d.transform.position = V3(2.f, 0.f, 0.f);
    Body b = world.createBody(d);

    auto c = BallSocketConstraint::create(
        a, V3(1.f,0.f,0.f), b, V3(-1.f,0.f,0.f));
    world.addConstraint(c);

    // Give them opposite velocities
    a.setLinearVelocity(V3(-3.f, 0.f, 0.f));
    b.setLinearVelocity(V3( 3.f, 0.f, 0.f));

    for (int i = 0; i < 120; ++i)
        world.step(1.f / 60.f);

    // Anchors should remain close
    const auto wA = a.transform().position + V3(1.f, 0.f, 0.f);
    const auto wB = b.transform().position + V3(-1.f, 0.f, 0.f);
    const auto diff = wA - wB;
    float dist = std::sqrt(diff.x()*diff.x() + diff.y()*diff.y() + diff.z()*diff.z());
    EXPECT_LT(dist, 0.1f) << "Anchor separation = " << dist;
}

// ── Fixed timestep accumulator ─────────────────────────────────────────────────

TEST(World, FixedTimestepAccumulator) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));

    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = 1.f;
    d.linearDamping = d.angularDamping = 0.f;
    Body sphere = world.createBody(d);

    // Feed a large dt — should sub-step internally
    world.step(0.5f);   // ~30 × 1/60 steps

    float vy = sphere.linearVelocity().y();
    EXPECT_LT(vy, -1.f) << "No steps fired for large dt";
}
