#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>

using namespace campello::physics;
using V3 = vm::Vector3<float>;

// ── Helpers ───────────────────────────────────────────────────────────────────

static Body makeStaticWall(PhysicsWorld& world, V3 center, V3 halfExtents) {
    BodyDescriptor d;
    d.type             = BodyType::Static;
    d.shape            = std::make_shared<BoxShape>(halfExtents);
    d.transform.position = center;
    d.restitution      = 0.f;
    d.friction         = 0.f;
    return world.createBody(d);
}

static Body makeFastSphere(PhysicsWorld& world, V3 pos, V3 vel,
                            float radius = 0.1f, bool ccd = true)
{
    BodyDescriptor d;
    d.type             = BodyType::Dynamic;
    d.mass             = 1.f;
    d.shape            = std::make_shared<SphereShape>(radius);
    d.transform.position = pos;
    d.linearVelocity   = vel;
    d.restitution      = 0.f;
    d.friction         = 0.f;
    d.ccdEnabled       = ccd;
    return world.createBody(d);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

// Without CCD the sphere tunnels through a thin wall in a single step.
TEST(CCD, WithoutCcdSphereWouldTunnel) {
    PhysicsWorld world;
    world.setGravity(V3(0, 0, 0));

    // Thin wall at x=0, half-thickness 0.05 (full thickness 0.1)
    makeStaticWall(world, V3(0, 0, 0), V3(0.05f, 2.f, 2.f));

    // Sphere radius=0.1 at x=-0.3, velocity 200 m/s rightward.
    // One step: 200*(1/60)≈3.33 units → sphere passes completely through the wall.
    Body sphere = makeFastSphere(world, V3(-0.3f, 0, 0), V3(200, 0, 0),
                                  0.1f, /*ccd=*/false);

    world.step(1.f / 60.f);

    // Sphere has tunnelled past the wall center (x=0)
    EXPECT_GT(sphere.transform().position.x(), 0.1f);
}

// CCD prevents the sphere from crossing the wall.
TEST(CCD, SphereStopsAtThinWall) {
    PhysicsWorld world;
    world.setGravity(V3(0, 0, 0));

    makeStaticWall(world, V3(0, 0, 0), V3(0.05f, 2.f, 2.f));
    Body sphere = makeFastSphere(world, V3(-0.3f, 0, 0), V3(200, 0, 0));

    // Run 10 steps; the sphere must stay on the approach side of the wall
    for (int i = 0; i < 10; ++i)
        world.step(1.f / 60.f);

    EXPECT_LT(sphere.transform().position.x(), 0.f);
}

// A slow body with CCD enabled still lands on a normal floor correctly.
TEST(CCD, SlowBodyFallsAndRests) {
    PhysicsWorld world;
    world.setGravity(V3(0, -9.81f, 0));

    makeStaticWall(world, V3(0, -0.5f, 0), V3(10, 0.5f, 10));

    BodyDescriptor d;
    d.type             = BodyType::Dynamic;
    d.mass             = 1.f;
    d.shape            = std::make_shared<SphereShape>(0.5f);
    d.transform.position = V3(0, 2, 0);
    d.restitution      = 0.f;
    d.friction         = 0.f;
    d.ccdEnabled       = true;
    Body sphere = world.createBody(d);

    for (int i = 0; i < 120; ++i)
        world.step(1.f / 60.f);

    float y = sphere.transform().position.y();
    EXPECT_GT(y, -0.1f);   // not through the floor
    EXPECT_LT(y,  0.8f);   // resting, not floating
}

// A CCD body moving away from a wall is not spuriously constrained.
TEST(CCD, SeparatingBodyMovesFreely) {
    PhysicsWorld world;
    world.setGravity(V3(0, 0, 0));

    // Wall far to the left — sphere moves away from it
    makeStaticWall(world, V3(-5, 0, 0), V3(0.05f, 2.f, 2.f));

    Body sphere = makeFastSphere(world, V3(0, 0, 0), V3(10, 0, 0));

    world.step(1.f / 60.f);

    float x = sphere.transform().position.x();
    // Should move ≈ 10/60 ≈ 0.167 units to the right — the wall plays no role
    EXPECT_NEAR(x, 10.f / 60.f, 0.02f);
}

// CCD works for sphere approaching from the opposite direction.
TEST(CCD, SphereApproachingFromRight) {
    PhysicsWorld world;
    world.setGravity(V3(0, 0, 0));

    makeStaticWall(world, V3(0, 0, 0), V3(0.05f, 2.f, 2.f));

    // Sphere to the right, moving left fast
    Body sphere = makeFastSphere(world, V3(0.3f, 0, 0), V3(-200, 0, 0));

    for (int i = 0; i < 10; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(sphere.transform().position.x(), 0.f);  // stays on approach side
}

// Sensor bodies are ignored by CCD.
TEST(CCD, SensorNotConstrainingCcdBody) {
    PhysicsWorld world;
    world.setGravity(V3(0, 0, 0));

    BodyDescriptor sd;
    sd.type             = BodyType::Sensor;
    sd.shape            = std::make_shared<BoxShape>(V3(0.05f, 2.f, 2.f));
    sd.transform.position = V3(0, 0, 0);
    [[maybe_unused]] auto unused = world.createBody(sd);

    Body sphere = makeFastSphere(world, V3(-0.3f, 0, 0), V3(200, 0, 0));

    world.step(1.f / 60.f);

    // Sensors don't block — sphere passes through freely
    EXPECT_GT(sphere.transform().position.x(), 0.1f);
}
