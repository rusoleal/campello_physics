#include <campello_physics/physics_world.h>
#include <campello_physics/vehicle.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <gtest/gtest.h>
#include <cmath>

using namespace campello::physics;

static void runSteps(PhysicsWorld& w, int n) {
    for (int i = 0; i < n; ++i)
        w.step(1.f / 60.f);
}

// Build a minimal 4-wheel vehicle descriptor over a flat plane.
// Chassis: 2×1×4 box at height 1.5, 4 wheels at corners.
static VehicleDescriptor makeCarDesc() {
    VehicleDescriptor vd;

    BodyDescriptor chassis;
    chassis.type  = BodyType::Dynamic;
    chassis.mass  = 1200.f;
    chassis.shape = std::make_shared<BoxShape>(vm::Vector3<float>(1.f, 0.4f, 2.f));
    Transform t   = Transform::identity();
    t.position    = vm::Vector3<float>(0.f, 1.5f, 0.f);
    chassis.transform   = t;
    chassis.linearDamping  = 0.1f;
    chassis.angularDamping = 0.3f;
    vd.chassis = chassis;

    vd.maxEngineForce = 4000.f;
    vd.maxBrakeForce  = 8000.f;

    // Four wheels — corners of a 1.8 m × 3.6 m footprint
    for (int side : {-1, 1}) {
        for (int fore : {-1, 1}) {
            WheelDescriptor wd;
            wd.attachmentLocal = vm::Vector3<float>(side * 1.f, -0.4f, fore * 1.8f);
            wd.restLength      = 0.3f;
            wd.stiffness       = 35000.f;
            wd.damping         = 3000.f;
            wd.radius          = 0.35f;
            wd.maxSteerAngle   = (fore == 1) ? 0.5f : 0.f;  // front wheels steer
            wd.lateralFriction = 1.5f;
            vd.wheels.push_back(wd);
        }
    }
    return vd;
}

// ── Vehicle has correct wheel count ──────────────────────────────────────────

TEST(Vehicle, WheelCount) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    VehicleBody vb = world.createVehicle(makeCarDesc());
    EXPECT_TRUE(vb.isValid());
    EXPECT_TRUE(vb.chassisBody().isValid());
    EXPECT_EQ(world.vehicleWheelCount(vb), 4);
}

// ── Vehicle chassis body exists in pool ───────────────────────────────────────

TEST(Vehicle, ChassisIsInPool) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    const uint32_t before = world.bodyPool().activeCount();
    VehicleBody vb = world.createVehicle(makeCarDesc());
    EXPECT_EQ(world.bodyPool().activeCount(), before + 1);
}

// ── Destroy removes the chassis ───────────────────────────────────────────────

TEST(Vehicle, DestroyRemovesChassis) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    const uint32_t before = world.bodyPool().activeCount();
    VehicleBody vb = world.createVehicle(makeCarDesc());
    world.destroyVehicle(vb);
    EXPECT_EQ(world.bodyPool().activeCount(), before);
}

// ── Vehicle resting on a plane: wheels should be grounded ────────────────────

TEST(Vehicle, WheelsGroundedOnPlane) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    // Static ground plane
    BodyDescriptor gnd;
    gnd.type  = BodyType::Static;
    gnd.shape = std::make_shared<BoxShape>(vm::Vector3<float>(50.f, 0.5f, 50.f));
    Transform gt = Transform::identity();
    gt.position = vm::Vector3<float>(0.f, -0.5f, 0.f);
    gnd.transform = gt;
    world.createBody(gnd);

    VehicleBody vb = world.createVehicle(makeCarDesc());

    // Settle the vehicle for ~1 s
    for (int i = 0; i < 60; ++i) {
        world.syncVehicleControls(vb);
        world.step(1.f / 60.f);
    }

    int groundedCount = 0;
    for (int i = 0; i < world.vehicleWheelCount(vb); ++i) {
        if (world.vehicleWheelState(vb, i).isGrounded) ++groundedCount;
    }
    EXPECT_GE(groundedCount, 2) << "At least 2 wheels should be grounded after settling";
}

// ── Vehicle accelerates when throttle is applied ──────────────────────────────

TEST(Vehicle, ThrottleAcceleratesVehicle) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    BodyDescriptor gnd;
    gnd.type  = BodyType::Static;
    gnd.shape = std::make_shared<BoxShape>(vm::Vector3<float>(200.f, 0.5f, 200.f));
    Transform gt = Transform::identity();
    gt.position  = vm::Vector3<float>(0.f, -0.5f, 0.f);
    gnd.transform = gt;
    world.createBody(gnd);

    VehicleBody vb = world.createVehicle(makeCarDesc());

    // Settle first (no throttle)
    for (int i = 0; i < 60; ++i) {
        world.syncVehicleControls(vb);
        world.step(1.f / 60.f);
    }

    const float startZ = vb.chassisBody().transform().position.z();

    // Apply throttle for ~2 s
    vb.throttle = 1.f;
    for (int i = 0; i < 120; ++i) {
        world.syncVehicleControls(vb);
        world.step(1.f / 60.f);
    }

    const float endZ = vb.chassisBody().transform().position.z();
    EXPECT_NE(endZ, startZ) << "Vehicle should move under throttle";
}

// ── Invalid handle returns safe defaults ──────────────────────────────────────

TEST(Vehicle, InvalidHandleIsSafe) {
    VehicleBody vb;
    EXPECT_FALSE(vb.isValid());
    EXPECT_FALSE(vb.chassisBody().isValid());

    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    EXPECT_EQ(world.vehicleWheelCount(vb), 0);
    EXPECT_FALSE(world.vehicleWheelState(vb, 0).isGrounded);
}
