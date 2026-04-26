#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/buoyancy.h>
#include <gtest/gtest.h>

using namespace campello::physics;

static void runSteps(PhysicsWorld& w, int n) {
    for (int i = 0; i < n; ++i)
        w.step(1.f / 60.f);
}

// ── Floating: a sphere less dense than water should rise ─────────────────────

TEST(Buoyancy, SphereLessDenseThanWaterFloats) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    // Fluid volume: a 20×10×20 box centred at origin (top Y = 5)
    BuoyancyDescriptor fluid;
    fluid.shape        = std::make_shared<BoxShape>(vm::Vector3<float>(10.f, 5.f, 10.f));
    fluid.transform    = Transform::identity();
    fluid.fluidDensity = 1000.f;
    fluid.linearDrag   = 2.0f;
    fluid.angularDrag  = 1.0f;
    world.addBuoyancyVolume(fluid);

    // Sphere radius 0.5, mass 0.1 → density ≈ 191 kg/m³ < 1000 → should float
    BodyDescriptor bd;
    bd.type           = BodyType::Dynamic;
    bd.shape          = std::make_shared<SphereShape>(0.5f);
    bd.mass           = 0.1f;
    bd.transform      = Transform::identity();
    bd.linearDamping  = 0.f;
    bd.angularDamping = 0.f;
    Body sphere = world.createBody(bd);

    const float startY = sphere.transform().position.y();
    runSteps(world, 120);
    EXPECT_GT(sphere.transform().position.y(), startY) << "Under-dense sphere should rise";
}

// ── Sinking: a sphere denser than water should sink ──────────────────────────

TEST(Buoyancy, SphereDenserThanWaterSinks) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    BuoyancyDescriptor fluid;
    fluid.shape        = std::make_shared<BoxShape>(vm::Vector3<float>(10.f, 5.f, 10.f));
    fluid.transform    = Transform::identity();
    fluid.fluidDensity = 1000.f;
    fluid.linearDrag   = 0.1f;
    fluid.angularDrag  = 0.1f;
    [[maybe_unused]] auto fv = world.addBuoyancyVolume(fluid);

    // Sphere radius=0.1, mass=20 → V≈4.19e-3 m³, density≈4774 kg/m³ >> 1000 → sinks
    // Buoyant force ≈ 41 N up, gravity ≈ 196 N down → net 155 N downward
    [[maybe_unused]] auto fv2 = fv;  // suppress nodiscard warning
    BodyDescriptor bd;
    bd.type           = BodyType::Dynamic;
    bd.shape          = std::make_shared<SphereShape>(0.1f);
    bd.mass           = 20.f;
    bd.transform      = Transform::identity();
    bd.linearDamping  = 0.f;
    bd.angularDamping = 0.f;
    Body sphere = world.createBody(bd);

    const float startY = sphere.transform().position.y();
    runSteps(world, 120);
    EXPECT_LT(sphere.transform().position.y(), startY) << "Over-dense sphere should sink";
}

// ── Body above the fluid volume is unaffected ─────────────────────────────────

TEST(Buoyancy, BodyAboveFluidUnaffected) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    // Fluid occupies Y ∈ [-5, 5]
    BuoyancyDescriptor fluid;
    fluid.shape        = std::make_shared<BoxShape>(vm::Vector3<float>(10.f, 5.f, 10.f));
    fluid.transform    = Transform::identity();
    fluid.fluidDensity = 1000.f;
    [[maybe_unused]] auto fvol = world.addBuoyancyVolume(fluid);

    // Sphere starts at Y=20, far above the fluid
    BodyDescriptor bd;
    bd.type           = BodyType::Dynamic;
    bd.shape          = std::make_shared<SphereShape>(0.5f);
    bd.mass           = 1.f;
    bd.linearDamping  = 0.f;
    bd.angularDamping = 0.f;
    Transform t       = Transform::identity();
    t.position        = vm::Vector3<float>(0.f, 20.f, 0.f);
    bd.transform      = t;
    Body sphere = world.createBody(bd);

    runSteps(world, 30);
    // Should fall freely — below start but still well above the fluid surface
    EXPECT_LT(sphere.transform().position.y(), 20.f) << "Sphere should fall freely";
    EXPECT_GT(sphere.transform().position.y(),  5.f) << "Sphere should not have entered fluid yet";
}

// ── Removing the volume stops buoyancy ───────────────────────────────────────

TEST(Buoyancy, RemoveVolumeStopsBuoyancy) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    // High-drag fluid so the sphere reaches near-equilibrium quickly.
    BuoyancyDescriptor fluid;
    fluid.shape        = std::make_shared<BoxShape>(vm::Vector3<float>(10.f, 50.f, 10.f));
    fluid.transform    = Transform::identity();  // top Y = 50
    fluid.fluidDensity = 1000.f;
    fluid.linearDrag   = 10.0f;
    BuoyancyVolume vol = world.addBuoyancyVolume(fluid);

    // Under-dense sphere; high body damping to damp out overshoot fast.
    BodyDescriptor bd;
    bd.type           = BodyType::Dynamic;
    bd.shape          = std::make_shared<SphereShape>(0.5f);
    bd.mass           = 0.1f;   // density ≈ 191 kg/m³ << 1000 → floats
    bd.transform      = Transform::identity();
    bd.linearDamping  = 2.0f;
    bd.angularDamping = 0.f;
    Body sphere = world.createBody(bd);

    // Let it reach approximate equilibrium (≈3 s)
    runSteps(world, 180);
    const float yEquilibrium = sphere.transform().position.y();

    // Remove the fluid volume — only gravity remains
    world.removeBuoyancyVolume(vol);

    // After sufficient time without buoyancy, sphere must fall below equilibrium
    runSteps(world, 300);
    const float yAfterRemove = sphere.transform().position.y();

    EXPECT_LT(yAfterRemove, yEquilibrium)
        << "After removing buoyancy volume, sphere should sink below equilibrium";
}

// ── BuoyancyVolume handle validity ───────────────────────────────────────────

TEST(Buoyancy, HandleValidity) {
    BuoyancyVolume invalid;
    EXPECT_FALSE(invalid.isValid());

    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    BuoyancyDescriptor fluid;
    fluid.shape        = std::make_shared<BoxShape>(vm::Vector3<float>(10.f, 5.f, 10.f));
    fluid.transform    = Transform::identity();
    fluid.fluidDensity = 1000.f;

    BuoyancyVolume vol = world.addBuoyancyVolume(fluid);
    EXPECT_TRUE(vol.isValid());

    world.removeBuoyancyVolume(vol);
    EXPECT_TRUE(vol.isValid());  // handle still carries the ID — validity = ID is set
}

// ── Box shape also receives buoyancy ─────────────────────────────────────────

TEST(Buoyancy, BoxShapeReceivesBuoyancy) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    BuoyancyDescriptor fluid;
    fluid.shape        = std::make_shared<BoxShape>(vm::Vector3<float>(10.f, 5.f, 10.f));
    fluid.transform    = Transform::identity();
    fluid.fluidDensity = 1000.f;
    fluid.linearDrag   = 2.0f;
    [[maybe_unused]] auto fvol2 = world.addBuoyancyVolume(fluid);

    // 1×1×1 box, mass=0.1 → density=100 < 1000 → should float
    BodyDescriptor bd;
    bd.type           = BodyType::Dynamic;
    bd.shape          = std::make_shared<BoxShape>(vm::Vector3<float>(0.5f, 0.5f, 0.5f));
    bd.mass           = 0.1f;
    bd.transform      = Transform::identity();
    bd.linearDamping  = 0.f;
    bd.angularDamping = 0.f;
    Body box = world.createBody(bd);

    const float startY = box.transform().position.y();
    runSteps(world, 120);
    EXPECT_GT(box.transform().position.y(), startY) << "Under-dense box should rise";
}
