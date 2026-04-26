#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <cmath>

using namespace campello::physics;

namespace {
using V3 = vm::Vector3<float>;

bool enableGpu(PhysicsWorld& world) {
    world.setBackend(PhysicsBackend::Gpu);
    return world.backend() == PhysicsBackend::Gpu;
}

float dist3(const V3& a, const V3& b) {
    float dx = a.x()-b.x(), dy = a.y()-b.y(), dz = a.z()-b.z();
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
} // namespace

// ── API tests (always run, no GPU hardware needed) ────────────────────────────

TEST(GpuBackend, DefaultBackendIsCpu) {
    PhysicsWorld world;
    EXPECT_EQ(world.backend(), PhysicsBackend::Cpu);
}

TEST(GpuBackend, SetBackendCpu) {
    PhysicsWorld world;
    world.setBackend(PhysicsBackend::Cpu);
    EXPECT_EQ(world.backend(), PhysicsBackend::Cpu);
}

TEST(GpuBackend, SetGpuBodyThresholdDoesNotCrash) {
    PhysicsWorld world;
    world.setGpuBodyThreshold(500);
    SUCCEED();
}

TEST(GpuBackend, SetBackendGpuYieldsGpuOrCpu) {
    // GPU init may fail (no GPU build or no device); must stay in a valid state.
    PhysicsWorld world;
    world.setBackend(PhysicsBackend::Gpu);
    auto b = world.backend();
    EXPECT_TRUE(b == PhysicsBackend::Gpu || b == PhysicsBackend::Cpu);
}

TEST(GpuBackend, SetBackendAutoYieldsAutoOrCpu) {
    PhysicsWorld world;
    world.setBackend(PhysicsBackend::Auto);
    auto b = world.backend();
    EXPECT_TRUE(b == PhysicsBackend::Auto || b == PhysicsBackend::Cpu);
}

// ── GPU physics tests (skipped when GPU is unavailable) ───────────────────────

TEST(GpuBackend, FreeFallUnderGravity) {
    PhysicsWorld world;
    if (!enableGpu(world)) GTEST_SKIP() << "GPU backend unavailable";

    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.setFixedTimestep(1.f / 60.f);

    BodyDescriptor d;
    d.type               = BodyType::Dynamic;
    d.mass               = 1.f;
    d.shape              = std::make_shared<SphereShape>(0.5f);
    d.transform.position = V3(0.f, 10.f, 0.f);
    Body sphere = world.createBody(d);

    for (int i = 0; i < 60; ++i) world.step(1.f / 60.f);

    float y = sphere.transform().position.y();
    EXPECT_LT(y, 8.f)  << "sphere should have fallen under gravity";
    EXPECT_GT(y, -5.f) << "sphere should not have fallen indefinitely (no floor)";
}

TEST(GpuBackend, FreeFallMatchesCpuApproximately) {
    // For free fall (no contacts), XPBD and SI are equivalent — results should match.
    auto runFreeFall = [](PhysicsBackend backend) -> float {
        PhysicsWorld world;
        world.setBackend(backend);
        if (world.backend() != backend) return -9999.f;
        world.setGravity(V3(0.f, -9.81f, 0.f));
        world.setFixedTimestep(1.f / 60.f);
        BodyDescriptor d;
        d.type               = BodyType::Dynamic;
        d.mass               = 1.f;
        d.shape              = std::make_shared<SphereShape>(0.5f);
        d.transform.position = V3(0.f, 10.f, 0.f);
        Body b = world.createBody(d);
        for (int i = 0; i < 60; ++i) world.step(1.f / 60.f);
        return b.transform().position.y();
    };

    float yGpu = runFreeFall(PhysicsBackend::Gpu);
    if (yGpu < -9000.f) GTEST_SKIP() << "GPU backend unavailable";
    float yCpu = runFreeFall(PhysicsBackend::Cpu);

    EXPECT_NEAR(yCpu, yGpu, 0.5f)
        << "CPU y=" << yCpu << "  GPU y=" << yGpu;
}

TEST(GpuBackend, StaticBodyDoesNotMove) {
    PhysicsWorld world;
    if (!enableGpu(world)) GTEST_SKIP() << "GPU backend unavailable";

    world.setGravity(V3(0.f, -9.81f, 0.f));

    BodyDescriptor d;
    d.type               = BodyType::Static;
    d.shape              = std::make_shared<BoxShape>(V3(5.f, 0.5f, 5.f));
    d.transform.position = V3(0.f, 0.f, 0.f);
    Body floor = world.createBody(d);

    for (int i = 0; i < 60; ++i) world.step(1.f / 60.f);

    auto pos = floor.transform().position;
    EXPECT_NEAR(pos.x(), 0.f, 1e-4f);
    EXPECT_NEAR(pos.y(), 0.f, 1e-4f);
    EXPECT_NEAR(pos.z(), 0.f, 1e-4f);
}

TEST(GpuBackend, TwoSpheresRepelWhenOverlapping) {
    PhysicsWorld world;
    if (!enableGpu(world)) GTEST_SKIP() << "GPU backend unavailable";

    world.setGravity(V3(0.f, 0.f, 0.f));
    world.setFixedTimestep(1.f / 60.f);

    const float r = 0.5f;
    BodyDescriptor da, db;
    da.type = db.type = BodyType::Dynamic;
    da.mass = db.mass = 1.f;
    da.shape = db.shape = std::make_shared<SphereShape>(r);
    da.transform.position = V3(-0.4f, 0.f, 0.f);  // centers 0.8 apart, need 1.0
    db.transform.position = V3( 0.4f, 0.f, 0.f);
    Body a = world.createBody(da);
    Body b = world.createBody(db);

    for (int i = 0; i < 10; ++i) world.step(1.f / 60.f);

    float sep = dist3(a.transform().position, b.transform().position);
    EXPECT_GE(sep, 2.f * r * 0.9f)
        << "spheres should be pushed apart, separation=" << sep;
}

TEST(GpuBackend, SphereDropsAndIsStoppedByStaticPlane) {
    PhysicsWorld world;
    if (!enableGpu(world)) GTEST_SKIP() << "GPU backend unavailable";

    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.setFixedTimestep(1.f / 60.f);

    BodyDescriptor plane;
    plane.type               = BodyType::Static;
    plane.shape              = std::make_shared<BoxShape>(V3(10.f, 0.5f, 10.f));
    plane.transform.position = V3(0.f, -0.5f, 0.f);  // top surface at y=0
    world.createBody(plane);

    const float r = 0.5f;
    BodyDescriptor sphere;
    sphere.type               = BodyType::Dynamic;
    sphere.mass               = 1.f;
    sphere.shape              = std::make_shared<SphereShape>(r);
    sphere.transform.position = V3(0.f, 5.f, 0.f);
    Body s = world.createBody(sphere);

    for (int i = 0; i < 120; ++i) world.step(1.f / 60.f);

    float y = s.transform().position.y();
    EXPECT_GT(y, -r - 0.5f) << "sphere should not fall through plane, y=" << y;
    EXPECT_LT(y, 5.f)       << "sphere should have fallen from y=5, y=" << y;
}

TEST(GpuBackend, AutoModeRunsWithoutCrash) {
    PhysicsWorld world;
    world.setBackend(PhysicsBackend::Gpu);
    if (world.backend() != PhysicsBackend::Gpu) GTEST_SKIP() << "GPU backend unavailable";

    PhysicsWorld w2;
    w2.setBackend(PhysicsBackend::Auto);
    w2.setGpuBodyThreshold(2);
    w2.setGravity(V3(0.f, -9.81f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.mass  = 1.f;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.transform.position = V3(0.f, 5.f, 0.f);
    w2.createBody(d);
    d.transform.position = V3(2.f, 5.f, 0.f);
    w2.createBody(d);

    EXPECT_NO_THROW(w2.step(1.f / 60.f));
}
