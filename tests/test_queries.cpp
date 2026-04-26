#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/query.h>
#include <cmath>

using namespace campello::physics;

namespace {
using V3 = vm::Vector3<float>;
} // namespace

// ── Raycast ───────────────────────────────────────────────────────────────────

TEST(Queries, RaycastMissEmptyWorld) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    Ray ray;
    ray.origin      = V3(0.f, 0.f, 0.f);
    ray.direction   = V3(0.f, 1.f, 0.f);
    ray.maxDistance = 100.f;

    EXPECT_FALSE(world.raycastClosest(ray).has_value());
}

TEST(Queries, RaycastHitSphere) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(0.f, 5.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    Ray ray;
    ray.origin      = V3(0.f, 0.f, 0.f);
    ray.direction   = V3(0.f, 1.f, 0.f);
    ray.maxDistance = 100.f;

    auto hit = world.raycastClosest(ray);
    ASSERT_TRUE(hit.has_value()) << "Expected hit on sphere at y=5";

    // Entry point: sphere bottom at y = 4.0 (center 5, radius 1)
    EXPECT_NEAR(hit->fraction, 4.f, 0.01f) << "fraction = " << hit->fraction;
    EXPECT_NEAR(hit->point.y(), 4.f, 0.02f) << "point.y = " << hit->point.y();
    // Normal points away from sphere center toward ray origin → negative Y
    EXPECT_LT(hit->normal.y(), -0.9f) << "normal.y = " << hit->normal.y();
}

TEST(Queries, RaycastMissSphere) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(5.f, 0.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    Ray ray;
    ray.origin      = V3(0.f, 0.f, 0.f);
    ray.direction   = V3(0.f, 0.f, 1.f);  // along Z, misses sphere at x=5
    ray.maxDistance = 100.f;

    EXPECT_FALSE(world.raycastClosest(ray).has_value());
}

TEST(Queries, RaycastClosestOfTwo) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(0.5f);

    d.transform.position = V3(0.f, 3.f, 0.f);
    auto near = world.createBody(d);

    d.transform.position = V3(0.f, 8.f, 0.f);
    [[maybe_unused]] auto far = world.createBody(d);

    Ray ray;
    ray.origin      = V3(0.f, 0.f, 0.f);
    ray.direction   = V3(0.f, 1.f, 0.f);
    ray.maxDistance = 100.f;

    auto hit = world.raycastClosest(ray);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->fraction, 2.5f, 0.05f);  // near sphere surface at y=2.5
    EXPECT_EQ(hit->body.id(), near.id()) << "Should hit the nearer body";
}

TEST(Queries, RaycastAll) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(0.5f);

    d.transform.position = V3(0.f, 3.f, 0.f);
    [[maybe_unused]] auto b1 = world.createBody(d);
    d.transform.position = V3(0.f, 8.f, 0.f);
    [[maybe_unused]] auto b2 = world.createBody(d);

    Ray ray;
    ray.origin      = V3(0.f, 0.f, 0.f);
    ray.direction   = V3(0.f, 1.f, 0.f);
    ray.maxDistance = 100.f;

    auto hits = world.raycastAll(ray);
    ASSERT_EQ(hits.size(), 2u) << "Expected 2 hits";
    EXPECT_LT(hits[0].fraction, hits[1].fraction) << "Should be sorted by fraction";
}

TEST(Queries, RaycastMaxDistanceCulls) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(0.f, 20.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    Ray ray;
    ray.origin      = V3(0.f, 0.f, 0.f);
    ray.direction   = V3(0.f, 1.f, 0.f);
    ray.maxDistance = 10.f;  // sphere surface at y=19, beyond range

    EXPECT_FALSE(world.raycastClosest(ray).has_value());
}

TEST(Queries, RaycastHitBox) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(V3(1.f, 1.f, 1.f));
    d.transform.position = V3(0.f, 5.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    Ray ray;
    ray.origin      = V3(0.f, 0.f, 0.f);
    ray.direction   = V3(0.f, 1.f, 0.f);
    ray.maxDistance = 100.f;

    auto hit = world.raycastClosest(ray);
    ASSERT_TRUE(hit.has_value()) << "Expected hit on box at y=5";
    EXPECT_NEAR(hit->fraction, 4.f, 0.01f);  // bottom face at y=4
    EXPECT_LT(hit->normal.y(), -0.9f);
}

// ── Overlap ───────────────────────────────────────────────────────────────────

TEST(Queries, OverlapFindsBody) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(1.5f, 0.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    auto querySphere = SphereShape(2.f);
    Transform qt;
    qt.position = V3(0.f, 0.f, 0.f);
    auto results = world.overlap(querySphere, qt);
    EXPECT_EQ(results.size(), 1u) << "Expected 1 overlap";
}

TEST(Queries, OverlapMissesDistantBody) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(10.f, 0.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    auto querySphere = SphereShape(1.f);
    Transform qt;
    qt.position = V3(0.f, 0.f, 0.f);
    auto results = world.overlap(querySphere, qt);
    EXPECT_EQ(results.size(), 0u);
}

TEST(Queries, OverlapMultipleBodies) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(0.5f);

    d.transform.position = V3(0.5f, 0.f, 0.f);
    [[maybe_unused]] auto b1 = world.createBody(d);
    d.transform.position = V3(-0.5f, 0.f, 0.f);
    [[maybe_unused]] auto b2 = world.createBody(d);
    d.transform.position = V3(5.f, 0.f, 0.f);
    [[maybe_unused]] auto b3 = world.createBody(d);

    auto querySphere = SphereShape(1.5f);
    Transform qt;
    qt.position = V3(0.f, 0.f, 0.f);
    auto results = world.overlap(querySphere, qt);
    EXPECT_EQ(results.size(), 2u) << "Expected 2 overlapping bodies";
}

// ── Shape cast ────────────────────────────────────────────────────────────────

TEST(Queries, ShapeCastHitsBody) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(0.f, 5.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    auto castShape = SphereShape(0.5f);
    Transform startT;
    startT.position = V3(0.f, 0.f, 0.f);
    V3 dir(0.f, 1.f, 0.f);

    auto hit = world.shapeCast(castShape, startT, dir, 10.f);
    ASSERT_TRUE(hit.has_value()) << "Shape cast should hit sphere at y=5";
    // First contact: cast sphere (r=0.5) touches target (r=1), combined dist = 1.5 below center
    // Contact at cast center y = 5 - 1.5 = 3.5
    EXPECT_NEAR(hit->fraction, 3.5f, 0.1f) << "fraction = " << hit->fraction;
}

TEST(Queries, ShapeCastMisses) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(5.f, 0.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    auto castShape = SphereShape(0.5f);
    Transform startT;
    startT.position = V3(0.f, 0.f, 0.f);
    V3 dir(0.f, 1.f, 0.f);  // straight up, misses sphere at x=5

    EXPECT_FALSE(world.shapeCast(castShape, startT, dir, 10.f).has_value());
}

TEST(Queries, ShapeCastMaxDistance) {
    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.transform.position = V3(0.f, 20.f, 0.f);
    [[maybe_unused]] auto body = world.createBody(d);

    auto castShape = SphereShape(0.5f);
    Transform startT;
    startT.position = V3(0.f, 0.f, 0.f);
    V3 dir(0.f, 1.f, 0.f);

    EXPECT_FALSE(world.shapeCast(castShape, startT, dir, 5.f).has_value());
}
