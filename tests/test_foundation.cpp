#include <gtest/gtest.h>
#include <campello_physics/aabb.h>
#include <campello_physics/ray.h>
#include <campello_physics/transform.h>
#include <campello_physics/plane.h>
#include <campello_physics/allocator.h>

using namespace campello::physics;

// ── AABB ─────────────────────────────────────────────────────────────────────

TEST(AABB, EmptyIsEmpty) {
    auto b = AABB::empty();
    EXPECT_TRUE(b.isEmpty());
}

TEST(AABB, ExpandPoint) {
    auto b = AABB::empty();
    b.expand(vm::Vector3<float>(1.f, 2.f, 3.f));
    b.expand(vm::Vector3<float>(-1.f, -2.f, -3.f));
    EXPECT_FLOAT_EQ(b.min.x(), -1.f);
    EXPECT_FLOAT_EQ(b.max.z(),  3.f);
}

TEST(AABB, IntersectsOverlapping) {
    auto a = AABB::fromCenterHalfExtents({0,0,0}, {1,1,1});
    auto b = AABB::fromCenterHalfExtents({1,0,0}, {1,1,1});
    EXPECT_TRUE(a.intersects(b));
}

TEST(AABB, NoIntersectSeparated) {
    auto a = AABB::fromCenterHalfExtents({0,0,0}, {1,1,1});
    auto b = AABB::fromCenterHalfExtents({5,0,0}, {1,1,1});
    EXPECT_FALSE(a.intersects(b));
}

TEST(AABB, ContainsPoint) {
    auto b = AABB::fromCenterHalfExtents({0,0,0}, {1,1,1});
    EXPECT_TRUE(b.contains({0.f, 0.f, 0.f}));
    EXPECT_FALSE(b.contains({2.f, 0.f, 0.f}));
}

TEST(AABB, Center) {
    auto b = AABB::fromCenterHalfExtents({3.f,0.f,0.f}, {1.f,1.f,1.f});
    EXPECT_FLOAT_EQ(b.center().x(), 3.f);
}

TEST(AABB, SurfaceArea) {
    // Unit cube: 6 * 1 = 6
    auto b = AABB::fromCenterHalfExtents({0,0,0}, {0.5f,0.5f,0.5f});
    EXPECT_FLOAT_EQ(b.surfaceArea(), 6.f);
}

TEST(AABB, Merged) {
    auto a = AABB::fromCenterHalfExtents({-2,0,0}, {1,1,1});
    auto b = AABB::fromCenterHalfExtents({ 2,0,0}, {1,1,1});
    auto m = a.merged(b);
    EXPECT_FLOAT_EQ(m.min.x(), -3.f);
    EXPECT_FLOAT_EQ(m.max.x(),  3.f);
}

// ── Ray ───────────────────────────────────────────────────────────────────────

TEST(Ray, PointAt) {
    Ray r{{0,0,0}, {0,0,1}};
    auto p = r.pointAt(5.f);
    EXPECT_FLOAT_EQ(p.z(), 5.f);
}

// ── Transform ─────────────────────────────────────────────────────────────────

TEST(Transform, IdentityLeavesPointUnchanged) {
    auto t = Transform::identity();
    vm::Vector3<float> p{1.f, 2.f, 3.f};
    auto result = t.transformPoint(p);
    EXPECT_NEAR(result.x(), 1.f, 1e-5f);
    EXPECT_NEAR(result.y(), 2.f, 1e-5f);
    EXPECT_NEAR(result.z(), 3.f, 1e-5f);
}

TEST(Transform, TranslationOnly) {
    Transform t{{1.f, 0.f, 0.f}, vm::Quaternion<float>::identity()};
    vm::Vector3<float> p{0.f, 0.f, 0.f};
    auto result = t.transformPoint(p);
    EXPECT_NEAR(result.x(), 1.f, 1e-5f);
}

TEST(Transform, InverseTransformPoint) {
    Transform t{{5.f, 0.f, 0.f}, vm::Quaternion<float>::identity()};
    vm::Vector3<float> world{5.f, 0.f, 0.f};
    auto local = t.inverseTransformPoint(world);
    EXPECT_NEAR(local.x(), 0.f, 1e-5f);
}

TEST(Transform, CombinedTranslations) {
    Transform parent{{1.f, 0.f, 0.f}, vm::Quaternion<float>::identity()};
    Transform child {{2.f, 0.f, 0.f}, vm::Quaternion<float>::identity()};
    auto combined = parent.combined(child);
    EXPECT_NEAR(combined.position.x(), 3.f, 1e-5f);
}

TEST(Transform, InversedCancels) {
    Transform t{{3.f, 1.f, -2.f}, vm::Quaternion<float>::identity()};
    auto inv = t.inversed();
    auto combined = t.combined(inv);
    EXPECT_NEAR(combined.position.x(), 0.f, 1e-4f);
    EXPECT_NEAR(combined.position.y(), 0.f, 1e-4f);
    EXPECT_NEAR(combined.position.z(), 0.f, 1e-4f);
}

// ── Plane ─────────────────────────────────────────────────────────────────────

TEST(Plane, SignedDistanceAbovePlane) {
    auto p = Plane::fromPointNormal({0,0,0}, {0,1,0});
    EXPECT_FLOAT_EQ(p.signedDistance({0.f, 5.f, 0.f}),  5.f);
    EXPECT_FLOAT_EQ(p.signedDistance({0.f,-5.f, 0.f}), -5.f);
}

TEST(Plane, FromPoints) {
    auto p = Plane::fromPoints({0,0,0}, {1,0,0}, {0,0,1});
    // Normal should be (0, ±1, 0)
    EXPECT_NEAR(std::abs(p.normal.y()), 1.f, 1e-5f);
}

TEST(Plane, IsOnPositiveSide) {
    auto p = Plane::fromPointNormal({0,0,0}, {0,1,0});
    EXPECT_TRUE (p.isOnPositiveSide({0.f, 1.f, 0.f}));
    EXPECT_FALSE(p.isOnPositiveSide({0.f,-1.f, 0.f}));
}

// ── PoolAllocator ─────────────────────────────────────────────────────────────

TEST(PoolAllocator, AllocateDeallocate) {
    PoolAllocator pool(32, 8);
    EXPECT_EQ(pool.capacity(), 8u);
    EXPECT_EQ(pool.used(),     0u);

    void* a = pool.allocate();
    EXPECT_NE(a, nullptr);
    EXPECT_EQ(pool.used(), 1u);

    pool.deallocate(a);
    EXPECT_EQ(pool.used(), 0u);
}

TEST(PoolAllocator, ExhaustPool) {
    PoolAllocator pool(16, 3);
    void* a = pool.allocate();
    void* b = pool.allocate();
    void* c = pool.allocate();
    EXPECT_NE(a, nullptr);
    EXPECT_NE(b, nullptr);
    EXPECT_NE(c, nullptr);
    EXPECT_TRUE(pool.full());

    void* d = pool.allocate();
    EXPECT_EQ(d, nullptr);

    pool.deallocate(b);
    void* e = pool.allocate();
    EXPECT_NE(e, nullptr);
}

TEST(PoolAllocator, ReuseAfterFree) {
    PoolAllocator pool(sizeof(int), 2);
    int* a = static_cast<int*>(pool.allocate());
    *a = 42;
    pool.deallocate(a);

    int* b = static_cast<int*>(pool.allocate());
    *b = 7;
    EXPECT_EQ(*b, 7);
}
