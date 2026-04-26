#include <gtest/gtest.h>
#include <campello_physics/shapes.h>
#include <cmath>

using namespace campello::physics;

static constexpr float kEps = 1e-4f;
static Transform Identity = Transform::identity();

// ── BoxShape ──────────────────────────────────────────────────────────────────

TEST(BoxShape, AABBIdentityTransform) {
    BoxShape box({1.f, 2.f, 3.f});
    auto aabb = box.computeAABB(Identity);
    EXPECT_NEAR(aabb.min.x(), -1.f, kEps);
    EXPECT_NEAR(aabb.max.y(),  2.f, kEps);
    EXPECT_NEAR(aabb.max.z(),  3.f, kEps);
}

TEST(BoxShape, AABBTranslated) {
    BoxShape box({1.f, 1.f, 1.f});
    Transform t{{5.f, 0.f, 0.f}, vm::Quaternion<float>::identity()};
    auto aabb = box.computeAABB(t);
    EXPECT_NEAR(aabb.min.x(), 4.f, kEps);
    EXPECT_NEAR(aabb.max.x(), 6.f, kEps);
}

TEST(BoxShape, AABBRotated90DegreesY) {
    BoxShape box({2.f, 1.f, 1.f});
    auto q = vm::Quaternion<float>::axisAngle({0.f, 1.f, 0.f}, std::numbers::pi_v<float> / 2.f);
    Transform t{{0.f, 0.f, 0.f}, q};
    auto aabb = box.computeAABB(t);
    // After 90° Y rotation, x-extent (2) maps to z — AABB x should be ~1, z should be ~2
    EXPECT_NEAR(aabb.max.x(), 1.f, kEps);
    EXPECT_NEAR(aabb.max.z(), 2.f, kEps);
}

TEST(BoxShape, Inertia) {
    BoxShape box({1.f, 1.f, 1.f});  // unit cube
    auto I = box.computeLocalInertiaDiagonal(12.f);
    // (12/3)*(1+1) = 8 for each axis
    EXPECT_NEAR(I.x(), 8.f, kEps);
    EXPECT_NEAR(I.y(), 8.f, kEps);
    EXPECT_NEAR(I.z(), 8.f, kEps);
}

// ── SphereShape ───────────────────────────────────────────────────────────────

TEST(SphereShape, AABB) {
    SphereShape sphere(3.f);
    auto aabb = sphere.computeAABB(Identity);
    EXPECT_NEAR(aabb.min.x(), -3.f, kEps);
    EXPECT_NEAR(aabb.max.y(),  3.f, kEps);
}

TEST(SphereShape, Inertia) {
    SphereShape sphere(1.f);
    auto I = sphere.computeLocalInertiaDiagonal(1.f);
    EXPECT_NEAR(I.x(), 2.f / 5.f, kEps);
    EXPECT_NEAR(I.y(), I.x(), kEps);
    EXPECT_NEAR(I.z(), I.x(), kEps);
}

// ── CapsuleShape ──────────────────────────────────────────────────────────────

TEST(CapsuleShape, AABBIdentity) {
    CapsuleShape cap(0.5f, 1.f);  // radius=0.5, halfHeight=1
    auto aabb = cap.computeAABB(Identity);
    // Y extent: -(1 + 0.5) to +(1 + 0.5)
    EXPECT_NEAR(aabb.min.y(), -1.5f, kEps);
    EXPECT_NEAR(aabb.max.y(),  1.5f, kEps);
    // X/Z extent: ±0.5
    EXPECT_NEAR(aabb.max.x(),  0.5f, kEps);
}

TEST(CapsuleShape, InertiaSphere) {
    // A capsule with halfHeight=0 should degenerate to a sphere inertia
    CapsuleShape cap(1.f, 0.f);
    auto I = cap.computeLocalInertiaDiagonal(1.f);
    const float expected = (2.f / 5.f) * 1.f * 1.f;
    EXPECT_NEAR(I.y(), expected, kEps);
    EXPECT_NEAR(I.x(), I.z(), kEps);
}

// ── CylinderShape ─────────────────────────────────────────────────────────────

TEST(CylinderShape, AABBIdentity) {
    CylinderShape cyl(1.f, 2.f);
    auto aabb = cyl.computeAABB(Identity);
    EXPECT_NEAR(aabb.max.x(),  1.f, kEps);
    EXPECT_NEAR(aabb.max.y(),  2.f, kEps);
    EXPECT_NEAR(aabb.max.z(),  1.f, kEps);
}

TEST(CylinderShape, Inertia) {
    CylinderShape cyl(1.f, 1.f);
    auto I = cyl.computeLocalInertiaDiagonal(4.f);
    // Ixx = 4*(1/4 + 1/3) = 4*7/12 = 7/3
    EXPECT_NEAR(I.x(), 4.f * (1.f / 4.f + 1.f / 3.f), kEps);
    EXPECT_NEAR(I.x(), I.z(), kEps);
    // Iyy = 4*(1/2) = 2
    EXPECT_NEAR(I.y(), 2.f, kEps);
}

// ── ConvexHullShape ───────────────────────────────────────────────────────────

TEST(ConvexHullShape, AABBContainsAllPoints) {
    std::vector<vm::Vector3<float>> pts = {
        {-1.f, 0.f, 0.f}, {1.f, 0.f, 0.f},
        {0.f, -2.f, 0.f}, {0.f, 2.f, 0.f},
        {0.f, 0.f, -3.f}, {0.f, 0.f, 3.f}
    };
    ConvexHullShape hull(pts);
    auto aabb = hull.computeAABB(Identity);
    for (const auto& p : pts)
        EXPECT_TRUE(aabb.contains(p));
}

// ── TriangleMeshShape ─────────────────────────────────────────────────────────

TEST(TriangleMeshShape, AABBCoversAllVerts) {
    std::vector<vm::Vector3<float>> verts = {
        {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}
    };
    std::vector<std::array<uint32_t,3>> idx = {{0,1,2},{0,1,3},{0,2,3},{1,2,3}};
    TriangleMeshShape mesh(verts, idx);
    auto aabb = mesh.computeAABB(Identity);
    for (const auto& v : verts)
        EXPECT_TRUE(aabb.contains(v));
}

TEST(TriangleMeshShape, BVHBuilt) {
    std::vector<vm::Vector3<float>> verts;
    std::vector<std::array<uint32_t,3>> idx;
    // 8 triangles so the BVH splits at least once
    for (int i = 0; i < 8; ++i) {
        float x = static_cast<float>(i);
        verts.push_back({x, 0.f, 0.f});
        verts.push_back({x + 1.f, 0.f, 0.f});
        verts.push_back({x + 0.5f, 1.f, 0.f});
        uint32_t base = static_cast<uint32_t>(i * 3);
        idx.push_back({base, base + 1, base + 2});
    }
    TriangleMeshShape mesh(verts, idx);
    EXPECT_GT(mesh.bvhNodes().size(), 1u);
}

// ── HeightFieldShape ──────────────────────────────────────────────────────────

TEST(HeightFieldShape, AABBCorrect) {
    std::vector<float> heights = {0.f, 1.f, 2.f, 3.f};  // 2×2 grid
    HeightFieldShape hf(2, 2, 1.f, 1.f, 1.f, heights);
    auto aabb = hf.computeAABB(Identity);
    EXPECT_NEAR(aabb.min.y(), 0.f, kEps);
    EXPECT_NEAR(aabb.max.y(), 3.f, kEps);
    EXPECT_NEAR(aabb.max.x(), 1.f, kEps);
    EXPECT_NEAR(aabb.max.z(), 1.f, kEps);
}

TEST(HeightFieldShape, HeightAtCell) {
    std::vector<float> heights = {0.f, 1.f, 2.f, 5.f};
    HeightFieldShape hf(2, 2, 1.f, 1.f, 2.f, heights);  // heightScale=2
    EXPECT_NEAR(hf.heightAt(1, 1), 10.f, kEps);
}

// ── CompoundShape ─────────────────────────────────────────────────────────────

TEST(CompoundShape, AABBIsUnion) {
    CompoundShape compound;
    compound.addChild(std::make_shared<SphereShape>(1.f),
                      Transform{{-5.f, 0.f, 0.f}, vm::Quaternion<float>::identity()});
    compound.addChild(std::make_shared<SphereShape>(1.f),
                      Transform{{ 5.f, 0.f, 0.f}, vm::Quaternion<float>::identity()});
    auto aabb = compound.computeAABB(Identity);
    EXPECT_NEAR(aabb.min.x(), -6.f, kEps);
    EXPECT_NEAR(aabb.max.x(),  6.f, kEps);
}

TEST(CompoundShape, InertiaTwoBodies) {
    CompoundShape compound;
    compound.addChild(std::make_shared<SphereShape>(1.f),
                      Transform{{0.f, 2.f, 0.f}, vm::Quaternion<float>::identity()});
    compound.addChild(std::make_shared<SphereShape>(1.f),
                      Transform{{0.f,-2.f, 0.f}, vm::Quaternion<float>::identity()});
    auto I = compound.computeLocalInertiaDiagonal(2.f);
    // Should be > sphere inertia alone (parallel axis contribution)
    SphereShape s(1.f);
    auto Is = s.computeLocalInertiaDiagonal(1.f);
    EXPECT_GT(I.x(), Is.x());
}
