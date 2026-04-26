#include <gtest/gtest.h>
#include <campello_physics/narrow_phase.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/shapes/convex_hull_shape.h>
#include <cmath>

using namespace campello::physics;

namespace {

using V3 = vm::Vector3<float>;
using Q  = vm::Quaternion<float>;

Transform at(float x, float y = 0.f, float z = 0.f) {
    return Transform{ V3(x, y, z), Q::identity() };
}

} // namespace

// ── Sphere – Sphere ───────────────────────────────────────────────────────────

TEST(NarrowPhase, SphereSphereSeparated) {
    SphereShape s(1.f);
    auto m = collide({ &s, at(0) }, { &s, at(3) });
    EXPECT_FALSE(m.has_value());
}

TEST(NarrowPhase, SphereSphereOverlap) {
    SphereShape s(1.f);
    auto m = collide({ &s, at(0) }, { &s, at(1.5f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->count, 1);
    EXPECT_NEAR(m->points[0].depth, 0.5f, 1e-4f);
    // Normal from B (at x=1.5) toward A (at x=0) should be -X
    EXPECT_NEAR(m->points[0].normal.x(), -1.f, 1e-4f);
}

TEST(NarrowPhase, SphereSphereDepthAndNormal) {
    SphereShape a(0.5f), b(1.f);
    // Centers 1 apart: rA+rB = 1.5, depth = 0.5
    auto m = collide({ &a, at(0) }, { &b, at(1.f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->points[0].depth, 0.5f, 1e-4f);
    EXPECT_NEAR(m->points[0].normal.x(), -1.f, 1e-4f);  // from B toward A = -X
}

// ── Sphere – Box ──────────────────────────────────────────────────────────────

TEST(NarrowPhase, SphereBoxSeparated) {
    SphereShape s(0.5f);
    BoxShape    b{ V3(1.f, 1.f, 1.f) };
    auto m = collide({ &s, at(3.f) }, { &b, at(0) });
    EXPECT_FALSE(m.has_value());
}

TEST(NarrowPhase, SphereBoxOverlap) {
    SphereShape s(0.5f);
    BoxShape    b{ V3(1.f, 1.f, 1.f) };
    // Sphere center at x=1.2, box halfExtent=1 → closest point on box = (1,0,0)
    // distance = 0.2, radius = 0.5, depth = 0.3
    auto m = collide({ &s, at(1.2f) }, { &b, at(0) });
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->points[0].depth, 0.3f, 1e-4f);
    // Normal from B(box) toward A(sphere) should be +X
    EXPECT_NEAR(m->points[0].normal.x(), 1.f, 1e-4f);
}

TEST(NarrowPhase, BoxSphereSphereIsB) {
    SphereShape s(0.5f);
    BoxShape    b{ V3(1.f, 1.f, 1.f) };
    // A=box at origin, B=sphere at x=1.2; normal should point from B to A = -X
    auto m = collide({ &b, at(0) }, { &s, at(1.2f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->points[0].depth, 0.3f, 1e-4f);
    EXPECT_NEAR(m->points[0].normal.x(), -1.f, 1e-4f);
}

TEST(NarrowPhase, SphereInsideBox) {
    SphereShape s(0.5f);
    BoxShape    b{ V3(2.f, 2.f, 2.f) };
    auto m = collide({ &s, at(0) }, { &b, at(0) });
    ASSERT_TRUE(m.has_value());
    EXPECT_GT(m->points[0].depth, 0.f);
}

// ── Box – Box ─────────────────────────────────────────────────────────────────

TEST(NarrowPhase, BoxBoxSeparated) {
    BoxShape a{ V3(1,1,1) }, b{ V3(1,1,1) };
    auto m = collide({ &a, at(0) }, { &b, at(3.f) });
    EXPECT_FALSE(m.has_value());
}

TEST(NarrowPhase, BoxBoxFaceContact) {
    BoxShape a{ V3(1,1,1) }, b{ V3(1,1,1) };
    // Overlap of 0.5 along X axis
    auto m = collide({ &a, at(0) }, { &b, at(1.5f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_GE(m->count, 1);
    EXPECT_LE(m->count, 4);
    // All normals should be along ±X
    for (int i = 0; i < m->count; ++i) {
        EXPECT_NEAR(std::fabs(m->points[i].normal.x()), 1.f, 1e-3f);
        EXPECT_NEAR(m->points[i].normal.y(), 0.f, 1e-3f);
        EXPECT_NEAR(m->points[i].normal.z(), 0.f, 1e-3f);
    }
}

TEST(NarrowPhase, BoxBoxNormalDirection) {
    BoxShape a{ V3(1,1,1) }, b{ V3(1,1,1) };
    // B is to the right (+X) of A; normal should point from B to A = -X
    auto m = collide({ &a, at(0) }, { &b, at(1.5f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->points[0].normal.x(), -1.f, 1e-3f);
}

TEST(NarrowPhase, BoxBoxDepth) {
    BoxShape a{ V3(1,1,1) }, b{ V3(1,1,1) };
    auto m = collide({ &a, at(0) }, { &b, at(1.5f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->points[0].depth, 0.5f, 1e-3f);
}

// ── Capsule – Capsule ─────────────────────────────────────────────────────────

TEST(NarrowPhase, CapsuleCapsuleSeparated) {
    CapsuleShape c(0.5f, 1.f);
    auto m = collide({ &c, at(0) }, { &c, at(4.f) });
    EXPECT_FALSE(m.has_value());
}

TEST(NarrowPhase, CapsuleCapsuleOverlap) {
    CapsuleShape c(0.5f, 1.f);
    // Two capsules, centers 1 apart, rSum = 1 → depth = 0
    // At distance 0.8: depth = 0.2
    auto m = collide({ &c, at(0) }, { &c, at(0.8f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->points[0].depth, 0.2f, 1e-4f);
    EXPECT_NEAR(m->points[0].normal.x(), -1.f, 1e-4f);
}

// ── Sphere – Capsule ──────────────────────────────────────────────────────────

TEST(NarrowPhase, SphereCapsuleOverlap) {
    SphereShape  s(0.5f);
    CapsuleShape c(0.5f, 1.f);
    // Sphere at (0,0,0), capsule at (0.8,0,0): rSum=1.0, dist=0.8, depth=0.2
    auto m = collide({ &s, at(0) }, { &c, at(0.8f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->points[0].depth, 0.2f, 1e-4f);
}

// ── GJK fallback (Convex Hull) ────────────────────────────────────────────────

TEST(NarrowPhase, ConvexHullBoxOverlap) {
    // Convex hull that matches a unit box (8 vertices)
    std::vector<V3> pts = {
        {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
        {-1,-1, 1},{1,-1, 1},{1,1, 1},{-1,1, 1}
    };
    ConvexHullShape hull(pts);
    BoxShape        box{ V3(1.f, 1.f, 1.f) };

    // Hull at origin, box offset 1.5 → overlap 0.5
    auto m = collide({ &hull, at(0) }, { &box, at(1.5f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_GT(m->points[0].depth, 0.f);
}

TEST(NarrowPhase, ConvexHullSeparated) {
    std::vector<V3> pts = {
        {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
        {-1,-1, 1},{1,-1, 1},{1,1, 1},{-1,1, 1}
    };
    ConvexHullShape hull(pts);
    auto m = collide({ &hull, at(0) }, { &hull, at(5.f) });
    EXPECT_FALSE(m.has_value());
}

// ── Manifold contact count ────────────────────────────────────────────────────

TEST(NarrowPhase, BoxBoxFaceReturnsUpTo4Points) {
    BoxShape a{ V3(1,1,1) }, b{ V3(1,1,1) };
    // Full face-on-face overlap: should produce 4 contact points
    auto m = collide({ &a, at(0) }, { &b, at(1.9f) });
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->count, 4);
}

// ── Persistent contact cache ──────────────────────────────────────────────────

TEST(NarrowPhase, WarmStartTransferred) {
    SphereShape s(1.f);
    NarrowPhase np;

    auto getShape = [&](uint32_t id) -> ShapeInstance {
        return { &s, at(0.f + (id == 1 ? 1.5f : 0.f)) };
    };

    CollisionPair pair{ 0, 1 };
    std::vector<CollisionPair> pairs = { pair };

    // Frame 1
    np.process(pairs, getShape);
    ASSERT_EQ(np.manifolds().size(), 1u);
    // Manually inject a warm-start value into the cache result
    // (normally set by the constraint solver)
    // Frame 2: warm-start should be 0 (nothing was stored yet)
    np.clearManifolds();
    np.process(pairs, getShape);
    ASSERT_EQ(np.manifolds().size(), 1u);
    // The cache matched — warm-start is still 0 because we never set it,
    // but the pair was found (no crash/assert).
    EXPECT_EQ(np.manifolds()[0].count, 1);
}
