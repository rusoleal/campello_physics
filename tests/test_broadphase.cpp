#include <gtest/gtest.h>
#include <campello_physics/broad_phase.h>

using namespace campello::physics;

static constexpr uint32_t kLayerAll  = 0xFFFFFFFF;
static constexpr uint32_t kLayerA    = 0b0001;
static constexpr uint32_t kLayerB    = 0b0010;

static AABB box(float cx, float cy, float cz, float h = 0.5f) {
    return AABB::fromCenterHalfExtents({cx, cy, cz}, {h, h, h});
}

// ── Basic insert / remove ─────────────────────────────────────────────────────

TEST(BroadPhase, NoPairsEmpty) {
    BroadPhase bp;
    bp.computePairs();
    EXPECT_TRUE(bp.currentPairs().empty());
}

TEST(BroadPhase, TwoDynamicOverlap) {
    BroadPhase bp;
    bp.insertBody(0, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.insertBody(1, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();
    ASSERT_EQ(bp.currentPairs().size(), 1u);
    auto& p = bp.currentPairs()[0];
    EXPECT_TRUE((p.bodyA == 0 && p.bodyB == 1) || (p.bodyA == 1 && p.bodyB == 0));
}

TEST(BroadPhase, TwoDynamicSeparated) {
    BroadPhase bp;
    bp.insertBody(0, box( 0,0,0), kLayerAll, kLayerAll, false);
    bp.insertBody(1, box(10,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();
    EXPECT_TRUE(bp.currentPairs().empty());
}

TEST(BroadPhase, DynamicVsStatic) {
    BroadPhase bp;
    bp.insertBody(0, box(0, 0, 0), kLayerAll, kLayerAll, false);   // dynamic
    bp.insertBody(1, box(0, 0, 0), kLayerAll, kLayerAll, true);    // static
    bp.computePairs();
    ASSERT_EQ(bp.currentPairs().size(), 1u);
}

TEST(BroadPhase, StaticVsStaticNoPair) {
    BroadPhase bp;
    bp.insertBody(0, box(0,0,0), kLayerAll, kLayerAll, true);
    bp.insertBody(1, box(0,0,0), kLayerAll, kLayerAll, true);
    bp.computePairs();
    // Static vs static never reported (no relative motion possible)
    EXPECT_TRUE(bp.currentPairs().empty());
}

TEST(BroadPhase, CanonicalOrdering) {
    BroadPhase bp;
    bp.insertBody(5, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.insertBody(2, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();
    ASSERT_EQ(bp.currentPairs().size(), 1u);
    EXPECT_LT(bp.currentPairs()[0].bodyA, bp.currentPairs()[0].bodyB);
}

// ── Layer / mask filtering ────────────────────────────────────────────────────

TEST(BroadPhase, LayerMaskFiltered) {
    BroadPhase bp;
    // Body 0 is on layer A, body 1 is on layer B but doesn't accept layer A.
    bp.insertBody(0, box(0,0,0), kLayerA, kLayerA, false);
    bp.insertBody(1, box(0,0,0), kLayerB, kLayerB, false);
    bp.computePairs();
    EXPECT_TRUE(bp.currentPairs().empty());
}

TEST(BroadPhase, LayerMaskAccepted) {
    BroadPhase bp;
    bp.insertBody(0, box(0,0,0), kLayerA, kLayerB, false);
    bp.insertBody(1, box(0,0,0), kLayerB, kLayerA, false);
    bp.computePairs();
    EXPECT_EQ(bp.currentPairs().size(), 1u);
}

// ── Pair lifecycle ────────────────────────────────────────────────────────────

TEST(BroadPhase, PairAddedOnFirstOverlap) {
    BroadPhase bp;
    bp.insertBody(0, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.insertBody(1, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();
    ASSERT_EQ(bp.addedPairs().size(), 1u);
    EXPECT_TRUE(bp.removedPairs().empty());
}

TEST(BroadPhase, PairPersistedSecondFrame) {
    BroadPhase bp;
    bp.insertBody(0, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.insertBody(1, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();  // frame 1: added
    bp.computePairs();  // frame 2: persisted (not in added or removed)
    EXPECT_TRUE(bp.addedPairs().empty());
    EXPECT_TRUE(bp.removedPairs().empty());
    EXPECT_EQ(bp.currentPairs().size(), 1u);
}

TEST(BroadPhase, PairRemovedAfterSeparation) {
    BroadPhase bp;
    bp.insertBody(0, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.insertBody(1, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();

    bp.updateBody(1, box(100,0,0));
    bp.computePairs();

    EXPECT_TRUE(bp.currentPairs().empty());
    ASSERT_EQ(bp.removedPairs().size(), 1u);
}

// ── Many bodies ───────────────────────────────────────────────────────────────

TEST(BroadPhase, ManyBodiesNoPairs) {
    BroadPhase bp;
    for (uint32_t i = 0; i < 50; ++i)
        bp.insertBody(i, box(static_cast<float>(i) * 10.f, 0, 0), kLayerAll, kLayerAll, false);
    bp.computePairs();
    EXPECT_TRUE(bp.currentPairs().empty());
}

TEST(BroadPhase, ManyBodiesAllOverlap) {
    BroadPhase bp;
    for (uint32_t i = 0; i < 10; ++i)
        bp.insertBody(i, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();
    // 10 choose 2 = 45 pairs
    EXPECT_EQ(bp.currentPairs().size(), 45u);
}

TEST(BroadPhase, RemoveBodyClearsPairs) {
    BroadPhase bp;
    bp.insertBody(0, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.insertBody(1, box(0,0,0), kLayerAll, kLayerAll, false);
    bp.computePairs();
    ASSERT_EQ(bp.currentPairs().size(), 1u);

    bp.removeBody(1);
    bp.computePairs();
    EXPECT_TRUE(bp.currentPairs().empty());
    EXPECT_EQ(bp.removedPairs().size(), 1u);
}
