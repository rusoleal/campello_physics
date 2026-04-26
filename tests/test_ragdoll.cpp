#include <campello_physics/physics_world.h>
#include <campello_physics/ragdoll.h>
#include <gtest/gtest.h>
#include <cmath>

using namespace campello::physics;

static void runSteps(PhysicsWorld& w, int n) {
    for (int i = 0; i < n; ++i)
        w.step(1.f / 60.f);
}

// ── Ragdoll creates the expected number of bones ──────────────────────────────

TEST(Ragdoll, BoneCountMatchesTemplate) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    RagdollDescriptor desc;
    Transform t = Transform::identity();
    t.position  = { 0.f, 5.f, 0.f };
    desc.rootTransform = t;

    RagdollBody rb = world.createRagdoll(desc);
    ASSERT_TRUE(rb.isValid());
    EXPECT_EQ(rb.articulation().linkCount(), RagdollBone::Count);
}

// ── All bones are valid bodies ────────────────────────────────────────────────

TEST(Ragdoll, AllBonesAreValidBodies) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    RagdollDescriptor desc;
    RagdollBody rb = world.createRagdoll(desc);

    for (int i = 0; i < RagdollBone::Count; ++i) {
        EXPECT_TRUE(rb.bone(i).isValid()) << "Bone " << i << " is not valid";
    }
}

// ── Ragdoll falls under gravity ───────────────────────────────────────────────

TEST(Ragdoll, FallsUnderGravity) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    RagdollDescriptor desc;
    Transform t = Transform::identity();
    t.position  = { 0.f, 10.f, 0.f };
    desc.rootTransform = t;

    RagdollBody rb = world.createRagdoll(desc);

    const float pelvisStartY = rb.bone(RagdollBone::Pelvis).transform().position.y();
    runSteps(world, 60);
    const float pelvisEndY   = rb.bone(RagdollBone::Pelvis).transform().position.y();

    EXPECT_LT(pelvisEndY, pelvisStartY) << "Ragdoll pelvis should fall under gravity";
}

// ── No bone explodes (position stays bounded) ────────────────────────────────

TEST(Ragdoll, NoBoneExplodes) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });
    world.setFixedTimestep(1.f / 60.f);

    RagdollDescriptor desc;
    Transform t = Transform::identity();
    t.position  = { 0.f, 5.f, 0.f };
    desc.rootTransform = t;

    RagdollBody rb = world.createRagdoll(desc);

    runSteps(world, 120);

    for (int i = 0; i < RagdollBone::Count; ++i) {
        const auto& p = rb.bone(i).transform().position;
        const float dist = std::sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
        EXPECT_LT(dist, 200.f) << "Bone " << i << " exploded to " << dist;
    }
}

// ── Destroy cleans up all 15 bodies ──────────────────────────────────────────

TEST(Ragdoll, DestroyRemovesAllBones) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    const uint32_t before = world.bodyPool().activeCount();

    RagdollDescriptor desc;
    RagdollBody rb = world.createRagdoll(desc);

    EXPECT_EQ(world.bodyPool().activeCount(), before + RagdollBone::Count);

    world.destroyRagdoll(rb);
    EXPECT_EQ(world.bodyPool().activeCount(), before);
}

// ── Named bone accessors work ─────────────────────────────────────────────────

TEST(Ragdoll, NamedBoneAccessors) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    RagdollDescriptor desc;
    Transform t = Transform::identity();
    t.position  = { 0.f, 2.f, 0.f };
    desc.rootTransform = t;

    RagdollBody rb = world.createRagdoll(desc);

    // Pelvis is the root — should be near the spawn Y
    const float pelvisY = rb.bone(RagdollBone::Pelvis).transform().position.y();
    EXPECT_NEAR(pelvisY, 2.f, 0.5f) << "Pelvis should be near spawn position";

    // Head should be above pelvis
    const float headY = rb.bone(RagdollBone::Head).transform().position.y();
    EXPECT_GT(headY, pelvisY) << "Head should be above pelvis";

    // Legs should be below pelvis
    const float legY = rb.bone(RagdollBone::UpperLegL).transform().position.y();
    EXPECT_LT(legY, pelvisY) << "Upper leg should be below pelvis";
}
