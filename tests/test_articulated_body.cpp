#include <campello_physics/physics_world.h>
#include <campello_physics/articulated_body.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <gtest/gtest.h>
#include <cmath>

using namespace campello::physics;

static void runSteps(PhysicsWorld& w, int n) {
    for (int i = 0; i < n; ++i)
        w.step(1.f / 60.f);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

static BodyDescriptor makeDynamic(float mass, float y,
                                   std::shared_ptr<Shape> shape = nullptr) {
    BodyDescriptor bd;
    bd.type           = BodyType::Dynamic;
    bd.mass           = mass;
    bd.linearDamping  = 0.01f;
    bd.angularDamping = 0.05f;
    Transform t       = Transform::identity();
    t.position        = vm::Vector3<float>(0.f, y, 0.f);
    bd.transform      = t;
    bd.shape          = shape ? shape : std::make_shared<SphereShape>(0.2f);
    return bd;
}

// ── Single-link: a root-only articulation is just a normal body ───────────────

TEST(ArticulatedBody, SingleLinkRoot) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    ArticulatedBodyDescriptor desc;
    ArticulatedLinkDesc root;
    root.parentIndex = -1;
    root.body        = makeDynamic(1.f, 5.f);
    desc.links.push_back(root);

    ArticulatedBody ab = world.createArticulatedBody(desc);
    EXPECT_TRUE(ab.isValid());
    EXPECT_EQ(ab.linkCount(), 1);
    EXPECT_TRUE(ab.linkBody(0).isValid());
    EXPECT_EQ(ab.constraint(0), nullptr);  // root has no joint

    runSteps(world, 60);
    // Root body falls under gravity — should be below start
    EXPECT_LT(ab.linkBody(0).transform().position.y(), 5.f);
}

// ── Two-link chain: child hangs below root via BallSocket ─────────────────────

TEST(ArticulatedBody, TwoLinkBallSocketChain) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    ArticulatedBodyDescriptor desc;

    // Root: kinematic so it stays in place
    ArticulatedLinkDesc root;
    root.parentIndex = -1;
    root.body        = []() {
        BodyDescriptor bd;
        bd.type      = BodyType::Kinematic;
        bd.mass      = 1.f;
        bd.transform = Transform::identity();
        bd.shape     = std::make_shared<SphereShape>(0.2f);
        return bd;
    }();
    desc.links.push_back(root);

    // Child: hangs 1 m below root via ball-socket
    ArticulatedLinkDesc child;
    child.parentIndex    = 0;
    child.body           = makeDynamic(1.f, -1.f);
    child.jointType      = ArticulatedJointType::BallSocket;
    child.anchorInParent = { 0.f, -0.5f, 0.f };  // bottom of root
    child.anchorInSelf   = { 0.f,  0.5f, 0.f };  // top of child
    desc.links.push_back(child);

    ArticulatedBody ab = world.createArticulatedBody(desc);
    EXPECT_EQ(ab.linkCount(), 2);
    EXPECT_TRUE(ab.ballSocketConstraint(1) != nullptr);
    EXPECT_EQ(ab.constraint(0), nullptr);  // root has no joint

    const float startX = ab.linkBody(1).transform().position.x();
    const float startY = ab.linkBody(1).transform().position.y();

    runSteps(world, 120);

    // Child should have swung (gravity pulls it) but not exploded
    const float endY = ab.linkBody(1).transform().position.y();
    EXPECT_GT(endY, startY - 5.f)   << "Child should not fall freely (constrained)";
    EXPECT_LT(std::abs(ab.linkBody(1).transform().position.x() - startX), 2.f)
        << "Child should not drift far horizontally";
}

// ── Hinge chain: two links with hinge joint ───────────────────────────────────

TEST(ArticulatedBody, TwoLinkHingeChain) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    ArticulatedBodyDescriptor desc;

    // Root: static (anchor in space)
    ArticulatedLinkDesc root;
    root.parentIndex = -1;
    root.body        = []() {
        BodyDescriptor bd;
        bd.type      = BodyType::Static;
        bd.mass      = 0.f;
        bd.transform = Transform::identity();
        bd.shape     = std::make_shared<SphereShape>(0.1f);
        return bd;
    }();
    desc.links.push_back(root);

    // Child: hangs from root via hinge along Z axis
    ArticulatedLinkDesc child;
    child.parentIndex        = 0;
    child.body               = makeDynamic(1.f, -1.f);
    child.jointType          = ArticulatedJointType::Hinge;
    child.anchorInParent     = { 0.f,  0.f, 0.f };
    child.anchorInSelf       = { 0.f,  0.5f, 0.f };
    child.hingeAxisInParent  = { 0.f,  0.f, 1.f };
    child.hingeAxisInSelf    = { 0.f,  0.f, 1.f };
    child.hingeLimitsActive  = false;
    desc.links.push_back(child);

    ArticulatedBody ab = world.createArticulatedBody(desc);
    EXPECT_EQ(ab.linkCount(), 2);
    EXPECT_TRUE(ab.hingeConstraint(1) != nullptr);

    runSteps(world, 120);

    // Child should remain near its pivot (constraint holds)
    const float anchorX = ab.linkBody(0).transform().position.x();
    const float childX  = ab.linkBody(1).transform().position.x();
    EXPECT_LT(std::abs(childX - anchorX), 3.f) << "Child should stay near hinge pivot";
}

// ── Three-link chain: root → upper → lower (shoulder + elbow) ────────────────

TEST(ArticulatedBody, ThreeLinkChain) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    ArticulatedBodyDescriptor desc;

    // Root: kinematic (shoulder socket)
    ArticulatedLinkDesc root;
    root.parentIndex = -1;
    root.body        = []() {
        BodyDescriptor bd;
        bd.type      = BodyType::Kinematic;
        bd.mass      = 1.f;
        bd.transform = Transform::identity();
        bd.shape     = std::make_shared<SphereShape>(0.15f);
        return bd;
    }();
    desc.links.push_back(root);

    // Upper arm: dynamic, attached to root by ball-socket
    ArticulatedLinkDesc upper;
    upper.parentIndex    = 0;
    upper.body           = makeDynamic(0.5f, -0.5f);
    upper.jointType      = ArticulatedJointType::BallSocket;
    upper.anchorInParent = { 0.f, 0.f, 0.f };
    upper.anchorInSelf   = { 0.f, 0.5f, 0.f };
    desc.links.push_back(upper);

    // Lower arm: dynamic, attached to upper arm by hinge
    ArticulatedLinkDesc lower;
    lower.parentIndex       = 1;
    lower.body              = makeDynamic(0.3f, -1.5f);
    lower.jointType         = ArticulatedJointType::Hinge;
    lower.anchorInParent    = { 0.f, -0.5f, 0.f };
    lower.anchorInSelf      = { 0.f,  0.5f, 0.f };
    lower.hingeAxisInParent = { 0.f,  0.f,  1.f };
    lower.hingeAxisInSelf   = { 0.f,  0.f,  1.f };
    desc.links.push_back(lower);

    ArticulatedBody ab = world.createArticulatedBody(desc);
    EXPECT_EQ(ab.linkCount(), 3);
    EXPECT_TRUE(ab.ballSocketConstraint(1) != nullptr);
    EXPECT_TRUE(ab.hingeConstraint(2) != nullptr);
    EXPECT_EQ(ab.constraint(0), nullptr);

    runSteps(world, 180);

    // All links should remain within a reasonable range — not exploded
    for (int i = 0; i < 3; ++i) {
        const auto& pos = ab.linkBody(i).transform().position;
        EXPECT_LT(std::abs(pos.x()), 20.f) << "Link " << i << " x out of range";
        EXPECT_GT(pos.y(), -20.f)           << "Link " << i << " y out of range";
    }
}

// ── Destroy: bodies and constraints are cleaned up ────────────────────────────

TEST(ArticulatedBody, DestroyRemovesAllBodiesAndConstraints) {
    PhysicsWorld world;
    world.setGravity({ 0.f, -9.81f, 0.f });

    const uint32_t before = world.bodyPool().activeCount();

    ArticulatedBodyDescriptor desc;
    ArticulatedLinkDesc root;
    root.parentIndex = -1;
    root.body        = makeDynamic(1.f, 0.f);
    desc.links.push_back(root);
    ArticulatedLinkDesc child;
    child.parentIndex    = 0;
    child.body           = makeDynamic(1.f, -1.f);
    child.jointType      = ArticulatedJointType::BallSocket;
    child.anchorInParent = { 0.f, -0.3f, 0.f };
    child.anchorInSelf   = { 0.f,  0.3f, 0.f };
    desc.links.push_back(child);

    ArticulatedBody ab = world.createArticulatedBody(desc);
    EXPECT_EQ(world.bodyPool().activeCount(), before + 2);

    world.destroyArticulatedBody(ab);
    EXPECT_EQ(world.bodyPool().activeCount(), before);
}

// ── Invalid handle access is safe ────────────────────────────────────────────

TEST(ArticulatedBody, InvalidHandleIsSafe) {
    ArticulatedBody ab;
    EXPECT_FALSE(ab.isValid());
    EXPECT_EQ(ab.linkCount(), 0);
    EXPECT_FALSE(ab.linkBody(0).isValid());
    EXPECT_EQ(ab.constraint(0), nullptr);
    EXPECT_EQ(ab.hingeConstraint(1), nullptr);
}
