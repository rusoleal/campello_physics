#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/debug_draw.h>
#include <campello_physics/profiler.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/shapes/compound_shape.h>
#include <campello_physics/body.h>
#include <vector>
#include <string>

namespace campello::physics {

// ── Line-counting IDebugDraw ──────────────────────────────────────────────────

struct LineRecord {
    vm::Vector3<float> from, to;
    DebugColor color;
};

class RecordingDraw : public IDebugDraw {
public:
    std::vector<LineRecord> lines;

    void drawLine(const vm::Vector3<float>& from,
                  const vm::Vector3<float>& to,
                  DebugColor color) override {
        lines.push_back({from, to, color});
    }

    void reset() { lines.clear(); }
    int lineCount() const { return static_cast<int>(lines.size()); }
};

// ── DebugColor factory tests ──────────────────────────────────────────────────

TEST(DebugColor, Factories) {
    EXPECT_FLOAT_EQ(DebugColor::white().r, 1.f);
    EXPECT_FLOAT_EQ(DebugColor::red().g,   0.f);
    EXPECT_FLOAT_EQ(DebugColor::blue().b,  1.f);
    EXPECT_FLOAT_EQ(DebugColor::grey().r,  0.5f);
}

// ── DebugDrawFlags operator tests ─────────────────────────────────────────────

TEST(DebugDrawFlags, AndOr) {
    auto flags = DebugDrawFlags::BodyShapes | DebugDrawFlags::BodyAABBs;
    EXPECT_TRUE(flags & DebugDrawFlags::BodyShapes);
    EXPECT_TRUE(flags & DebugDrawFlags::BodyAABBs);
    EXPECT_FALSE(flags & DebugDrawFlags::ContactPoints);
}

// ── Default IDebugDraw implementations ───────────────────────────────────────

TEST(IDebugDraw, DrawAABBProduces12Lines) {
    RecordingDraw draw;
    draw.drawAABB({0.f,0.f,0.f}, {1.f,1.f,1.f}, DebugColor::white());
    EXPECT_EQ(draw.lineCount(), 12);
}

TEST(IDebugDraw, DrawSphereProducesThreeRings) {
    RecordingDraw draw;
    draw.drawSphere({0.f,0.f,0.f}, 1.f, DebugColor::green(), 16);
    // 3 great-circle rings × 16 segments each
    EXPECT_EQ(draw.lineCount(), 3 * 16);
}

TEST(IDebugDraw, DrawBoxProduces12Lines) {
    RecordingDraw draw;
    vm::Quaternion<float> identity;
    draw.drawBox({0.f,0.f,0.f}, {1.f,1.f,1.f}, identity, DebugColor::cyan());
    EXPECT_EQ(draw.lineCount(), 12);
}

TEST(IDebugDraw, DrawCapsuleProducesLines) {
    RecordingDraw draw;
    vm::Quaternion<float> identity;
    draw.drawCapsule({0.f,0.f,0.f}, identity, 0.5f, 1.f, DebugColor::yellow(), 8);
    EXPECT_GT(draw.lineCount(), 0);
}

TEST(IDebugDraw, DrawArrowProducesLines) {
    RecordingDraw draw;
    draw.drawArrow({0.f,0.f,0.f}, {0.f,1.f,0.f}, 1.f, DebugColor::red());
    EXPECT_GE(draw.lineCount(), 1);
}

TEST(IDebugDraw, DrawArrowZeroLengthIsNoOp) {
    RecordingDraw draw;
    draw.drawArrow({0.f,0.f,0.f}, {0.f,0.f,0.f}, 1.f, DebugColor::red());
    EXPECT_EQ(draw.lineCount(), 0);
}

TEST(IDebugDraw, DrawPointProduces3Lines) {
    RecordingDraw draw;
    draw.drawPoint({0.f,0.f,0.f}, 0.05f, DebugColor::magenta());
    EXPECT_EQ(draw.lineCount(), 3);
}

// ── PhysicsWorld::debugDraw integration ───────────────────────────────────────

TEST(PhysicsWorldDebugDraw, EmptyWorldNoLines) {
    PhysicsWorld world;
    RecordingDraw draw;
    world.debugDraw(draw, DebugDrawFlags::All);
    EXPECT_EQ(draw.lineCount(), 0);
}

TEST(PhysicsWorldDebugDraw, SphereBodyDrawsShapeLines) {
    PhysicsWorld world;
    BodyDescriptor desc;
    desc.type  = BodyType::Dynamic;
    desc.shape = std::make_shared<SphereShape>(0.5f);
    desc.mass  = 1.f;
    world.createBody(desc);

    RecordingDraw draw;
    world.debugDraw(draw, DebugDrawFlags::BodyShapes);
    EXPECT_GT(draw.lineCount(), 0);
}

TEST(PhysicsWorldDebugDraw, AABBFlagDrawsMore) {
    PhysicsWorld world;
    BodyDescriptor desc;
    desc.type  = BodyType::Static;
    desc.shape = std::make_shared<BoxShape>(vm::Vector3<float>(1.f, 1.f, 1.f));
    world.createBody(desc);

    RecordingDraw drawShapes, drawBoth;
    world.debugDraw(drawShapes, DebugDrawFlags::BodyShapes);
    world.debugDraw(drawBoth,
        DebugDrawFlags::BodyShapes | DebugDrawFlags::BodyAABBs);
    EXPECT_GT(drawBoth.lineCount(), drawShapes.lineCount());
}

TEST(PhysicsWorldDebugDraw, NoneDrawsNothing) {
    PhysicsWorld world;
    BodyDescriptor desc;
    desc.type  = BodyType::Dynamic;
    desc.shape = std::make_shared<SphereShape>(1.f);
    desc.mass  = 1.f;
    world.createBody(desc);

    RecordingDraw draw;
    world.debugDraw(draw, DebugDrawFlags::None);
    EXPECT_EQ(draw.lineCount(), 0);
}

TEST(PhysicsWorldDebugDraw, VelocityArrowsAppear) {
    PhysicsWorld world;
    world.setGravity({0.f, -9.81f, 0.f});
    BodyDescriptor desc;
    desc.type  = BodyType::Dynamic;
    desc.shape = std::make_shared<SphereShape>(0.5f);
    desc.mass  = 1.f;
    world.createBody(desc);
    world.step(1.f / 6.f);  // let it fall and gain speed

    RecordingDraw noVel, withVel;
    world.debugDraw(noVel,  DebugDrawFlags::BodyShapes);
    world.debugDraw(withVel, DebugDrawFlags::BodyShapes | DebugDrawFlags::Velocities);
    EXPECT_GT(withVel.lineCount(), noVel.lineCount());
}

TEST(PhysicsWorldDebugDraw, CompoundShapeDrawsChildren) {
    PhysicsWorld world;
    auto comp = std::make_shared<CompoundShape>();
    Transform t1; t1.position = {0.f, 0.5f, 0.f};
    Transform t2; t2.position = {0.f,-0.5f, 0.f};
    comp->addChild(std::make_shared<SphereShape>(0.3f), t1);
    comp->addChild(std::make_shared<SphereShape>(0.3f), t2);

    BodyDescriptor desc;
    desc.type  = BodyType::Dynamic;
    desc.shape = comp;
    desc.mass  = 1.f;
    world.createBody(desc);

    RecordingDraw draw;
    world.debugDraw(draw, DebugDrawFlags::BodyShapes);
    // Two spheres × 3 rings × 16 segs
    EXPECT_EQ(draw.lineCount(), 2 * 3 * 16);
}

// ── IPhysicsProfiler ──────────────────────────────────────────────────────────

struct ScopeRecord {
    std::string name;
    bool begin;
};

class RecordingProfiler : public IPhysicsProfiler {
public:
    std::vector<ScopeRecord> events;
    void beginScope(const char* name) override { events.push_back({name, true}); }
    void endScope  (const char* name) override { events.push_back({name, false}); }
    void reset() { events.clear(); }
};

TEST(IPhysicsProfiler, MacroCallsBeginAndEnd) {
    RecordingProfiler prof;
    {
        CAMPELLO_PROFILE_SCOPE(&prof, "TestScope");
    }
    ASSERT_EQ(prof.events.size(), 2u);
    EXPECT_EQ(prof.events[0].name, "TestScope");
    EXPECT_TRUE(prof.events[0].begin);
    EXPECT_EQ(prof.events[1].name, "TestScope");
    EXPECT_FALSE(prof.events[1].begin);
}

TEST(IPhysicsProfiler, NullProfilerNoOp) {
    EXPECT_NO_FATAL_FAILURE({
        CAMPELLO_PROFILE_SCOPE(nullptr, "NoOp");
    });
}

TEST(IPhysicsProfiler, WorldStepFiresProfilerScopes) {
    PhysicsWorld world;
    RecordingProfiler prof;
    world.setProfiler(&prof);

    BodyDescriptor desc;
    desc.type  = BodyType::Dynamic;
    desc.shape = std::make_shared<SphereShape>(0.5f);
    desc.mass  = 1.f;
    world.createBody(desc);

    world.step(1.f / 60.f);
    // Expect at least BroadPhase, NarrowPhase, Solver, Integrate scopes
    EXPECT_GE(prof.events.size(), 8u);

    // Every begin should have a matching end
    int depth = 0;
    for (const auto& e : prof.events) {
        if (e.begin) ++depth; else --depth;
    }
    EXPECT_EQ(depth, 0);
}

TEST(IPhysicsProfiler, DisableProfiler) {
    PhysicsWorld world;
    RecordingProfiler prof;
    world.setProfiler(&prof);
    world.step(1.f / 60.f);
    int count1 = static_cast<int>(prof.events.size());

    prof.reset();
    world.setProfiler(nullptr);
    world.step(1.f / 60.f);
    EXPECT_EQ(prof.events.size(), 0u);  // no calls after disabling
    (void)count1;
}

} // namespace campello::physics
