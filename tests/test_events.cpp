#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/listeners.h>
#include <cmath>
#include <vector>

using namespace campello::physics;

namespace {
using V3 = vm::Vector3<float>;
} // namespace

// ── IStepListener ─────────────────────────────────────────────────────────────

TEST(Events, StepListenerPrePostCalled) {
    struct Listener : IStepListener {
        int pre = 0, post = 0;
        void onPreStep (float) override { ++pre;  }
        void onPostStep(float) override { ++post; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));
    Listener l;
    world.addStepListener(&l);

    world.step(1.f / 60.f);
    EXPECT_EQ(l.pre,  1) << "onPreStep should fire once per substep";
    EXPECT_EQ(l.post, 1) << "onPostStep should fire once per substep";

    world.step(1.f / 60.f);
    EXPECT_EQ(l.pre,  2);
    EXPECT_EQ(l.post, 2);

    world.removeStepListener(&l);
}

TEST(Events, StepListenerReceivesDt) {
    struct Listener : IStepListener {
        std::vector<float> preDts, postDts;
        void onPreStep (float dt) override { preDts.push_back(dt);  }
        void onPostStep(float dt) override { postDts.push_back(dt); }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));
    world.setFixedTimestep(1.f / 60.f);
    world.setSubsteps(1);

    Listener l;
    world.addStepListener(&l);
    world.step(1.f / 60.f);

    ASSERT_EQ(l.preDts.size(), 1u);
    EXPECT_NEAR(l.preDts[0], 1.f / 60.f, 1e-5f);
    EXPECT_NEAR(l.postDts[0], 1.f / 60.f, 1e-5f);

    world.removeStepListener(&l);
}

TEST(Events, StepListenerRemovedStopsCallbacks) {
    struct Listener : IStepListener {
        int count = 0;
        void onPreStep(float) override { ++count; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));
    Listener l;
    world.addStepListener(&l);
    world.step(1.f / 60.f);
    EXPECT_EQ(l.count, 1);

    world.removeStepListener(&l);
    world.step(1.f / 60.f);
    EXPECT_EQ(l.count, 1) << "Removed listener should not receive further callbacks";
}

TEST(Events, MultipleStepListeners) {
    struct Listener : IStepListener {
        int count = 0;
        void onPreStep(float) override { ++count; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));
    Listener l1, l2;
    world.addStepListener(&l1);
    world.addStepListener(&l2);
    world.step(1.f / 60.f);
    EXPECT_EQ(l1.count, 1);
    EXPECT_EQ(l2.count, 1);

    world.removeStepListener(&l1);
    world.removeStepListener(&l2);
}

// ── IContactListener ──────────────────────────────────────────────────────────

TEST(Events, ContactAddedFires) {
    struct Listener : IContactListener {
        int added = 0;
        void onContactAdded(Body, Body, const ContactManifold&) override { ++added; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.contactIterations = 20;

    // Static floor
    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    // Sphere starting very close to floor surface (immediate contact)
    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 0.4f, 0.f);
    sphereDesc.restitution = 0.f;
    [[maybe_unused]] auto _sphere = world.createBody(sphereDesc);

    Listener l;
    world.addContactListener(&l);

    for (int i = 0; i < 10; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(l.added, 0) << "onContactAdded should fire when sphere touches floor";

    world.removeContactListener(&l);
}

TEST(Events, ContactPersistedFires) {
    struct Listener : IContactListener {
        int added = 0, persisted = 0;
        void onContactAdded    (Body, Body, const ContactManifold&) override { ++added; }
        void onContactPersisted(Body, Body, const ContactManifold&) override { ++persisted; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.contactIterations = 20;

    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    floorDesc.restitution = 0.f;
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 0.4f, 0.f);
    sphereDesc.restitution = 0.f;
    sphereDesc.linearDamping  = 0.5f;
    sphereDesc.angularDamping = 0.5f;
    [[maybe_unused]] auto _sphere = world.createBody(sphereDesc);

    Listener l;
    world.addContactListener(&l);

    // Simulate long enough for sphere to settle (many persisted events)
    for (int i = 0; i < 120; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(l.added,     0) << "onContactAdded should fire";
    EXPECT_GT(l.persisted, 0) << "onContactPersisted should fire on subsequent frames";

    world.removeContactListener(&l);
}

TEST(Events, ContactRemovedFires) {
    struct Listener : IContactListener {
        int added = 0, removed = 0;
        void onContactAdded  (Body, Body, const ContactManifold&) override { ++added; }
        void onContactRemoved(Body, Body)                         override { ++removed; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.contactIterations = 20;

    // Floor
    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    floorDesc.restitution = 1.f;  // perfect bounce so sphere rises back up
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    // Sphere dropped from close range — will bounce and separate
    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 0.6f, 0.f);
    sphereDesc.restitution = 1.f;
    sphereDesc.linearDamping = sphereDesc.angularDamping = 0.f;
    [[maybe_unused]] auto _sphere = world.createBody(sphereDesc);

    Listener l;
    world.addContactListener(&l);

    // Simulate 3 seconds — sphere should bounce several times, each separation fires removed
    for (int i = 0; i < 180; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(l.added,   0) << "onContactAdded should fire";
    EXPECT_GT(l.removed, 0) << "onContactRemoved should fire when sphere bounces away";

    world.removeContactListener(&l);
}

TEST(Events, ContactListenerRemovedStopsCallbacks) {
    struct Listener : IContactListener {
        int added = 0;
        void onContactAdded(Body, Body, const ContactManifold&) override { ++added; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));

    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 0.4f, 0.f);
    [[maybe_unused]] auto _sphere = world.createBody(sphereDesc);

    Listener l;
    world.addContactListener(&l);

    // Let contact establish
    for (int i = 0; i < 10; ++i)
        world.step(1.f / 60.f);
    EXPECT_GT(l.added, 0);

    world.removeContactListener(&l);
    int countAfterRemove = l.added;

    // Run more steps — no new events should arrive
    for (int i = 0; i < 30; ++i)
        world.step(1.f / 60.f);

    EXPECT_EQ(l.added, countAfterRemove)
        << "Removed contact listener should receive no further callbacks";
}

TEST(Events, MultipleContactListeners) {
    struct Listener : IContactListener {
        int added = 0;
        void onContactAdded(Body, Body, const ContactManifold&) override { ++added; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));

    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 0.4f, 0.f);
    [[maybe_unused]] auto _sphere = world.createBody(sphereDesc);

    Listener l1, l2;
    world.addContactListener(&l1);
    world.addContactListener(&l2);

    for (int i = 0; i < 10; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(l1.added, 0) << "First listener should receive events";
    EXPECT_GT(l2.added, 0) << "Second listener should receive events";
    EXPECT_EQ(l1.added, l2.added) << "Both listeners should see same event count";

    world.removeContactListener(&l1);
    world.removeContactListener(&l2);
}

// ── ITriggerListener ──────────────────────────────────────────────────────────

TEST(Events, TriggerEnterFires) {
    struct Listener : ITriggerListener {
        int entered = 0;
        void onTriggerEnter(Body, Body) override { ++entered; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    // Sensor at origin, radius 1
    BodyDescriptor sensorDesc;
    sensorDesc.type  = BodyType::Sensor;
    sensorDesc.shape = std::make_shared<SphereShape>(1.f);
    sensorDesc.transform.position = V3(0.f, 0.f, 0.f);
    [[maybe_unused]] auto _sensor = world.createBody(sensorDesc);

    // Body starting outside, moving into sensor
    BodyDescriptor bodyDesc;
    bodyDesc.type  = BodyType::Dynamic;
    bodyDesc.mass  = 1.f;
    bodyDesc.shape = std::make_shared<SphereShape>(0.3f);
    bodyDesc.transform.position = V3(5.f, 0.f, 0.f);
    bodyDesc.linearDamping = bodyDesc.angularDamping = 0.f;
    Body mover = world.createBody(bodyDesc);
    mover.setLinearVelocity(V3(-5.f, 0.f, 0.f));

    Listener l;
    world.addTriggerListener(&l);

    for (int i = 0; i < 60; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(l.entered, 0) << "onTriggerEnter should fire when body enters sensor";

    world.removeTriggerListener(&l);
}

TEST(Events, TriggerExitFires) {
    struct Listener : ITriggerListener {
        int entered = 0, exited = 0;
        void onTriggerEnter(Body, Body) override { ++entered; }
        void onTriggerExit (Body, Body) override { ++exited;  }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    // Sensor at origin, radius 1
    BodyDescriptor sensorDesc;
    sensorDesc.type  = BodyType::Sensor;
    sensorDesc.shape = std::make_shared<SphereShape>(1.f);
    sensorDesc.transform.position = V3(0.f, 0.f, 0.f);
    [[maybe_unused]] auto _sensor = world.createBody(sensorDesc);

    // Body that flies through sensor from left to right
    BodyDescriptor bodyDesc;
    bodyDesc.type  = BodyType::Dynamic;
    bodyDesc.mass  = 1.f;
    bodyDesc.shape = std::make_shared<SphereShape>(0.2f);
    bodyDesc.transform.position = V3(-5.f, 0.f, 0.f);
    bodyDesc.linearDamping = bodyDesc.angularDamping = 0.f;
    Body mover = world.createBody(bodyDesc);
    mover.setLinearVelocity(V3(5.f, 0.f, 0.f));  // fast enough to exit in ~2 s

    Listener l;
    world.addTriggerListener(&l);

    // Simulate 4 seconds — enough for entry and exit
    for (int i = 0; i < 240; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(l.entered, 0) << "onTriggerEnter should fire";
    EXPECT_GT(l.exited,  0) << "onTriggerExit should fire when body leaves sensor";

    world.removeTriggerListener(&l);
}

TEST(Events, TriggerListenerRemovedStopsCallbacks) {
    struct Listener : ITriggerListener {
        int entered = 0;
        void onTriggerEnter(Body, Body) override { ++entered; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor sensorDesc;
    sensorDesc.type  = BodyType::Sensor;
    sensorDesc.shape = std::make_shared<SphereShape>(2.f);
    sensorDesc.transform.position = V3(0.f, 0.f, 0.f);
    [[maybe_unused]] auto _sensor = world.createBody(sensorDesc);

    BodyDescriptor bodyDesc;
    bodyDesc.type  = BodyType::Dynamic;
    bodyDesc.mass  = 1.f;
    bodyDesc.shape = std::make_shared<SphereShape>(0.3f);
    bodyDesc.transform.position = V3(10.f, 0.f, 0.f);
    bodyDesc.linearDamping = bodyDesc.angularDamping = 0.f;
    Body mover = world.createBody(bodyDesc);
    mover.setLinearVelocity(V3(-5.f, 0.f, 0.f));

    Listener l;
    world.addTriggerListener(&l);

    // Remove listener immediately — should get no events
    world.removeTriggerListener(&l);

    for (int i = 0; i < 120; ++i)
        world.step(1.f / 60.f);

    EXPECT_EQ(l.entered, 0) << "Removed trigger listener should get no callbacks";
}

TEST(Events, MultipleTriggerListeners) {
    struct Listener : ITriggerListener {
        int entered = 0;
        void onTriggerEnter(Body, Body) override { ++entered; }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, 0.f, 0.f));

    BodyDescriptor sensorDesc;
    sensorDesc.type  = BodyType::Sensor;
    sensorDesc.shape = std::make_shared<SphereShape>(1.f);
    sensorDesc.transform.position = V3(0.f, 0.f, 0.f);
    [[maybe_unused]] auto _sensor = world.createBody(sensorDesc);

    BodyDescriptor bodyDesc;
    bodyDesc.type  = BodyType::Dynamic;
    bodyDesc.mass  = 1.f;
    bodyDesc.shape = std::make_shared<SphereShape>(0.3f);
    bodyDesc.transform.position = V3(5.f, 0.f, 0.f);
    bodyDesc.linearDamping = bodyDesc.angularDamping = 0.f;
    Body mover = world.createBody(bodyDesc);
    mover.setLinearVelocity(V3(-5.f, 0.f, 0.f));

    Listener l1, l2;
    world.addTriggerListener(&l1);
    world.addTriggerListener(&l2);

    for (int i = 0; i < 60; ++i)
        world.step(1.f / 60.f);

    EXPECT_GT(l1.entered, 0) << "First trigger listener should receive events";
    EXPECT_GT(l2.entered, 0) << "Second trigger listener should receive events";
    EXPECT_EQ(l1.entered, l2.entered);

    world.removeTriggerListener(&l1);
    world.removeTriggerListener(&l2);
}

// ── ContactManifold data in callbacks ────────────────────────────────────────

TEST(Events, ContactManifoldHasValidData) {
    struct Listener : IContactListener {
        bool gotValidManifold = false;
        void onContactAdded(Body, Body, const ContactManifold& m) override {
            if (m.count > 0) {
                // Normal should be approximately unit length
                const auto& n = m.points[0].normal;
                float len = std::sqrt(n.x()*n.x() + n.y()*n.y() + n.z()*n.z());
                if (std::abs(len - 1.f) < 0.1f && m.points[0].depth >= 0.f)
                    gotValidManifold = true;
            }
        }
    };

    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.contactIterations = 20;

    BodyDescriptor floorDesc;
    floorDesc.type  = BodyType::Static;
    floorDesc.shape = std::make_shared<BoxShape>(V3(10.f, 0.1f, 10.f));
    floorDesc.transform.position = V3(0.f, -0.1f, 0.f);
    [[maybe_unused]] auto _floor = world.createBody(floorDesc);

    BodyDescriptor sphereDesc;
    sphereDesc.type  = BodyType::Dynamic;
    sphereDesc.mass  = 1.f;
    sphereDesc.shape = std::make_shared<SphereShape>(0.5f);
    sphereDesc.transform.position = V3(0.f, 0.4f, 0.f);
    [[maybe_unused]] auto _sphere = world.createBody(sphereDesc);

    Listener l;
    world.addContactListener(&l);

    for (int i = 0; i < 10; ++i)
        world.step(1.f / 60.f);

    EXPECT_TRUE(l.gotValidManifold)
        << "Contact manifold should contain a unit normal and non-negative depth";

    world.removeContactListener(&l);
}
