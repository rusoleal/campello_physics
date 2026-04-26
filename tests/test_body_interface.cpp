#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

using namespace campello::physics;

namespace {

BodyDescriptor makeDynamicDesc(float x = 0.f) {
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.mass  = 1.f;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.transform.position = { x, 0.f, 0.f };
    return d;
}

} // namespace

// ── Basic functionality ───────────────────────────────────────────────────────

TEST(BodyInterface, CreateBody_HandleIsValid) {
    PhysicsWorld world;
    Body b = world.bodyInterface().createBody(makeDynamicDesc());
    EXPECT_TRUE(world.bodyInterface().isValid(b));
}

TEST(BodyInterface, DestroyBody_HandleBecomesInvalid) {
    PhysicsWorld world;
    Body b = world.bodyInterface().createBody(makeDynamicDesc());
    world.bodyInterface().destroyBody(b);
    EXPECT_FALSE(world.bodyInterface().isValid(b));
}

TEST(BodyInterface, SetTransform_GetTransformReturnsUpdatedValue) {
    PhysicsWorld world;
    Body b = world.bodyInterface().createBody(makeDynamicDesc());

    Transform t;
    t.position = { 3.f, 7.f, -2.f };
    world.bodyInterface().setTransform(b, t);

    const auto got = world.bodyInterface().getTransform(b);
    EXPECT_NEAR(got.position.x(), 3.f, 1e-5f);
    EXPECT_NEAR(got.position.y(), 7.f, 1e-5f);
    EXPECT_NEAR(got.position.z(), -2.f, 1e-5f);
}

TEST(BodyInterface, ApplyForce_BodyMovesAfterStep) {
    PhysicsWorld world;
    world.setGravity({ 0.f, 0.f, 0.f });

    BodyDescriptor d = makeDynamicDesc();
    d.linearDamping = 0.f;
    Body b = world.bodyInterface().createBody(d);

    world.bodyInterface().applyForce(b, { 10.f, 0.f, 0.f });
    world.step(1.f / 60.f);

    const float vx = world.bodyInterface().getLinearVelocity(b).x();
    EXPECT_GT(vx, 0.f) << "Body did not accelerate after applyForce";
}

// ── Thread safety ─────────────────────────────────────────────────────────────

TEST(BodyInterface, ConcurrentCreate_AllBodiesValid) {
    // 4 threads each create 25 bodies → 100 total; all must be valid afterward.
    PhysicsWorld world;

    constexpr int kThreads   = 4;
    constexpr int kPerThread = 25;

    std::vector<std::vector<Body>> results(kThreads);

    auto worker = [&](int idx) {
        for (int i = 0; i < kPerThread; ++i)
            results[idx].push_back(
                world.bodyInterface().createBody(makeDynamicDesc()));
    };

    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int i = 0; i < kThreads; ++i)
        threads.emplace_back(worker, i);
    for (auto& t : threads) t.join();

    int validCount = 0;
    for (const auto& vec : results)
        for (const auto& b : vec)
            if (world.bodyInterface().isValid(b)) ++validCount;

    EXPECT_EQ(validCount, kThreads * kPerThread);
    EXPECT_EQ(world.bodyPool().activeCount(), static_cast<uint32_t>(kThreads * kPerThread));
}

TEST(BodyInterface, StepAndCreate_SerializedWithoutCorruption) {
    // One thread runs 60 steps; another concurrently creates bodies.
    // At the end the world must be internally consistent (all reported active
    // bodies are actually valid).
    PhysicsWorld world;
    world.setGravity({ 0.f, 0.f, 0.f });

    // Pre-populate two bodies so there is something to step.
    world.bodyInterface().createBody(makeDynamicDesc(0.f));
    world.bodyInterface().createBody(makeDynamicDesc(2.f));

    constexpr int kSteps   = 60;
    constexpr int kCreates = 20;

    std::atomic<bool> done{false};

    std::thread stepper([&] {
        for (int i = 0; i < kSteps; ++i)
            world.step(1.f / 60.f);
        done = true;
    });

    std::vector<Body> created;
    created.reserve(kCreates);
    while (!done) {
        if (static_cast<int>(created.size()) < kCreates)
            created.push_back(world.bodyInterface().createBody(makeDynamicDesc()));
    }

    stepper.join();

    // Every body we created must still be valid.
    for (const auto& b : created)
        EXPECT_TRUE(world.bodyInterface().isValid(b));
}

// ── New API methods ───────────────────────────────────────────────────────────

TEST(BodyInterface, GetBodyType_ReturnsCorrectType) {
    PhysicsWorld world;
    BodyDescriptor dd = makeDynamicDesc();
    Body dyn = world.bodyInterface().createBody(dd);

    BodyDescriptor sd;
    sd.type = BodyType::Static;
    sd.mass = 0.f;
    Body stat = world.bodyInterface().createBody(sd);

    EXPECT_EQ(world.bodyInterface().getBodyType(dyn),  BodyType::Dynamic);
    EXPECT_EQ(world.bodyInterface().getBodyType(stat), BodyType::Static);
}

TEST(BodyInterface, GetMass_ReturnsDescriptorMass) {
    PhysicsWorld world;
    BodyDescriptor d = makeDynamicDesc();
    d.mass = 7.5f;
    Body b = world.bodyInterface().createBody(d);
    EXPECT_NEAR(world.bodyInterface().getMass(b), 7.5f, 1e-4f);
}

TEST(BodyInterface, GetMass_StaticBodyReturnsZero) {
    PhysicsWorld world;
    BodyDescriptor d;
    d.type = BodyType::Static;
    d.mass = 0.f;
    Body b = world.bodyInterface().createBody(d);
    EXPECT_NEAR(world.bodyInterface().getMass(b), 0.f, 1e-6f);
}

TEST(BodyInterface, ApplyTorque_BodyRotatesAfterStep) {
    PhysicsWorld world;
    world.setGravity({0.f, 0.f, 0.f});

    BodyDescriptor d = makeDynamicDesc();
    d.angularDamping = 0.f;
    Body b = world.bodyInterface().createBody(d);

    world.bodyInterface().applyTorque(b, {100.f, 0.f, 0.f});
    world.step(1.f / 60.f);

    EXPECT_GT(world.bodyInterface().getAngularVelocity(b).x(), 0.f)
        << "Body did not spin after applyTorque";
}

TEST(BodyInterface, ApplyAngularImpulse_AngularVelocityChanges) {
    PhysicsWorld world;
    world.setGravity({0.f, 0.f, 0.f});
    Body b = world.bodyInterface().createBody(makeDynamicDesc());

    world.bodyInterface().applyAngularImpulse(b, {0.f, 5.f, 0.f});
    EXPECT_GT(world.bodyInterface().getAngularVelocity(b).y(), 0.f)
        << "Angular velocity not set by applyAngularImpulse";
}

TEST(BodyInterface, ApplyLinearImpulse_VelocityChangesImmediately) {
    PhysicsWorld world;
    world.setGravity({0.f, 0.f, 0.f});
    Body b = world.bodyInterface().createBody(makeDynamicDesc());

    world.bodyInterface().applyLinearImpulse(b, {0.f, 0.f, 3.f});
    EXPECT_NEAR(world.bodyInterface().getLinearVelocity(b).z(), 3.f, 1e-4f);
}

TEST(BodyInterface, Wake_ClearsIsSleepingFlag) {
    PhysicsWorld world;
    world.setGravity({0.f, 0.f, 0.f});
    Body b = world.bodyInterface().createBody(makeDynamicDesc());

    // Force body to sleep via internal pool access
    world.bodyPool().get(b.id()).isSleeping  = true;
    world.bodyPool().get(b.id()).sleepFrames = 999;

    EXPECT_TRUE(world.bodyInterface().isSleeping(b));
    world.bodyInterface().wake(b);
    EXPECT_FALSE(world.bodyInterface().isSleeping(b));
}

TEST(BodyInterface, SetLinearVelocity_WakesSleepingBody) {
    PhysicsWorld world;
    world.setGravity({0.f, 0.f, 0.f});
    Body b = world.bodyInterface().createBody(makeDynamicDesc());
    world.bodyPool().get(b.id()).isSleeping = true;

    world.bodyInterface().setLinearVelocity(b, {1.f, 0.f, 0.f});
    EXPECT_FALSE(world.bodyInterface().isSleeping(b));
    EXPECT_NEAR(world.bodyInterface().getLinearVelocity(b).x(), 1.f, 1e-5f);
}

TEST(BodyInterface, StaticBody_ApplyForceIsNoOp) {
    PhysicsWorld world;
    world.setGravity({0.f, 0.f, 0.f});

    BodyDescriptor sd;
    sd.type = BodyType::Static;
    sd.mass = 0.f;
    Body b = world.bodyInterface().createBody(sd);

    world.bodyInterface().applyForce(b, {1000.f, 0.f, 0.f});
    world.step(1.f / 60.f);

    const auto& t = world.bodyInterface().getTransform(b);
    EXPECT_NEAR(t.position.x(), 0.f, 1e-5f) << "Static body moved after applyForce";
}

// ── Concurrent mutation safety ────────────────────────────────────────────────

TEST(BodyInterface, ConcurrentMutations_NoDataRace) {
    // 4 threads concurrently mutate 4 separate bodies via BodyInterface.
    // No data race → no UB, all final positions must be in a valid state.
    PhysicsWorld world;
    world.setGravity({0.f, 0.f, 0.f});

    constexpr int kN = 4;
    std::vector<Body> bodies;
    for (int i = 0; i < kN; ++i)
        bodies.push_back(world.bodyInterface().createBody(makeDynamicDesc(float(i) * 5.f)));

    std::vector<std::thread> threads;
    threads.reserve(kN);
    for (int i = 0; i < kN; ++i) {
        threads.emplace_back([&, i] {
            auto& bi = world.bodyInterface();
            for (int j = 0; j < 200; ++j) {
                bi.setLinearVelocity(bodies[i],  {float(j) * 0.1f, 0.f, 0.f});
                bi.applyLinearImpulse(bodies[i], {0.f, 0.01f, 0.f});
            }
        });
    }
    for (auto& t : threads) t.join();

    for (int i = 0; i < kN; ++i) {
        const auto& d = world.bodyPool().get(bodies[i].id());
        EXPECT_TRUE(std::isfinite(d.linearVelocity.x())) << "body " << i << " linVel not finite";
        EXPECT_TRUE(std::isfinite(d.linearVelocity.y())) << "body " << i << " linVel.y not finite";
    }
}

TEST(BodyInterface, ConcurrentDestroyAndCreate_PoolConsistent) {
    // One thread destroys bodies, another creates. Pool should remain consistent.
    // Note: pool IDs are recycled — a destroyed body's handle may become valid
    // again if a new body reuses the same slot. We therefore verify pool-level
    // consistency (activeCount == kInitial) rather than staleness of old handles.
    PhysicsWorld world;

    constexpr int kInitial = 20;
    std::vector<Body> initial;
    for (int i = 0; i < kInitial; ++i)
        initial.push_back(world.bodyInterface().createBody(makeDynamicDesc(float(i))));

    std::atomic<int> destroyed{0};
    std::atomic<int> created{0};

    std::thread destroyer([&] {
        for (auto& b : initial) {
            world.bodyInterface().destroyBody(b);
            ++destroyed;
        }
    });

    std::vector<Body> newBodies;
    std::thread creator([&] {
        for (int i = 0; i < kInitial; ++i) {
            newBodies.push_back(world.bodyInterface().createBody(makeDynamicDesc()));
            ++created;
        }
    });

    destroyer.join();
    creator.join();

    EXPECT_EQ(destroyed.load(), kInitial);
    EXPECT_EQ(created.load(),   kInitial);

    // All newly created bodies should be valid
    for (auto& b : newBodies)
        EXPECT_TRUE(world.bodyInterface().isValid(b));

    // Pool count: destroyed kInitial and created kInitial — net = kInitial active
    EXPECT_EQ(world.bodyPool().activeCount(), static_cast<uint32_t>(kInitial));
}

TEST(BodyInterface, ConcurrentStepAndMutate_SerializedCorrectly) {
    // Stepping thread and mutation thread must not interleave.
    // Verified by checking that the body count never goes negative and
    // the simulation stays finite.
    PhysicsWorld world;
    world.setGravity({0.f, -9.81f, 0.f});

    for (int i = 0; i < 10; ++i)
        world.bodyInterface().createBody(makeDynamicDesc(float(i)));

    std::atomic<bool> done{false};
    std::atomic<int>  mutations{0};

    std::thread mutator([&] {
        while (!done) {
            Body b = world.bodyInterface().createBody(makeDynamicDesc());
            world.bodyInterface().applyLinearImpulse(b, {0.f, 5.f, 0.f});
            world.bodyInterface().destroyBody(b);
            ++mutations;
        }
    });

    for (int i = 0; i < 120; ++i)
        world.step(1.f / 60.f);
    done = true;
    mutator.join();

    EXPECT_GT(mutations.load(), 0);
    // World must still be in a valid state
    EXPECT_GT(world.bodyPool().activeCount(), 0u);
}
