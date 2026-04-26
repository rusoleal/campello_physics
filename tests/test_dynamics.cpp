#include <gtest/gtest.h>
#include <campello_physics/body_pool.h>
#include <campello_physics/integrator.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <cmath>

using namespace campello::physics;

namespace {

using V3 = vm::Vector3<float>;
using Q  = vm::Quaternion<float>;

BodyPool makePool() { return BodyPool{}; }

BodyDescriptor dynamicDesc(float mass = 1.f) {
    BodyDescriptor d;
    d.type = BodyType::Dynamic;
    d.mass = mass;
    d.linearDamping  = 0.f;
    d.angularDamping = 0.f;
    return d;
}

IntegratorSettings noSleep() {
    IntegratorSettings s;
    s.sleepFramesRequired = 1000000;
    return s;
}

} // namespace

// Free-fall: after N steps v.y == -g*t (no damping, no initial vel)
TEST(Dynamics, FreeFallVelocity) {
    BodyPool pool;
    auto body = pool.createBody(dynamicDesc(1.f));

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, -9.81f, 0.f);

    const float dt = 1.f / 60.f;
    const int   N  = 120;
    for (int i = 0; i < N; ++i)
        integrate(pool, s, dt);

    float expectedVy = -9.81f * (dt * N);
    float gotVy = body.linearVelocity().y();
    EXPECT_NEAR(gotVy, expectedVy, 1e-3f);
}

// Free-fall: position after N steps should match kinematics
TEST(Dynamics, FreeFallPosition) {
    BodyPool pool;
    auto body = pool.createBody(dynamicDesc(1.f));

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, -9.81f, 0.f);

    const float dt = 1.f / 60.f;
    const int   N  = 60;
    for (int i = 0; i < N; ++i)
        integrate(pool, s, dt);

    // Semi-implicit Euler: v_n = -g*n*dt, y_n = sum(v_k * dt) for k=1..n
    // = -g * dt^2 * (1+2+...+n) = -g * dt^2 * n*(n+1)/2
    float expected = -9.81f * dt * dt * (N * (N + 1)) / 2.f;
    float got = body.transform().position.y();
    EXPECT_NEAR(got, expected, 1e-2f);
}

// Static body is not moved by the integrator
TEST(Dynamics, StaticBodyUnmoved) {
    BodyPool pool;
    BodyDescriptor d;
    d.type = BodyType::Static;
    d.linearDamping = d.angularDamping = 0.f;
    auto body = pool.createBody(d);

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, -9.81f, 0.f);

    integrate(pool, s, 1.f / 60.f);

    EXPECT_FLOAT_EQ(body.transform().position.y(), 0.f);
    EXPECT_FLOAT_EQ(body.linearVelocity().y(), 0.f);
}

// Kinematic body is not moved by the integrator either
TEST(Dynamics, KinematicBodyUnmoved) {
    BodyPool pool;
    BodyDescriptor d;
    d.type = BodyType::Kinematic;
    d.linearDamping = d.angularDamping = 0.f;
    auto body = pool.createBody(d);

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, -9.81f, 0.f);

    integrate(pool, s, 1.f / 60.f);

    EXPECT_FLOAT_EQ(body.transform().position.y(), 0.f);
}

// applyLinearImpulse changes velocity by imp * invMass = imp / mass
TEST(Dynamics, LinearImpulse) {
    BodyPool pool;
    auto body = pool.createBody(dynamicDesc(2.f));

    body.applyLinearImpulse(V3(4.f, 0.f, 0.f));

    // impulse applied immediately, no step needed
    EXPECT_NEAR(body.linearVelocity().x(), 2.f, 1e-6f);  // 4 / 2 = 2
}

// applyForce accumulates and is consumed in the next integrate step
TEST(Dynamics, ForceAccumulation) {
    BodyPool pool;
    auto body = pool.createBody(dynamicDesc(1.f));

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, 0.f, 0.f);

    body.applyForce(V3(10.f, 0.f, 0.f));

    const float dt = 0.1f;
    integrate(pool, s, dt);

    EXPECT_NEAR(body.linearVelocity().x(), 10.f * dt, 1e-5f);

    // Force accumulator should be cleared — second step should not add more
    integrate(pool, s, dt);
    EXPECT_NEAR(body.linearVelocity().x(), 10.f * dt, 1e-5f);
}

// Linear damping reduces speed each step
TEST(Dynamics, LinearDamping) {
    BodyPool pool;
    BodyDescriptor d = dynamicDesc(1.f);
    d.linearDamping = 0.5f;
    auto body = pool.createBody(d);

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, 0.f, 0.f);

    body.setLinearVelocity(V3(10.f, 0.f, 0.f));

    const float dt = 1.f / 60.f;
    integrate(pool, s, dt);

    float speed = body.linearVelocity().x();
    EXPECT_LT(speed, 10.f);
    EXPECT_GT(speed, 0.f);
}

// Sleep system: integrator accumulates sleepFrames when body is below threshold.
// Island-level sleep (isSleeping) is decided by PhysicsWorld::updateIslandSleep,
// not by the integrator directly.
TEST(Dynamics, SleepSystem) {
    BodyPool pool;
    auto body = pool.createBody(dynamicDesc(1.f));

    IntegratorSettings s;
    s.gravity               = V3(0.f, 0.f, 0.f);
    s.sleepLinearThreshold  = 1.f;    // generous threshold — body at rest qualifies
    s.sleepAngularThreshold = 1.f;
    s.sleepFramesRequired   = 5;

    EXPECT_FALSE(body.isSleeping());

    for (int i = 0; i < s.sleepFramesRequired; ++i)
        integrate(pool, s, 1.f / 60.f);

    // Integrator tracks sleepFrames; island manager decides isSleeping.
    EXPECT_GE(pool.get(body.id()).sleepFrames, s.sleepFramesRequired);
    EXPECT_FLOAT_EQ(body.linearVelocity().x(), 0.f);
}

// Sleeping body is not integrated (position stays constant)
TEST(Dynamics, SleepingBodySkipped) {
    BodyPool pool;
    auto body = pool.createBody(dynamicDesc(1.f));

    body.sleep();
    EXPECT_TRUE(body.isSleeping());

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, -9.81f, 0.f);

    integrate(pool, s, 1.f / 60.f);
    EXPECT_FLOAT_EQ(body.transform().position.y(), 0.f);
}

// Quaternion stays unit-length after many rotation steps
TEST(Dynamics, QuaternionNormalization) {
    BodyPool pool;
    auto body = pool.createBody(dynamicDesc(1.f));

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, 0.f, 0.f);

    // spin the body
    body.setAngularVelocity(V3(1.f, 2.f, 3.f));

    const float dt = 1.f / 60.f;
    for (int i = 0; i < 600; ++i)
        integrate(pool, s, dt);

    const auto& q = body.transform().rotation;
    float lenSq = q.data[0]*q.data[0] + q.data[1]*q.data[1]
                + q.data[2]*q.data[2] + q.data[3]*q.data[3];
    EXPECT_NEAR(lenSq, 1.f, 1e-5f);
}

// applyForceAt produces torque (angular velocity grows)
TEST(Dynamics, ForceAtProducesTorque) {
    BodyPool pool;
    BodyDescriptor d = dynamicDesc(1.f);
    d.shape = std::make_shared<SphereShape>(1.f);
    auto body = pool.createBody(d);

    IntegratorSettings s = noSleep();
    s.gravity = V3(0.f, 0.f, 0.f);

    // Force in +X at point (0,1,0) → torque around -Z
    body.applyForceAt(V3(1.f, 0.f, 0.f), V3(0.f, 1.f, 0.f));
    integrate(pool, s, 1.f / 60.f);

    // Angular velocity should be non-zero (specifically around -Z axis)
    const auto w = body.angularVelocity();
    float wMag = w.x()*w.x() + w.y()*w.y() + w.z()*w.z();
    EXPECT_GT(wMag, 0.f);
}
