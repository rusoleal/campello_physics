#include <gtest/gtest.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/shapes/cylinder_shape.h>
#include <campello_physics/shapes/compound_shape.h>
#include <campello_physics/shapes/convex_hull_shape.h>
#include <cmath>
#include <string>
#include <string_view>

namespace campello::physics {

// ── helpers ───────────────────────────────────────────────────────────────────

static bool nearEq(float a, float b, float eps = 1e-4f) {
    return std::abs(a - b) <= eps;
}
static bool vec3Eq(const vm::Vector3<float>& a,
                   const vm::Vector3<float>& b, float eps = 1e-4f) {
    return nearEq(a.x(),b.x(),eps) && nearEq(a.y(),b.y(),eps) && nearEq(a.z(),b.z(),eps);
}
static bool quatEq(const vm::Quaternion<float>& a,
                   const vm::Quaternion<float>& b, float eps = 1e-4f) {
    // q and -q represent the same rotation; compare both signs
    bool sameSign = nearEq(a.x(),b.x(),eps) && nearEq(a.y(),b.y(),eps) &&
                    nearEq(a.z(),b.z(),eps) && nearEq(a.w(),b.w(),eps);
    bool flipSign = nearEq(a.x(),-b.x(),eps) && nearEq(a.y(),-b.y(),eps) &&
                    nearEq(a.z(),-b.z(),eps) && nearEq(a.w(),-b.w(),eps);
    return sameSign || flipSign;
}

// ── Basic round-trip ──────────────────────────────────────────────────────────

TEST(Serialization, EmptyWorldRoundTrip) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    std::string json = w.serialize();
    EXPECT_FALSE(json.empty());

    PhysicsWorld w2;
    EXPECT_TRUE(w2.deserialize(json));
    EXPECT_TRUE(vec3Eq(w2.gravity(), {0.f, -9.81f, 0.f}));
    EXPECT_EQ(w2.bodyPool().activeCount(), 0u);
}

TEST(Serialization, SerializeProducesValidJson) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.mass  = 2.f;
    [[maybe_unused]] auto unused = w.createBody(d);
    std::string json = w.serialize();
    // Must contain key markers
    EXPECT_NE(json.find("\"bodies\""), std::string::npos);
    EXPECT_NE(json.find("\"sphere\""), std::string::npos);
    EXPECT_NE(json.find("\"gravity\""), std::string::npos);
}

TEST(Serialization, InvalidJsonReturnsFalse) {
    PhysicsWorld w;
    EXPECT_FALSE(w.deserialize("{bad json}"));
    EXPECT_FALSE(w.deserialize("not json at all"));
    EXPECT_FALSE(w.deserialize(""));
}

// ── World settings ────────────────────────────────────────────────────────────

TEST(Serialization, WorldSettingsRoundTrip) {
    PhysicsWorld w;
    w.setGravity({1.f, -4.f, 0.5f});
    w.setFixedTimestep(1.f / 120.f);
    w.setSubsteps(3);
    w.contactIterations    = 15;
    w.constraintIterations = 8;
    w.contactBaumgarte     = 0.4f;
    w.contactSlop          = 0.002f;

    PhysicsWorld w2;
    EXPECT_TRUE(w2.deserialize(w.serialize()));

    EXPECT_TRUE(vec3Eq(w2.gravity(), {1.f, -4.f, 0.5f}));
    EXPECT_TRUE(nearEq(w2.fixedTimestep(), 1.f / 120.f, 1e-6f));
    EXPECT_EQ(w2.contactIterations, 15);
    EXPECT_EQ(w2.constraintIterations, 8);
    EXPECT_TRUE(nearEq(w2.contactBaumgarte, 0.4f));
    EXPECT_TRUE(nearEq(w2.contactSlop, 0.002f));
}

// ── Body state ────────────────────────────────────────────────────────────────

TEST(Serialization, DynamicBodyRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type            = BodyType::Dynamic;
    d.shape           = std::make_shared<SphereShape>(0.75f);
    d.mass            = 3.f;
    d.linearDamping   = 0.05f;
    d.angularDamping  = 0.1f;
    d.restitution     = 0.7f;
    d.friction        = 0.3f;
    d.layer           = 0x00FF;
    d.mask            = 0xFF00;
    d.ccdEnabled      = true;
    d.transform.position = vm::Vector3<float>(1.f, 2.f, 3.f);
    d.linearVelocity     = vm::Vector3<float>(0.1f, -0.2f, 0.3f);
    [[maybe_unused]] auto _b = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    ASSERT_EQ(w2.bodyPool().activeCount(), 1u);

    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_EQ(bd.type, BodyType::Dynamic);
        EXPECT_TRUE(vec3Eq(bd.transform.position, {1.f, 2.f, 3.f}));
        EXPECT_TRUE(vec3Eq(bd.linearVelocity,     {0.1f,-0.2f,0.3f}));
        EXPECT_TRUE(nearEq(1.f/bd.invMass,        3.f));
        EXPECT_TRUE(nearEq(bd.linearDamping,      0.05f));
        EXPECT_TRUE(nearEq(bd.angularDamping,     0.1f));
        EXPECT_TRUE(nearEq(bd.restitution,        0.7f));
        EXPECT_TRUE(nearEq(bd.friction,           0.3f));
        EXPECT_EQ(bd.layer,     0x00FFu);
        EXPECT_EQ(bd.mask,      0xFF00u);
        EXPECT_TRUE(bd.ccdEnabled);
        ASSERT_NE(bd.shape, nullptr);
        ASSERT_EQ(bd.shape->type(), ShapeType::Sphere);
        EXPECT_TRUE(nearEq(static_cast<const SphereShape*>(bd.shape.get())->radius(), 0.75f));
    });
}

TEST(Serialization, StaticBodyRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(vm::Vector3<float>(5.f, 0.5f, 5.f));
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    ASSERT_EQ(w2.bodyPool().activeCount(), 1u);

    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_EQ(bd.type, BodyType::Static);
        ASSERT_NE(bd.shape, nullptr);
        EXPECT_EQ(bd.shape->type(), ShapeType::Box);
        const auto& he = static_cast<const BoxShape*>(bd.shape.get())->halfExtents();
        EXPECT_TRUE(vec3Eq(he, {5.f, 0.5f, 5.f}));
    });
}

TEST(Serialization, KinematicBodyRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Kinematic;
    d.shape = std::make_shared<CapsuleShape>(0.4f, 0.8f);
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_EQ(bd.type, BodyType::Kinematic);
        ASSERT_NE(bd.shape, nullptr);
        EXPECT_EQ(bd.shape->type(), ShapeType::Capsule);
        const auto* cs = static_cast<const CapsuleShape*>(bd.shape.get());
        EXPECT_TRUE(nearEq(cs->radius(),     0.4f));
        EXPECT_TRUE(nearEq(cs->halfHeight(), 0.8f));
    });
}

TEST(Serialization, SensorBodyRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Sensor;
    d.shape = std::make_shared<SphereShape>(2.f);
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_EQ(bd.type, BodyType::Sensor);
    });
}

// ── Shape round-trips ─────────────────────────────────────────────────────────

TEST(Serialization, CylinderShapeRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<CylinderShape>(0.3f, 0.6f);
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        ASSERT_EQ(bd.shape->type(), ShapeType::Cylinder);
        const auto* cy = static_cast<const CylinderShape*>(bd.shape.get());
        EXPECT_TRUE(nearEq(cy->radius(),     0.3f));
        EXPECT_TRUE(nearEq(cy->halfHeight(), 0.6f));
    });
}

TEST(Serialization, ConvexHullShapeRoundTrip) {
    PhysicsWorld w;
    std::vector<vm::Vector3<float>> pts = {
        {1.f,0.f,0.f},{-1.f,0.f,0.f},
        {0.f,1.f,0.f},{0.f,-1.f,0.f},
        {0.f,0.f,1.f},{0.f,0.f,-1.f}
    };
    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<ConvexHullShape>(pts);
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        ASSERT_EQ(bd.shape->type(), ShapeType::ConvexHull);
        const auto& rpts = static_cast<const ConvexHullShape*>(bd.shape.get())->points();
        ASSERT_EQ(rpts.size(), pts.size());
        for (std::size_t i = 0; i < pts.size(); ++i)
            EXPECT_TRUE(vec3Eq(rpts[i], pts[i]));
    });
}

TEST(Serialization, CompoundShapeRoundTrip) {
    PhysicsWorld w;
    auto comp = std::make_shared<CompoundShape>();
    Transform t1; t1.position = {0.f, 0.5f, 0.f};
    Transform t2; t2.position = {0.f,-0.5f, 0.f};
    comp->addChild(std::make_shared<SphereShape>(0.3f), t1);
    comp->addChild(std::make_shared<BoxShape>(vm::Vector3<float>(0.2f,0.2f,0.2f)), t2);
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.shape = comp;
    d.mass  = 1.f;
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        ASSERT_EQ(bd.shape->type(), ShapeType::Compound);
        const auto* cp = static_cast<const CompoundShape*>(bd.shape.get());
        ASSERT_EQ(cp->children().size(), 2u);
        EXPECT_EQ(cp->children()[0].shape->type(), ShapeType::Sphere);
        EXPECT_EQ(cp->children()[1].shape->type(), ShapeType::Box);
        EXPECT_TRUE(vec3Eq(cp->children()[0].localTransform.position, {0.f,0.5f,0.f}));
        EXPECT_TRUE(vec3Eq(cp->children()[1].localTransform.position, {0.f,-0.5f,0.f}));
    });
}

TEST(Serialization, NullShapeRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.shape = nullptr;
    d.mass  = 1.f;
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_EQ(bd.shape, nullptr);
    });
}

// ── Multiple bodies ───────────────────────────────────────────────────────────

TEST(Serialization, MultipleBodiesRoundTrip) {
    PhysicsWorld w;
    for (int i = 0; i < 5; ++i) {
        BodyDescriptor d;
        d.type  = BodyType::Dynamic;
        d.shape = std::make_shared<SphereShape>(0.1f * (i + 1));
        d.mass  = static_cast<float>(i + 1);
        d.transform.position = vm::Vector3<float>(static_cast<float>(i), 0.f, 0.f);
        [[maybe_unused]] auto unused = w.createBody(d);
    }

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    EXPECT_EQ(w2.bodyPool().activeCount(), 5u);
}

// ── Deserialize replaces state ────────────────────────────────────────────────

TEST(Serialization, DeserializeReplacesExistingBodies) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.shape = std::make_shared<SphereShape>(1.f);
    d.mass  = 1.f;
    [[maybe_unused]] auto unused = w.createBody(d);
    std::string json = w.serialize();

    // Put 10 bodies in target world first
    PhysicsWorld w2;
    for (int i = 0; i < 10; ++i) {
        BodyDescriptor bd;
        bd.type  = BodyType::Static;
        bd.shape = std::make_shared<BoxShape>(vm::Vector3<float>(1.f,1.f,1.f));
        [[maybe_unused]] auto unused = w2.createBody(bd);
    }
    ASSERT_TRUE(w2.deserialize(json));
    EXPECT_EQ(w2.bodyPool().activeCount(), 1u);
}

// ── Transform and rotation ────────────────────────────────────────────────────

TEST(Serialization, TransformRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.mass  = 1.f;
    d.transform.position = vm::Vector3<float>(1.5f, -2.f, 0.7f);
    d.transform.rotation = vm::Quaternion<float>::axisAngle(
        vm::Vector3<float>(0.f, 1.f, 0.f), 1.2f);
    [[maybe_unused]] auto unused = w.createBody(d);

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_TRUE(vec3Eq(bd.transform.position, {1.5f,-2.f,0.7f}));
        EXPECT_TRUE(quatEq(bd.transform.rotation, d.transform.rotation));
    });
}

// ── Simulate then save/restore ────────────────────────────────────────────────

TEST(Serialization, SimulatedStateRoundTrip) {
    PhysicsWorld w;
    w.setGravity({0.f, -9.81f, 0.f});
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.mass  = 1.f;
    [[maybe_unused]] auto _b = w.createBody(d);

    // Simulate for half a second
    for (int i = 0; i < 30; ++i) w.step(1.f / 60.f);

    vm::Vector3<float> posAfterSim;
    vm::Vector3<float> velAfterSim;
    w.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        posAfterSim = bd.transform.position;
        velAfterSim = bd.linearVelocity;
    });

    // Save and restore into fresh world
    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));

    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_TRUE(vec3Eq(bd.transform.position, posAfterSim));
        EXPECT_TRUE(vec3Eq(bd.linearVelocity,     velAfterSim));
    });

    // Simulate one more step from both — positions should remain identical
    w.step(1.f / 60.f);
    w2.step(1.f / 60.f);

    vm::Vector3<float> pos1, pos2;
    w.bodyPool().forEach([&](uint32_t,  const BodyData& bd) { pos1 = bd.transform.position; });
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) { pos2 = bd.transform.position; });
    EXPECT_TRUE(vec3Eq(pos1, pos2, 1e-3f));
}

// ── Sleep state ───────────────────────────────────────────────────────────────

TEST(Serialization, SleepingBodyRoundTrip) {
    PhysicsWorld w;
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.shape = std::make_shared<SphereShape>(0.5f);
    d.mass  = 1.f;
    Body b = w.createBody(d);
    b.sleep();

    PhysicsWorld w2;
    ASSERT_TRUE(w2.deserialize(w.serialize()));
    w2.bodyPool().forEach([&](uint32_t, const BodyData& bd) {
        EXPECT_TRUE(bd.isSleeping);
    });
}

} // namespace campello::physics
