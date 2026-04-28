#include "campello_adapter.h"
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <cmath>

using namespace campello::physics;
using V3 = vm::Vector3<float>;

void CampelloAdapter::init(int workerThreads) {
    m_workerThreads = workerThreads;
    m_world = std::make_unique<PhysicsWorld>();
    m_world->setGravity(V3(0.f, -9.81f, 0.f));
    m_world->setFixedTimestep(1.f / 60.f);
    m_world->setSubsteps(1);
    m_world->setWorkerThreads(workerThreads);
}

void CampelloAdapter::setupFreeFall(int bodyCount) {
    m_world = std::make_unique<PhysicsWorld>();
    m_world->setGravity(V3(0.f, -9.81f, 0.f));
    m_world->setFixedTimestep(1.f / 60.f);
    m_world->setSubsteps(1);
    m_world->setWorkerThreads(m_workerThreads);

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 2.5f;
    for (int i = 0; i < bodyCount; ++i) {
        BodyDescriptor d;
        d.type = BodyType::Dynamic;
        d.mass = 1.f;
        d.shape = std::make_shared<SphereShape>(0.5f);
        int ix = i % side;
        int iz = i / side;
        d.transform.position = V3(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.1f,
            (float(iz) - side * 0.5f) * spacing);
        [[maybe_unused]] auto unused = m_world->createBody(d);
    }
}

void CampelloAdapter::setupPile(int bodyCount) {
    m_world = std::make_unique<PhysicsWorld>();
    m_world->setGravity(V3(0.f, -9.81f, 0.f));
    m_world->setFixedTimestep(1.f / 60.f);
    m_world->setSubsteps(1);
    m_world->setWorkerThreads(m_workerThreads);

    // Floor
    BodyDescriptor floor;
    floor.type = BodyType::Static;
    floor.shape = std::make_shared<BoxShape>(V3(200.f, 0.5f, 200.f));
    floor.transform.position = V3(0.f, -0.5f, 0.f);
    [[maybe_unused]] auto unused = m_world->createBody(floor);

    // Boxes
    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 1.2f;
    for (int i = 0; i < bodyCount; ++i) {
        BodyDescriptor d;
        d.type = BodyType::Dynamic;
        d.mass = 1.f;
        d.shape = std::make_shared<BoxShape>(V3(0.5f, 0.5f, 0.5f));
        int ix = i % side;
        int iz = i / side;
        d.transform.position = V3(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.05f,
            (float(iz) - side * 0.5f) * spacing);
        [[maybe_unused]] auto unused = m_world->createBody(d);
    }
}

void CampelloAdapter::setupConstraintChains(int chains, int linksPerChain) {
    m_world = std::make_unique<PhysicsWorld>();
    m_world->setGravity(V3(0.f, -9.81f, 0.f));
    m_world->setFixedTimestep(1.f / 60.f);
    m_world->setSubsteps(1);
    m_world->setWorkerThreads(m_workerThreads);

    // Floor
    BodyDescriptor floor;
    floor.type = BodyType::Static;
    floor.shape = std::make_shared<BoxShape>(V3(200.f, 0.5f, 200.f));
    floor.transform.position = V3(0.f, -0.5f, 0.f);
    [[maybe_unused]] auto unused = m_world->createBody(floor);

    const float linkHalfH = 0.3f;
    const float linkSpacing = linkHalfH * 2.f + 0.05f;

    for (int c = 0; c < chains; ++c) {
        Body prev = {};
        for (int l = 0; l < linksPerChain; ++l) {
            BodyDescriptor d;
            d.type = BodyType::Dynamic;
            d.mass = 1.f;
            d.shape = std::make_shared<CapsuleShape>(0.15f, linkHalfH);
            d.transform.position = V3(
                float(c) * 2.f,
                5.f + float(l) * linkSpacing,
                0.f);
            Body b = m_world->createBody(d);

            if (l == 0) {
                // Anchor first link to static body (world anchor)
                BodyDescriptor anchor;
                anchor.type = BodyType::Static;
                anchor.shape = std::make_shared<SphereShape>(0.1f);
                anchor.transform.position = V3(float(c) * 2.f, 5.f - linkSpacing, 0.f);
                Body anchorBody = m_world->createBody(anchor);
                auto constraint = BallSocketConstraint::create(
                    anchorBody, V3(float(c) * 2.f, 5.f - linkSpacing * 0.5f, 0.f),
                    b, V3(float(c) * 2.f, 5.f - linkSpacing * 0.5f, 0.f));
                m_world->addConstraint(constraint);
            } else {
                auto constraint = BallSocketConstraint::create(
                    prev, V3(float(c) * 2.f, 5.f + (float(l) - 0.5f) * linkSpacing, 0.f),
                    b, V3(float(c) * 2.f, 5.f + (float(l) - 0.5f) * linkSpacing, 0.f));
                m_world->addConstraint(constraint);
            }
            prev = b;
        }
    }
}

void CampelloAdapter::setupMixedScene(int bodyCount) {
    m_world = std::make_unique<PhysicsWorld>();
    m_world->setGravity(V3(0.f, -9.81f, 0.f));
    m_world->setFixedTimestep(1.f / 60.f);
    m_world->setSubsteps(1);
    m_world->setWorkerThreads(m_workerThreads);

    // Floor
    BodyDescriptor floor;
    floor.type = BodyType::Static;
    floor.shape = std::make_shared<BoxShape>(V3(200.f, 0.5f, 200.f));
    floor.transform.position = V3(0.f, -0.5f, 0.f);
    [[maybe_unused]] auto unused = m_world->createBody(floor);

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 1.2f;
    for (int i = 0; i < bodyCount; ++i) {
        BodyDescriptor d;
        d.type = BodyType::Dynamic;
        d.mass = 1.f;
        int ix = i % side;
        int iz = i / side;
        d.transform.position = V3(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.05f,
            (float(iz) - side * 0.5f) * spacing);
        // Mix shapes
        if (i % 3 == 0) d.shape = std::make_shared<BoxShape>(V3(0.5f, 0.5f, 0.5f));
        else if (i % 3 == 1) d.shape = std::make_shared<SphereShape>(0.5f);
        else d.shape = std::make_shared<CapsuleShape>(0.3f, 0.4f);
        [[maybe_unused]] auto unused = m_world->createBody(d);
    }
}

void CampelloAdapter::step(float dt) {
    m_world->step(dt);
}
