#include "bullet_adapter.h"
#include <cmath>

BulletAdapter::BulletAdapter() = default;
BulletAdapter::~BulletAdapter() = default;

void BulletAdapter::reset() {
    m_world.reset();
    m_solver.reset();
    m_broadphase.reset();
    m_dispatcher.reset();
    m_collisionConfig.reset();

    m_constraints.clear();
    m_bodies.clear();
    m_motionStates.clear();
    m_shapes.clear();
}

void BulletAdapter::init(int workerThreads) {
    m_workerThreads = workerThreads;
    reset();
}

static btVector3 toBt(float x, float y, float z) {
    return btVector3(x, y, z);
}

static btTransform toBtTr(const btVector3& pos) {
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(pos);
    return tr;
}

void BulletAdapter::setupFreeFall(int bodyCount) {
    reset();

    m_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_collisionConfig.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionConfig.get());
    m_world->setGravity(toBt(0.f, -9.81f, 0.f));

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 2.5f;
    for (int i = 0; i < bodyCount; ++i) {
        int ix = i % side;
        int iz = i / side;
        btVector3 pos(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.1f,
            (float(iz) - side * 0.5f) * spacing);

        auto shape = std::make_unique<btSphereShape>(0.5f);
        btScalar mass = 1.f;
        btVector3 localInertia(0, 0, 0);
        shape->calculateLocalInertia(mass, localInertia);

        auto ms = std::make_unique<btDefaultMotionState>(toBtTr(pos));
        btRigidBody::btRigidBodyConstructionInfo info(mass, ms.get(), shape.get(), localInertia);
        auto body = std::make_unique<btRigidBody>(info);
        body->setActivationState(DISABLE_DEACTIVATION);

        m_world->addRigidBody(body.get());
        m_bodies.push_back(std::move(body));
        m_motionStates.push_back(std::move(ms));
        m_shapes.push_back(std::move(shape));
    }
}

void BulletAdapter::setupPile(int bodyCount) {
    reset();

    m_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_collisionConfig.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionConfig.get());
    m_world->setGravity(toBt(0.f, -9.81f, 0.f));

    // Floor
    {
        auto shape = std::make_unique<btBoxShape>(toBt(200.f, 0.5f, 200.f));
        btScalar mass = 0.f;
        auto ms = std::make_unique<btDefaultMotionState>(toBtTr(toBt(0.f, -0.5f, 0.f)));
        btRigidBody::btRigidBodyConstructionInfo info(mass, ms.get(), shape.get(), btVector3(0,0,0));
        auto body = std::make_unique<btRigidBody>(info);
        m_world->addRigidBody(body.get());
        m_bodies.push_back(std::move(body));
        m_motionStates.push_back(std::move(ms));
        m_shapes.push_back(std::move(shape));
    }

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 1.2f;
    for (int i = 0; i < bodyCount; ++i) {
        int ix = i % side;
        int iz = i / side;
        btVector3 pos(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.05f,
            (float(iz) - side * 0.5f) * spacing);

        auto shape = std::make_unique<btBoxShape>(toBt(0.5f, 0.5f, 0.5f));
        btScalar mass = 1.f;
        btVector3 localInertia(0, 0, 0);
        shape->calculateLocalInertia(mass, localInertia);

        auto ms = std::make_unique<btDefaultMotionState>(toBtTr(pos));
        btRigidBody::btRigidBodyConstructionInfo info(mass, ms.get(), shape.get(), localInertia);
        auto body = std::make_unique<btRigidBody>(info);
        body->setActivationState(DISABLE_DEACTIVATION);

        m_world->addRigidBody(body.get());
        m_bodies.push_back(std::move(body));
        m_motionStates.push_back(std::move(ms));
        m_shapes.push_back(std::move(shape));
    }
}

void BulletAdapter::setupConstraintChains(int chains, int linksPerChain) {
    reset();

    m_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_collisionConfig.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionConfig.get());
    m_world->setGravity(toBt(0.f, -9.81f, 0.f));

    // Floor
    {
        auto shape = std::make_unique<btBoxShape>(toBt(200.f, 0.5f, 200.f));
        btScalar mass = 0.f;
        auto ms = std::make_unique<btDefaultMotionState>(toBtTr(toBt(0.f, -0.5f, 0.f)));
        btRigidBody::btRigidBodyConstructionInfo info(mass, ms.get(), shape.get(), btVector3(0,0,0));
        auto body = std::make_unique<btRigidBody>(info);
        m_world->addRigidBody(body.get());
        m_bodies.push_back(std::move(body));
        m_motionStates.push_back(std::move(ms));
        m_shapes.push_back(std::move(shape));
    }

    const float linkHalfH = 0.3f;
    const float linkSpacing = linkHalfH * 2.f + 0.05f;

    for (int c = 0; c < chains; ++c) {
        btRigidBody* prev = nullptr;
        for (int l = 0; l < linksPerChain; ++l) {
            btVector3 pos(
                float(c) * 2.f,
                5.f + float(l) * linkSpacing,
                0.f);

            auto shape = std::make_unique<btCapsuleShape>(0.15f, linkHalfH * 2.f);
            btScalar mass = 1.f;
            btVector3 localInertia(0, 0, 0);
            shape->calculateLocalInertia(mass, localInertia);

            auto ms = std::make_unique<btDefaultMotionState>(toBtTr(pos));
            btRigidBody::btRigidBodyConstructionInfo info(mass, ms.get(), shape.get(), localInertia);
            auto body = std::make_unique<btRigidBody>(info);
            // Damping to roughly match Jolt/campello defaults
            body->setDamping(0.01f, 0.05f);

            m_world->addRigidBody(body.get());
            btRigidBody* b = body.get();
            m_bodies.push_back(std::move(body));
            m_motionStates.push_back(std::move(ms));
            m_shapes.push_back(std::move(shape));

            if (l == 0) {
                // Anchor to static body
                btVector3 anchorPos(float(c) * 2.f, 5.f - linkSpacing, 0.f);
                auto anchorShape = std::make_unique<btSphereShape>(0.1f);
                auto anchorMs = std::make_unique<btDefaultMotionState>(toBtTr(anchorPos));
                btRigidBody::btRigidBodyConstructionInfo anchorInfo(0.f, anchorMs.get(), anchorShape.get(), btVector3(0,0,0));
                auto anchorBody = std::make_unique<btRigidBody>(anchorInfo);
                m_world->addRigidBody(anchorBody.get());

                btVector3 pivot(float(c) * 2.f, 5.f - linkSpacing * 0.5f, 0.f);
                auto constraint = std::make_unique<btPoint2PointConstraint>(*anchorBody, *b, pivot, pivot);
                m_world->addConstraint(constraint.get(), true);
                m_constraints.push_back(std::move(constraint));

                m_bodies.push_back(std::move(anchorBody));
                m_motionStates.push_back(std::move(anchorMs));
                m_shapes.push_back(std::move(anchorShape));
            } else {
                btVector3 pivot(float(c) * 2.f, 5.f + (float(l) - 0.5f) * linkSpacing, 0.f);
                auto constraint = std::make_unique<btPoint2PointConstraint>(*prev, *b, pivot, pivot);
                m_world->addConstraint(constraint.get(), true);
                m_constraints.push_back(std::move(constraint));
            }
            prev = b;
        }
    }
}

void BulletAdapter::setupMixedScene(int bodyCount) {
    reset();

    m_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_collisionConfig.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionConfig.get());
    m_world->setGravity(toBt(0.f, -9.81f, 0.f));

    // Floor
    {
        auto shape = std::make_unique<btBoxShape>(toBt(200.f, 0.5f, 200.f));
        btScalar mass = 0.f;
        auto ms = std::make_unique<btDefaultMotionState>(toBtTr(toBt(0.f, -0.5f, 0.f)));
        btRigidBody::btRigidBodyConstructionInfo info(mass, ms.get(), shape.get(), btVector3(0,0,0));
        auto body = std::make_unique<btRigidBody>(info);
        m_world->addRigidBody(body.get());
        m_bodies.push_back(std::move(body));
        m_motionStates.push_back(std::move(ms));
        m_shapes.push_back(std::move(shape));
    }

    const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(bodyCount))));
    const float spacing = 1.2f;
    for (int i = 0; i < bodyCount; ++i) {
        int ix = i % side;
        int iz = i / side;
        btVector3 pos(
            (float(ix) - side * 0.5f) * spacing,
            5.f + float(i) * 0.05f,
            (float(iz) - side * 0.5f) * spacing);

        std::unique_ptr<btCollisionShape> shape;
        if (i % 3 == 0) shape = std::make_unique<btBoxShape>(toBt(0.5f, 0.5f, 0.5f));
        else if (i % 3 == 1) shape = std::make_unique<btSphereShape>(0.5f);
        else shape = std::make_unique<btCapsuleShape>(0.3f, 0.4f * 2.f);

        btScalar mass = 1.f;
        btVector3 localInertia(0, 0, 0);
        shape->calculateLocalInertia(mass, localInertia);

        auto ms = std::make_unique<btDefaultMotionState>(toBtTr(pos));
        btRigidBody::btRigidBodyConstructionInfo info(mass, ms.get(), shape.get(), localInertia);
        auto body = std::make_unique<btRigidBody>(info);
        body->setActivationState(DISABLE_DEACTIVATION);

        m_world->addRigidBody(body.get());
        m_bodies.push_back(std::move(body));
        m_motionStates.push_back(std::move(ms));
        m_shapes.push_back(std::move(shape));
    }
}

void BulletAdapter::step(float dt) {
    m_world->stepSimulation(dt, 1, dt);
}
