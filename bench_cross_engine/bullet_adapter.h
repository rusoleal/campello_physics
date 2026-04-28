#pragma once

#include "adapter.h"
#include <btBulletDynamicsCommon.h>
#include <memory>
#include <vector>

struct BulletAdapter : Adapter {
    BulletAdapter();
    ~BulletAdapter();

    const char* name() const override { return "Bullet"; }
    void init(int workerThreads) override;
    void setupFreeFall(int bodyCount) override;
    void setupPile(int bodyCount) override;
    void setupConstraintChains(int chains, int linksPerChain) override;
    void setupMixedScene(int bodyCount) override;
    void step(float dt) override;

private:
    void reset();

    int m_workerThreads = 1;

    std::unique_ptr<btDefaultCollisionConfiguration> m_collisionConfig;
    std::unique_ptr<btCollisionDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::unique_ptr<btDiscreteDynamicsWorld> m_world;

    // We own all bodies, shapes, and constraints
    std::vector<std::unique_ptr<btRigidBody>> m_bodies;
    std::vector<std::unique_ptr<btCollisionShape>> m_shapes;
    std::vector<std::unique_ptr<btTypedConstraint>> m_constraints;
    std::vector<std::unique_ptr<btDefaultMotionState>> m_motionStates;
};
