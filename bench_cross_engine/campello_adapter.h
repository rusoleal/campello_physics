#pragma once

#include "adapter.h"
#include <campello_physics/physics_world.h>
#include <memory>

struct CampelloAdapter : Adapter {
    const char* name() const override { return "Campello"; }
    void init(int workerThreads) override;
    void setupFreeFall(int bodyCount) override;
    void setupPile(int bodyCount) override;
    void setupConstraintChains(int chains, int linksPerChain) override;
    void setupMixedScene(int bodyCount) override;
    void step(float dt) override;

private:
    std::unique_ptr<campello::physics::PhysicsWorld> m_world;
    int m_workerThreads = 1;
};
