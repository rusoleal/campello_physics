#pragma once

// Uniform interface for cross-engine benchmarking.

struct Adapter {
    virtual ~Adapter() = default;

    // Engine name for benchmark labels.
    virtual const char* name() const = 0;

    // Re-initialise the world with the given thread count.
    // Must discard any previous state.
    virtual void init(int workerThreads) = 0;

    // Scene setups — must match between engines.
    virtual void setupFreeFall(int bodyCount) = 0;
    virtual void setupPile(int bodyCount) = 0;
    virtual void setupConstraintChains(int chains, int linksPerChain) = 0;
    virtual void setupMixedScene(int bodyCount) = 0;

    // Advance one fixed timestep.
    virtual void step(float dt) = 0;
};
