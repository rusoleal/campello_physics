#pragma once

#include <campello_physics/body_pool.h>
#include <memory>
#include <vector>

namespace campello::physics {

// ── ConstraintRow ─────────────────────────────────────────────────────────────
// One scalar constraint equation J * v + bias = 0 (or inequality with clamp).
// Jacobian layout: J = [J_va | J_wa | J_vb | J_wb] (1 × 12).

struct ConstraintRow {
    vm::Vector3<float> J_va{0.f, 0.f, 0.f};
    vm::Vector3<float> J_wa{0.f, 0.f, 0.f};
    vm::Vector3<float> J_vb{0.f, 0.f, 0.f};
    vm::Vector3<float> J_wb{0.f, 0.f, 0.f};
    float bias      = 0.f;
    float lambdaMin = -1e30f;
    float lambdaMax =  1e30f;
    float lambda    = 0.f;   // accumulated impulse — persists for warm starting
    float effMass   = 0.f;   // 1 / (J * M^-1 * J^T), precomputed by prepare()
};

// ── Constraint ────────────────────────────────────────────────────────────────

class Constraint {
public:
    static constexpr int kMaxRows = 6;

    virtual ~Constraint() = default;

    // Precompute rows (Jacobians, biases, effective masses) for this step.
    // lambda is preserved from the previous frame for warm starting.
    virtual void prepare(BodyPool& pool, float dt, float baumgarte, float slop) = 0;

    // Apply warm-start impulses (scaled accumulated lambda) before the solve loop.
    virtual void warmStart(BodyPool& pool) = 0;

    // One iteration of velocity correction. Called N times by the solver.
    virtual void solveVelocity(BodyPool& pool) = 0;

    // One iteration of position correction (split-impulse pass, runs after integration).
    // alpha: correction gain [0,1].  dt: fixed timestep.
    virtual void solvePosition(BodyPool& pool, float dt, float alpha) = 0;

    [[nodiscard]] Body bodyA() const noexcept { return m_bodyA; }
    [[nodiscard]] Body bodyB() const noexcept { return m_bodyB; }

protected:
    Body m_bodyA, m_bodyB;
};

// ── ConstraintSolver ──────────────────────────────────────────────────────────

class ConstraintSolver {
public:
    int   iterations = 10;
    float baumgarte  = 0.2f;
    float slop       = 0.005f;   // position error allowance before Baumgarte kicks in

    // Position solve settings (split-impulse pass run after integration).
    // Set positionIterations > 0 to enable.  positionAlpha: correction gain [0,1].
    int   positionIterations = 3;
    float positionAlpha      = 0.2f;

    void add(std::shared_ptr<Constraint> c);
    void remove(const std::shared_ptr<Constraint>& c);
    void clear();

    // Runs prepare → warm start → N × solveVelocity on all registered constraints.
    void solve(BodyPool& pool, float dt);

    // Runs M × solvePosition on all registered constraints (call after integration).
    void solvePositions(BodyPool& pool, float dt);

    [[nodiscard]] const std::vector<std::shared_ptr<Constraint>>& constraints() const noexcept {
        return m_constraints;
    }

private:
    std::vector<std::shared_ptr<Constraint>> m_constraints;
};

} // namespace campello::physics
