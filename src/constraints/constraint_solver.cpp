#include <campello_physics/constraint.h>
#include <algorithm>

namespace campello::physics {

void ConstraintSolver::add(std::shared_ptr<Constraint> c) {
    m_constraints.push_back(std::move(c));
}

void ConstraintSolver::remove(const std::shared_ptr<Constraint>& c) {
    auto it = std::find(m_constraints.begin(), m_constraints.end(), c);
    if (it != m_constraints.end()) m_constraints.erase(it);
}

void ConstraintSolver::clear() {
    m_constraints.clear();
}

void ConstraintSolver::solve(BodyPool& pool, float dt) {
    for (auto& c : m_constraints)
        c->prepare(pool, dt, baumgarte, slop);

    for (auto& c : m_constraints)
        c->warmStart(pool);

    for (int i = 0; i < iterations; ++i)
        for (auto& c : m_constraints)
            c->solveVelocity(pool);
}

void ConstraintSolver::solvePositions(BodyPool& pool, float dt) {
    if (positionIterations <= 0) return;
    for (int i = 0; i < positionIterations; ++i)
        for (auto& c : m_constraints)
            c->solvePosition(pool, dt, positionAlpha);
}

} // namespace campello::physics
