#include <campello_physics/constraints/distance_constraint.h>
#include "constraint_utils.h"
#include <cmath>

namespace campello::physics {

using namespace detail;

std::shared_ptr<DistanceConstraint> DistanceConstraint::create(
    Body a, const vm::Vector3<float>& anchorLocalA,
    Body b, const vm::Vector3<float>& anchorLocalB,
    float minDist, float maxDist)
{
    auto c = std::shared_ptr<DistanceConstraint>(new DistanceConstraint{});
    c->m_bodyA   = a;
    c->m_bodyB   = b;
    c->m_anchorA = anchorLocalA;
    c->m_anchorB = anchorLocalB;
    c->m_minDist = minDist;
    c->m_maxDist = maxDist;
    return c;
}

void DistanceConstraint::prepare(BodyPool& pool, float dt, float baumgarte, float /*slop*/) {
    const auto& da = pool.get(m_bodyA.id());
    const auto& db = pool.get(m_bodyB.id());

    const auto wA  = da.transform.position + da.transform.rotation.rotated(m_anchorA);
    const auto wB  = db.transform.position + db.transform.rotation.rotated(m_anchorB);
    const auto sep  = wA - wB;
    const float dist = std::sqrt(lenSq3(sep));

    m_active = false;

    if (dist < 1e-6f) return;  // degenerate: skip

    const auto n = sep * (1.f / dist);  // unit direction from B to A

    const auto rA = da.transform.rotation.rotated(m_anchorA);
    const auto rB = db.transform.rotation.rotated(m_anchorB);

    auto& r = m_row;
    r.J_va = n;
    r.J_wa = cross3(rA, n);
    r.J_vb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
    r.J_wb = vm::Vector3<float>(-cross3(rB, n).x(),
                                -cross3(rB, n).y(),
                                -cross3(rB, n).z());

    bool equality = (m_minDist == m_maxDist);
    if (equality) {
        float C = dist - m_minDist;
        r.bias      = (baumgarte / dt) * C;
        r.lambdaMin = -1e30f;
        r.lambdaMax =  1e30f;
        computeEffMass(da, db, r);
        m_active = true;
    } else if (dist < m_minDist) {
        float C = dist - m_minDist;  // negative → push apart
        r.bias      = (baumgarte / dt) * C;
        r.lambdaMin = -1e30f;
        r.lambdaMax =  0.f;
        r.lambda    = r.lambda < 0.f ? r.lambda : 0.f;
        computeEffMass(da, db, r);
        m_active = true;
    } else if (dist > m_maxDist) {
        // Too far apart — apply negative impulse (along n from B→A) to pull together.
        float C = dist - m_maxDist;  // positive when violated
        r.bias      = (baumgarte / dt) * C;
        r.lambdaMin = -1e30f;
        r.lambdaMax =  0.f;  // only pull, never push
        r.lambda    = r.lambda < 0.f ? r.lambda : 0.f;
        computeEffMass(da, db, r);
        m_active = true;
    } else {
        r.lambda = 0.f;
    }
}

void DistanceConstraint::warmStart(BodyPool& pool) {
    if (!m_active) return;
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    warmStartRow(da, db, m_row);
}

void DistanceConstraint::solveVelocity(BodyPool& pool) {
    if (!m_active) return;
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    solveRow(da, db, m_row);
}

void DistanceConstraint::solvePosition(BodyPool& pool, float /*dt*/, float alpha) {
    if (!m_active) return;
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());

    const auto wA  = da.transform.position + da.transform.rotation.rotated(m_anchorA);
    const auto wB  = db.transform.position + db.transform.rotation.rotated(m_anchorB);
    const float dist = std::sqrt(lenSq3(wA - wB));
    if (dist < 1e-6f) return;

    float C = 0.f;
    if      (m_minDist == m_maxDist) C = dist - m_minDist;
    else if (dist < m_minDist)       C = dist - m_minDist;
    else if (dist > m_maxDist)       C = dist - m_maxDist;
    else return;

    solvePosRow(da, db, m_row, C, alpha);
}

} // namespace campello::physics
