#include <campello_physics/constraints/hinge_constraint.h>
#include "constraint_utils.h"
#include <cmath>

namespace campello::physics {

using namespace detail;

std::shared_ptr<HingeConstraint> HingeConstraint::create(
    Body a, const vm::Vector3<float>& anchorLocalA, const vm::Vector3<float>& axisLocalA,
    Body b, const vm::Vector3<float>& anchorLocalB, const vm::Vector3<float>& axisLocalB)
{
    auto c = std::shared_ptr<HingeConstraint>(new HingeConstraint{});
    c->m_bodyA      = a;
    c->m_bodyB      = b;
    c->m_anchorA    = anchorLocalA;
    c->m_anchorB    = anchorLocalB;
    c->m_axisLocalA = norm3(axisLocalA);
    c->m_axisLocalB = norm3(axisLocalB);

    // Pick a reference perpendicular in A's local space for angle measurement.
    vm::Vector3<float> dummy;
    perp3(c->m_axisLocalA, c->m_refPerpLocalA, dummy);
    // Map this world-space reference perp into B's local space (using initial identity transform).
    // We'll recompute this properly in the first prepare() call via the pool transforms.
    c->m_refPerpLocalB = c->m_refPerpLocalA;
    return c;
}

void HingeConstraint::setLimits(float minAngle, float maxAngle) {
    m_limitsActive = true;
    m_limitMin = minAngle;
    m_limitMax = maxAngle;
}
void HingeConstraint::clearLimits() { m_limitsActive = false; }

void HingeConstraint::setMotor(float targetSpeed, float maxImpulse) {
    m_motorActive    = true;
    m_motorSpeed     = targetSpeed;
    m_motorMaxImpulse = maxImpulse;
}
void HingeConstraint::clearMotor() { m_motorActive = false; }

void HingeConstraint::prepare(BodyPool& pool, float dt, float baumgarte, float /*slop*/) {
    const auto& da = pool.get(m_bodyA.id());
    const auto& db = pool.get(m_bodyB.id());

    // World-space arms and anchors
    const auto rA   = da.transform.rotation.rotated(m_anchorA);
    const auto rB   = db.transform.rotation.rotated(m_anchorB);
    const auto wA   = da.transform.position + rA;
    const auto wB   = db.transform.position + rB;
    const auto err  = wA - wB;

    // World-space hinge axes
    const auto aA = da.transform.rotation.rotated(m_axisLocalA);
    const auto aB = db.transform.rotation.rotated(m_axisLocalB);

    const vm::Vector3<float> worldAxes[3] = {
        {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}
    };

    // ── Linear rows (ball socket) ───────────────────────────────────────────
    for (int i = 0; i < 3; ++i) {
        auto& r       = m_rows[i];
        const auto& n = worldAxes[i];
        r.J_va = n;
        r.J_wa = cross3(rA, n);
        r.J_vb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        r.J_wb = vm::Vector3<float>(-cross3(rB, n).x(),
                                    -cross3(rB, n).y(),
                                    -cross3(rB, n).z());
        float C = dot3(n, err);
        r.bias  = (baumgarte / dt) * C;
        r.lambdaMin = -1e30f;
        r.lambdaMax =  1e30f;
        computeEffMass(da, db, r);
    }

    // ── Angular rows: keep hinge axes aligned (2 perp axes) ────────────────
    vm::Vector3<float> p1, p2;
    perp3(aA, p1, p2);

    // Angular error perpendicular to hinge axis
    const auto angErr = cross3(aA, aB);  // zero when aligned

    auto& ra3 = m_rows[3];
    ra3.J_va = {0.f, 0.f, 0.f}; ra3.J_vb = {0.f, 0.f, 0.f};
    ra3.J_wa = vm::Vector3<float>(-p1.x(), -p1.y(), -p1.z());
    ra3.J_wb = p1;
    ra3.bias = (baumgarte / dt) * dot3(p1, angErr);
    ra3.lambdaMin = -1e30f; ra3.lambdaMax = 1e30f;
    computeEffMass(da, db, ra3);

    auto& ra4 = m_rows[4];
    ra4.J_va = {0.f, 0.f, 0.f}; ra4.J_vb = {0.f, 0.f, 0.f};
    ra4.J_wa = vm::Vector3<float>(-p2.x(), -p2.y(), -p2.z());
    ra4.J_wb = p2;
    ra4.bias = (baumgarte / dt) * dot3(p2, angErr);
    ra4.lambdaMin = -1e30f; ra4.lambdaMax = 1e30f;
    computeEffMass(da, db, ra4);

    m_rowCount = 5;

    // ── Optional limit ──────────────────────────────────────────────────────
    if (m_limitsActive) {
        // Measure current angle about hinge axis using stored perp reference vectors
        const auto refA = da.transform.rotation.rotated(m_refPerpLocalA);
        const auto refB = db.transform.rotation.rotated(m_refPerpLocalB);
        float cosA = dot3(refA, refB);
        float sinA = dot3(cross3(refA, refB), aA);
        float angle = std::atan2(sinA, cosA);

        auto& rl = m_rows[5];
        rl.J_va = {0.f, 0.f, 0.f}; rl.J_vb = {0.f, 0.f, 0.f};
        rl.J_wa = vm::Vector3<float>(-aA.x(), -aA.y(), -aA.z());
        rl.J_wb = aA;
        rl.bias = 0.f;

        if (angle < m_limitMin) {
            float C = m_limitMin - angle;
            rl.bias = (baumgarte / dt) * C;
            rl.lambdaMin = 0.f; rl.lambdaMax = 1e30f;
            computeEffMass(da, db, rl);
            m_rowCount = 6;
        } else if (angle > m_limitMax) {
            float C = angle - m_limitMax;
            rl.bias = -(baumgarte / dt) * C;
            rl.lambdaMin = -1e30f; rl.lambdaMax = 0.f;
            computeEffMass(da, db, rl);
            m_rowCount = 6;
        } else {
            rl.lambda = 0.f;
            rl.effMass = 0.f;
        }
    }

    // ── Optional motor (overrides limit row if both active) ─────────────────
    if (m_motorActive) {
        auto& rm = m_rows[5];
        rm.J_va = {0.f, 0.f, 0.f}; rm.J_vb = {0.f, 0.f, 0.f};
        rm.J_wa = vm::Vector3<float>(-aA.x(), -aA.y(), -aA.z());
        rm.J_wb = aA;
        rm.bias       = -m_motorSpeed;
        rm.lambdaMin  = -m_motorMaxImpulse;
        rm.lambdaMax  =  m_motorMaxImpulse;
        computeEffMass(da, db, rm);
        m_rowCount = 6;
    }
}

void HingeConstraint::warmStart(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (int i = 0; i < m_rowCount; ++i) warmStartRow(da, db, m_rows[i]);
}

void HingeConstraint::solveVelocity(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (int i = 0; i < m_rowCount; ++i) solveRow(da, db, m_rows[i]);
}

void HingeConstraint::solvePosition(BodyPool& pool, float /*dt*/, float alpha) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());

    // Linear rows (ball socket)
    const auto rA  = da.transform.rotation.rotated(m_anchorA);
    const auto rB  = db.transform.rotation.rotated(m_anchorB);
    const auto err = (da.transform.position + rA) - (db.transform.position + rB);
    const vm::Vector3<float> axes[3] = {{1.f,0.f,0.f},{0.f,1.f,0.f},{0.f,0.f,1.f}};
    for (int i = 0; i < 3; ++i)
        solvePosRow(da, db, m_rows[i], dot3(axes[i], err), alpha);

    // Angular rows: align hinge axes (rows 3-4)
    const auto aA      = da.transform.rotation.rotated(m_axisLocalA);
    const auto aB      = db.transform.rotation.rotated(m_axisLocalB);
    const auto angErr  = cross3(aA, aB);
    vm::Vector3<float> p1, p2;
    perp3(aA, p1, p2);
    solvePosRow(da, db, m_rows[3], dot3(p1, angErr), alpha);
    solvePosRow(da, db, m_rows[4], dot3(p2, angErr), alpha);

    // Limit row (row 5) — only when a violated limit is active (not a motor)
    if (m_limitsActive && !m_motorActive && m_rowCount == 6) {
        const auto aAc  = da.transform.rotation.rotated(m_axisLocalA);
        const auto refA = da.transform.rotation.rotated(m_refPerpLocalA);
        const auto refB = db.transform.rotation.rotated(m_refPerpLocalB);
        float cosA  = dot3(refA, refB);
        float sinA  = dot3(cross3(refA, refB), aAc);
        float angle = std::atan2(sinA, cosA);
        float C = 0.f;
        if      (angle < m_limitMin) C = angle - m_limitMin;
        else if (angle > m_limitMax) C = angle - m_limitMax;
        if (C != 0.f) solvePosRow(da, db, m_rows[5], C, alpha);
    }
}

} // namespace campello::physics
