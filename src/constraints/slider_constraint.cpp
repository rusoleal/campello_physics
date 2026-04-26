#include <campello_physics/constraints/slider_constraint.h>
#include "constraint_utils.h"

namespace campello::physics {

using namespace detail;

std::shared_ptr<SliderConstraint> SliderConstraint::create(
    Body a, const vm::Vector3<float>& anchorLocalA, const vm::Vector3<float>& axisLocalA,
    Body b, const vm::Vector3<float>& anchorLocalB, const vm::Vector3<float>& axisLocalB)
{
    auto c = std::shared_ptr<SliderConstraint>(new SliderConstraint{});
    c->m_bodyA      = a;
    c->m_bodyB      = b;
    c->m_anchorA    = anchorLocalA;
    c->m_anchorB    = anchorLocalB;
    c->m_axisLocalA = norm3(axisLocalA);
    c->m_axisLocalB = norm3(axisLocalB);
    // Reference relative quaternion initialised lazily.
    c->m_qRefInA = vm::Quaternion<float>::identity();
    return c;
}

void SliderConstraint::setLimits(float minDist, float maxDist) {
    m_limitsActive = true;
    m_limitMin = minDist;
    m_limitMax = maxDist;
}
void SliderConstraint::clearLimits() { m_limitsActive = false; }

void SliderConstraint::setMotor(float targetSpeed, float maxImpulse) {
    m_motorActive    = true;
    m_motorSpeed     = targetSpeed;
    m_motorMaxImpulse = maxImpulse;
}
void SliderConstraint::clearMotor() { m_motorActive = false; }

void SliderConstraint::prepare(BodyPool& pool, float dt, float baumgarte, float slop) {
    const auto& da = pool.get(m_bodyA.id());
    const auto& db = pool.get(m_bodyB.id());

    // Lazy init for angular reference
    {
        static const vm::Quaternion<float> kI = vm::Quaternion<float>::identity();
        bool isIdentity = (m_qRefInA.data[0] == kI.data[0]
                        && m_qRefInA.data[1] == kI.data[1]
                        && m_qRefInA.data[2] == kI.data[2]
                        && m_qRefInA.data[3] == kI.data[3]);
        if (isIdentity)
            m_qRefInA = da.transform.rotation.conjugated() * db.transform.rotation;
    }

    // World-space arms and anchors
    const auto rA   = da.transform.rotation.rotated(m_anchorA);
    const auto rB   = db.transform.rotation.rotated(m_anchorB);
    const auto wA   = da.transform.position + rA;
    const auto wB   = db.transform.position + rB;
    const auto sep  = wA - wB;

    // World-space slide axis (from body A)
    const auto axis = da.transform.rotation.rotated(m_axisLocalA);

    // Two perpendicular axes to slide axis
    vm::Vector3<float> p1, p2;
    perp3(axis, p1, p2);

    // ── Linear rows: constrain perpendicular displacement ──────────────────
    const vm::Vector3<float> perps[2] = {p1, p2};
    for (int i = 0; i < 2; ++i) {
        auto& r       = m_rows[i];
        const auto& n = perps[i];
        r.J_va = n;
        r.J_wa = cross3(rA, n);
        r.J_vb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        r.J_wb = vm::Vector3<float>(-cross3(rB, n).x(),
                                    -cross3(rB, n).y(),
                                    -cross3(rB, n).z());
        float C = dot3(n, sep);
        r.bias  = (baumgarte / dt) * (C > slop ? C - slop : (C < -slop ? C + slop : 0.f));
        r.lambdaMin = -1e30f; r.lambdaMax = 1e30f;
        computeEffMass(da, db, r);
    }

    // ── Angular rows: fix all rotation (3 rows) ────────────────────────────
    const auto qRel  = da.transform.rotation.conjugated() * db.transform.rotation;
    const auto qRefC = vm::Quaternion<float>(
        -m_qRefInA.data[0], -m_qRefInA.data[1], -m_qRefInA.data[2], m_qRefInA.data[3]);
    auto qErr = qRel * qRefC;
    if (qErr.data[3] < 0.f) {
        qErr.data[0]=-qErr.data[0]; qErr.data[1]=-qErr.data[1];
        qErr.data[2]=-qErr.data[2]; qErr.data[3]=-qErr.data[3];
    }
    const auto angErrLocal = vm::Vector3<float>(
        qErr.data[0]*2.f, qErr.data[1]*2.f, qErr.data[2]*2.f);
    const auto angErrWorld = da.transform.rotation.rotated(angErrLocal);

    const vm::Vector3<float> worldAxes[3] = {
        {1.f,0.f,0.f},{0.f,1.f,0.f},{0.f,0.f,1.f}
    };
    for (int i = 0; i < 3; ++i) {
        auto& r       = m_rows[2 + i];
        const auto& n = worldAxes[i];
        r.J_va = {0.f,0.f,0.f}; r.J_vb = {0.f,0.f,0.f};
        r.J_wa = vm::Vector3<float>(-n.x(),-n.y(),-n.z());
        r.J_wb = n;
        float C = i == 0 ? angErrWorld.x() : (i==1 ? angErrWorld.y() : angErrWorld.z());
        r.bias  = (baumgarte / dt) * C;
        r.lambdaMin = -1e30f; r.lambdaMax = 1e30f;
        computeEffMass(da, db, r);
    }

    m_rowCount = 5;

    // ── Optional limit / motor (row 5) ──────────────────────────────────────
    if (m_limitsActive) {
        float d = dot3(axis, sep);  // current displacement along axis
        auto& rl = m_rows[5];
        rl.J_va = axis;
        rl.J_wa = cross3(rA, axis);
        rl.J_vb = vm::Vector3<float>(-axis.x(),-axis.y(),-axis.z());
        rl.J_wb = vm::Vector3<float>(-cross3(rB, axis).x(),
                                     -cross3(rB, axis).y(),
                                     -cross3(rB, axis).z());
        rl.bias = 0.f;
        if (d < m_limitMin) {
            rl.bias = (baumgarte / dt) * (m_limitMin - d);
            rl.lambdaMin = 0.f; rl.lambdaMax = 1e30f;
            computeEffMass(da, db, rl);
            m_rowCount = 6;
        } else if (d > m_limitMax) {
            rl.bias = -(baumgarte / dt) * (d - m_limitMax);
            rl.lambdaMin = -1e30f; rl.lambdaMax = 0.f;
            computeEffMass(da, db, rl);
            m_rowCount = 6;
        } else {
            rl.lambda = 0.f; rl.effMass = 0.f;
        }
    }

    if (m_motorActive) {
        auto& rm = m_rows[5];
        rm.J_va = axis;
        rm.J_wa = cross3(rA, axis);
        rm.J_vb = vm::Vector3<float>(-axis.x(),-axis.y(),-axis.z());
        rm.J_wb = vm::Vector3<float>(-cross3(rB, axis).x(),
                                     -cross3(rB, axis).y(),
                                     -cross3(rB, axis).z());
        rm.bias       = -m_motorSpeed;
        rm.lambdaMin  = -m_motorMaxImpulse;
        rm.lambdaMax  =  m_motorMaxImpulse;
        computeEffMass(da, db, rm);
        m_rowCount = 6;
    }
}

void SliderConstraint::warmStart(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (int i = 0; i < m_rowCount; ++i) warmStartRow(da, db, m_rows[i]);
}

void SliderConstraint::solveVelocity(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (int i = 0; i < m_rowCount; ++i) solveRow(da, db, m_rows[i]);
}

void SliderConstraint::solvePosition(BodyPool& pool, float /*dt*/, float alpha) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());

    const auto rA   = da.transform.rotation.rotated(m_anchorA);
    const auto rB   = db.transform.rotation.rotated(m_anchorB);
    const auto sep  = (da.transform.position + rA) - (db.transform.position + rB);
    const auto axis = da.transform.rotation.rotated(m_axisLocalA);

    // Perpendicular linear rows (0-1)
    vm::Vector3<float> p1, p2;
    perp3(axis, p1, p2);
    solvePosRow(da, db, m_rows[0], dot3(p1, sep), alpha);
    solvePosRow(da, db, m_rows[1], dot3(p2, sep), alpha);

    // Angular rows (2-4): maintain relative rotation
    const auto qRel  = da.transform.rotation.conjugated() * db.transform.rotation;
    const auto qRefC = vm::Quaternion<float>(
        -m_qRefInA.data[0], -m_qRefInA.data[1], -m_qRefInA.data[2], m_qRefInA.data[3]);
    auto qErr = qRel * qRefC;
    if (qErr.data[3] < 0.f) {
        qErr.data[0]=-qErr.data[0]; qErr.data[1]=-qErr.data[1];
        qErr.data[2]=-qErr.data[2]; qErr.data[3]=-qErr.data[3];
    }
    const auto errLocal = vm::Vector3<float>(qErr.data[0]*2.f, qErr.data[1]*2.f, qErr.data[2]*2.f);
    const auto errWorld = da.transform.rotation.rotated(errLocal);
    for (int i = 0; i < 3; ++i) {
        float C = i==0 ? errWorld.x() : (i==1 ? errWorld.y() : errWorld.z());
        solvePosRow(da, db, m_rows[2+i], C, alpha);
    }

    // Limit row (row 5) — violated limits only, not motors
    if (m_limitsActive && !m_motorActive && m_rowCount == 6) {
        const auto axisC = da.transform.rotation.rotated(m_axisLocalA);
        const auto rAc   = da.transform.rotation.rotated(m_anchorA);
        const auto rBc   = db.transform.rotation.rotated(m_anchorB);
        const auto sepC  = (da.transform.position + rAc) - (db.transform.position + rBc);
        float d = dot3(axisC, sepC);
        float C = 0.f;
        if      (d < m_limitMin) C = d - m_limitMin;
        else if (d > m_limitMax) C = d - m_limitMax;
        if (C != 0.f) solvePosRow(da, db, m_rows[5], C, alpha);
    }
}

} // namespace campello::physics
