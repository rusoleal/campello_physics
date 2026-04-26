#include <campello_physics/constraints/d6_constraint.h>
#include "constraint_utils.h"
#include <cmath>

namespace campello::physics {

using namespace detail;

// ── Factory ───────────────────────────────────────────────────────────────────

std::shared_ptr<D6Constraint> D6Constraint::create(
    Body a, const Transform& frameLocalA,
    Body b, const Transform& frameLocalB)
{
    auto c = std::shared_ptr<D6Constraint>(new D6Constraint{});
    c->m_bodyA  = a;
    c->m_bodyB  = b;
    c->m_frameA = frameLocalA;
    c->m_frameB = frameLocalB;
    return c;
}

// ── Configuration ─────────────────────────────────────────────────────────────

void D6Constraint::setMotion(int dof, D6Motion motion) noexcept {
    if (dof >= 0 && dof < 6) m_motion[dof] = motion;
}

D6Motion D6Constraint::getMotion(int dof) const noexcept {
    return (dof >= 0 && dof < 6) ? m_motion[dof] : D6Motion::Free;
}

void D6Constraint::setLimit(int dof, float lower, float upper) noexcept {
    if (dof >= 0 && dof < 6) { m_limitLower[dof] = lower; m_limitUpper[dof] = upper; }
}

void D6Constraint::setDrive(int dof, float targetVelocity, float maxImpulse) noexcept {
    if (dof >= 0 && dof < 6) {
        m_drive[dof].targetVelocity = targetVelocity;
        m_drive[dof].maxImpulse     = maxImpulse;
    }
}

void D6Constraint::clearDrive(int dof) noexcept {
    if (dof >= 0 && dof < 6) m_drive[dof] = {};
}

// ── prepare ───────────────────────────────────────────────────────────────────

void D6Constraint::prepare(BodyPool& pool, float dt, float baumgarte, float slop) {
    const auto& da = pool.get(m_bodyA.id());
    const auto& db = pool.get(m_bodyB.id());

    // World-space constraint frame quaternions
    const auto qFAW = da.transform.rotation * m_frameA.rotation;
    const auto qFBW = db.transform.rotation * m_frameB.rotation;

    // Lever arms: body CoM → constraint frame origin, in world space
    const auto rA = da.transform.rotation.rotated(m_frameA.position);
    const auto rB = db.transform.rotation.rotated(m_frameB.position);

    // World-space constraint frame origins
    const auto wA = da.transform.position + rA;
    const auto wB = db.transform.position + rB;

    // Frame A world-space axes (shared by linear and angular DOF)
    const vm::Vector3<float> axis[3] = {
        qFAW.rotated(vm::Vector3<float>(1.f, 0.f, 0.f)),
        qFAW.rotated(vm::Vector3<float>(0.f, 1.f, 0.f)),
        qFAW.rotated(vm::Vector3<float>(0.f, 0.f, 1.f))
    };

    // Relative translation expressed in frame A axes (A minus B, consistent with Jacobian sign)
    const auto relPos = wA - wB;
    const float linPos[3] = {
        dot3(relPos, axis[0]),
        dot3(relPos, axis[1]),
        dot3(relPos, axis[2])
    };

    // Lazy-init angular reference: capture the relative frame orientation at creation
    if (!m_qRefInit) {
        m_qRef     = qFAW.conjugated() * qFBW;
        m_qRefInit = true;
    }

    // Current relative rotation in frame A space, error relative to reference
    // qErr = qCurrent * qRef^-1  (right-multiply convention, consistent with SliderConstraint)
    const auto qCurrent  = qFAW.conjugated() * qFBW;
    const auto qRefConj  = vm::Quaternion<float>(
        -m_qRef.data[0], -m_qRef.data[1], -m_qRef.data[2], m_qRef.data[3]);
    auto qErr = qCurrent * qRefConj;
    if (qErr.data[3] < 0.f) {   // shortest-path disambiguation
        qErr.data[0]=-qErr.data[0]; qErr.data[1]=-qErr.data[1];
        qErr.data[2]=-qErr.data[2]; qErr.data[3]=-qErr.data[3];
    }
    // Angular error in frame A local axes (small-angle: angErr ≈ 2 * qErr.xyz)
    const float angErr[3] = {
        qErr.data[0] * 2.f,
        qErr.data[1] * 2.f,
        qErr.data[2] * 2.f
    };

    // ── Linear rows (DOF 0-2) ────────────────────────────────────────────────
    for (int i = 0; i < 3; ++i) {
        auto& r       = m_rows[i];
        const auto& n = axis[i];

        r.J_va = n;
        r.J_wa = cross3(rA, n);
        r.J_vb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        r.J_wb = vm::Vector3<float>(-cross3(rB, n).x(),
                                    -cross3(rB, n).y(),
                                    -cross3(rB, n).z());

        if (m_drive[i].maxImpulse > 0.f) {
            // Jacobian measures vA-vB; positive bias drives vB-vA = targetVelocity.
            r.bias      = +m_drive[i].targetVelocity;
            r.lambdaMin = -m_drive[i].maxImpulse;
            r.lambdaMax =  m_drive[i].maxImpulse;
            computeEffMass(da, db, r);
        } else if (m_motion[i] == D6Motion::Locked) {
            float C = linPos[i];
            r.bias      = (baumgarte / dt) * (C > slop ? C-slop : (C < -slop ? C+slop : 0.f));
            r.lambdaMin = -1e30f;
            r.lambdaMax =  1e30f;
            computeEffMass(da, db, r);
        } else if (m_motion[i] == D6Motion::Limited) {
            float C = linPos[i];
            if (C < m_limitLower[i]) {
                r.bias      = (baumgarte / dt) * (m_limitLower[i] - C);
                r.lambdaMin = 0.f;   r.lambdaMax = 1e30f;
                computeEffMass(da, db, r);
            } else if (C > m_limitUpper[i]) {
                r.bias      = -(baumgarte / dt) * (C - m_limitUpper[i]);
                r.lambdaMin = -1e30f; r.lambdaMax = 0.f;
                computeEffMass(da, db, r);
            } else {
                r.effMass = 0.f;  r.lambda = 0.f;
            }
        } else {  // Free
            r.effMass = 0.f;  r.lambda = 0.f;
        }
    }

    // ── Angular rows (DOF 3-5) ───────────────────────────────────────────────
    for (int i = 0; i < 3; ++i) {
        const int dof = i + 3;
        auto& r       = m_rows[dof];
        const auto& n = axis[i];

        r.J_va = {0.f, 0.f, 0.f};
        r.J_vb = {0.f, 0.f, 0.f};
        r.J_wa = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        r.J_wb = n;

        if (m_drive[dof].maxImpulse > 0.f) {
            r.bias      = -m_drive[dof].targetVelocity;
            r.lambdaMin = -m_drive[dof].maxImpulse;
            r.lambdaMax =  m_drive[dof].maxImpulse;
            computeEffMass(da, db, r);
        } else if (m_motion[dof] == D6Motion::Locked) {
            r.bias      = (baumgarte / dt) * angErr[i];
            r.lambdaMin = -1e30f;
            r.lambdaMax =  1e30f;
            computeEffMass(da, db, r);
        } else if (m_motion[dof] == D6Motion::Limited) {
            float C = angErr[i];
            if (C < m_limitLower[dof]) {
                r.bias      = (baumgarte / dt) * (m_limitLower[dof] - C);
                r.lambdaMin = 0.f;    r.lambdaMax = 1e30f;
                computeEffMass(da, db, r);
            } else if (C > m_limitUpper[dof]) {
                r.bias      = -(baumgarte / dt) * (C - m_limitUpper[dof]);
                r.lambdaMin = -1e30f; r.lambdaMax = 0.f;
                computeEffMass(da, db, r);
            } else {
                r.effMass = 0.f;  r.lambda = 0.f;
            }
        } else {  // Free
            r.effMass = 0.f;  r.lambda = 0.f;
        }
    }
}

// ── warmStart / solveVelocity ─────────────────────────────────────────────────

void D6Constraint::warmStart(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (int i = 0; i < 6; ++i) warmStartRow(da, db, m_rows[i]);
}

void D6Constraint::solveVelocity(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (int i = 0; i < 6; ++i) solveRow(da, db, m_rows[i]);
}

void D6Constraint::solvePosition(BodyPool& pool, float /*dt*/, float alpha) {
    if (!m_qRefInit) return;
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());

    const auto qFAW  = da.transform.rotation * m_frameA.rotation;
    const auto qFBW  = db.transform.rotation * m_frameB.rotation;
    const auto rA    = da.transform.rotation.rotated(m_frameA.position);
    const auto rB    = db.transform.rotation.rotated(m_frameB.position);
    const auto relP  = (da.transform.position + rA) - (db.transform.position + rB);

    const vm::Vector3<float> axis[3] = {
        qFAW.rotated(vm::Vector3<float>(1.f,0.f,0.f)),
        qFAW.rotated(vm::Vector3<float>(0.f,1.f,0.f)),
        qFAW.rotated(vm::Vector3<float>(0.f,0.f,1.f))
    };
    const float linPos[3] = { dot3(relP,axis[0]), dot3(relP,axis[1]), dot3(relP,axis[2]) };

    const auto qCurrent = qFAW.conjugated() * qFBW;
    const auto qRefConj = vm::Quaternion<float>(
        -m_qRef.data[0], -m_qRef.data[1], -m_qRef.data[2], m_qRef.data[3]);
    auto qErr = qCurrent * qRefConj;
    if (qErr.data[3] < 0.f) {
        qErr.data[0]=-qErr.data[0]; qErr.data[1]=-qErr.data[1];
        qErr.data[2]=-qErr.data[2]; qErr.data[3]=-qErr.data[3];
    }
    const float angErr[3] = { qErr.data[0]*2.f, qErr.data[1]*2.f, qErr.data[2]*2.f };

    for (int i = 0; i < 3; ++i) {
        if (m_drive[i].maxImpulse > 0.f || m_motion[i] == D6Motion::Free) continue;
        float C = linPos[i];
        if (m_motion[i] == D6Motion::Limited) {
            if      (C < m_limitLower[i]) C = C - m_limitLower[i];
            else if (C > m_limitUpper[i]) C = C - m_limitUpper[i];
            else continue;
        }
        solvePosRow(da, db, m_rows[i], C, alpha);
    }
    for (int i = 0; i < 3; ++i) {
        const int dof = i + 3;
        if (m_drive[dof].maxImpulse > 0.f || m_motion[dof] == D6Motion::Free) continue;
        float C = angErr[i];
        if (m_motion[dof] == D6Motion::Limited) {
            if      (C < m_limitLower[dof]) C = C - m_limitLower[dof];
            else if (C > m_limitUpper[dof]) C = C - m_limitUpper[dof];
            else continue;
        }
        solvePosRow(da, db, m_rows[dof], C, alpha);
    }
}

} // namespace campello::physics
