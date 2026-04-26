#include <campello_physics/constraints/fixed_constraint.h>
#include "constraint_utils.h"

namespace campello::physics {

using namespace detail;

std::shared_ptr<FixedConstraint> FixedConstraint::create(Body a, Body b) {
    auto c = std::shared_ptr<FixedConstraint>(new FixedConstraint{});
    c->m_bodyA = a;
    c->m_bodyB = b;
    // The anchors are the body origins in local space (zeros).
    // Store the reference relative quaternion at creation time.
    // We need the pool to read transforms — defer to first prepare().
    // Use a sentinel: identity means "not yet initialised".
    c->m_qRefInA = vm::Quaternion<float>::identity();
    c->m_anchorA = {0.f, 0.f, 0.f};
    c->m_anchorB = {0.f, 0.f, 0.f};
    return c;
}

// Called once when the constraint is first prepared (qRefInA is still identity).
static void initialise(FixedConstraint* /*self*/, const BodyData& da, const BodyData& db,
                       vm::Vector3<float>& anchorA, vm::Vector3<float>& anchorB,
                       vm::Quaternion<float>& qRefInA)
{
    // Anchors: offset of B origin from A origin, expressed in A local space.
    const auto offset = db.transform.position - da.transform.position;
    anchorA = {0.f, 0.f, 0.f};
    anchorB = da.transform.rotation.conjugated().rotated(-offset);
    // ^ local vector in B-space that, when rotated back by B, gives -offset
    // Actually: we want world(anchorA) == world(anchorB).
    // world(anchorA) = posA + RA * anchorA_local = posA
    // world(anchorB) = posB + RB * anchorB_local = posB + RB * anchorB_local
    // So anchorB_local = RB^-1 * (posA - posB)
    anchorB = db.transform.rotation.conjugated().rotated(
        da.transform.position - db.transform.position);

    qRefInA = da.transform.rotation.conjugated() * db.transform.rotation;
}

void FixedConstraint::prepare(BodyPool& pool, float dt, float baumgarte, float slop) {
    const auto& da = pool.get(m_bodyA.id());
    const auto& db = pool.get(m_bodyB.id());

    // Lazy initialisation on first frame
    static const vm::Quaternion<float> kIdentity = vm::Quaternion<float>::identity();
    const bool needsInit = (m_qRefInA.data[0] == kIdentity.data[0]
                         && m_qRefInA.data[1] == kIdentity.data[1]
                         && m_qRefInA.data[2] == kIdentity.data[2]
                         && m_qRefInA.data[3] == kIdentity.data[3]
                         && m_anchorA.x() == 0.f && m_anchorA.y() == 0.f && m_anchorA.z() == 0.f
                         && m_anchorB.x() == 0.f && m_anchorB.y() == 0.f && m_anchorB.z() == 0.f);
    if (needsInit)
        initialise(this, da, db, m_anchorA, m_anchorB, m_qRefInA);

    // ── Linear rows (ball socket) ───────────────────────────────────────────
    const auto rA  = da.transform.rotation.rotated(m_anchorA);
    const auto rB  = db.transform.rotation.rotated(m_anchorB);
    const auto wA  = da.transform.position + rA;
    const auto wB  = db.transform.position + rB;
    const auto err = wA - wB;

    const vm::Vector3<float> axes[3] = {
        {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}
    };

    for (int i = 0; i < 3; ++i) {
        auto& r       = m_rows[i];
        const auto& n = axes[i];
        r.J_va = n;
        r.J_wa = cross3(rA, n);
        r.J_vb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        r.J_wb = vm::Vector3<float>(-cross3(rB, n).x(),
                                    -cross3(rB, n).y(),
                                    -cross3(rB, n).z());
        float C = dot3(n, err);
        r.bias  = (baumgarte / dt) * (C > slop ? C - slop : (C < -slop ? C + slop : 0.f));
        r.lambdaMin = -1e30f;
        r.lambdaMax =  1e30f;
        computeEffMass(da, db, r);
    }

    // ── Angular rows ────────────────────────────────────────────────────────
    // q_err = (q_A_conj * q_B) * q_ref_conj   (error in body A frame)
    const auto qRel  = da.transform.rotation.conjugated() * db.transform.rotation;
    const auto qRefC = vm::Quaternion<float>(
        -m_qRefInA.data[0], -m_qRefInA.data[1], -m_qRefInA.data[2], m_qRefInA.data[3]);
    auto qErr = qRel * qRefC;
    if (qErr.data[3] < 0.f) {
        qErr.data[0] = -qErr.data[0]; qErr.data[1] = -qErr.data[1];
        qErr.data[2] = -qErr.data[2]; qErr.data[3] = -qErr.data[3];
    }
    // Linearised angular error in world frame
    const auto errLocal = vm::Vector3<float>(
        qErr.data[0] * 2.f, qErr.data[1] * 2.f, qErr.data[2] * 2.f);
    const auto errWorld = da.transform.rotation.rotated(errLocal);

    for (int i = 0; i < 3; ++i) {
        auto& r       = m_rows[3 + i];
        const auto& n = axes[i];
        r.J_va = {0.f, 0.f, 0.f};
        r.J_vb = {0.f, 0.f, 0.f};
        r.J_wa = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        r.J_wb = n;
        float C = i == 0 ? errWorld.x() : (i == 1 ? errWorld.y() : errWorld.z());
        r.bias  = (baumgarte / dt) * C;
        r.lambdaMin = -1e30f;
        r.lambdaMax =  1e30f;
        computeEffMass(da, db, r);
    }
}

void FixedConstraint::warmStart(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (auto& r : m_rows) warmStartRow(da, db, r);
}

void FixedConstraint::solveVelocity(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (auto& r : m_rows) solveRow(da, db, r);
}

void FixedConstraint::solvePosition(BodyPool& pool, float /*dt*/, float alpha) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());

    // Linear error: world anchor separation
    const auto rA  = da.transform.rotation.rotated(m_anchorA);
    const auto rB  = db.transform.rotation.rotated(m_anchorB);
    const auto err = (da.transform.position + rA) - (db.transform.position + rB);

    const vm::Vector3<float> axes[3] = {{1.f,0.f,0.f},{0.f,1.f,0.f},{0.f,0.f,1.f}};
    for (int i = 0; i < 3; ++i)
        solvePosRow(da, db, m_rows[i], dot3(axes[i], err), alpha);

    // Angular error: same quaternion convention as prepare()
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
        float C = i == 0 ? errWorld.x() : (i == 1 ? errWorld.y() : errWorld.z());
        solvePosRow(da, db, m_rows[3+i], C, alpha);
    }
}

} // namespace campello::physics
