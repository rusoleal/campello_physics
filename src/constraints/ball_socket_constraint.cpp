#include <campello_physics/constraints/ball_socket_constraint.h>
#include "constraint_utils.h"

namespace campello::physics {

using namespace detail;

std::shared_ptr<BallSocketConstraint> BallSocketConstraint::create(
    Body a, const vm::Vector3<float>& anchorLocalA,
    Body b, const vm::Vector3<float>& anchorLocalB)
{
    auto c = std::shared_ptr<BallSocketConstraint>(new BallSocketConstraint{});
    c->m_bodyA   = a;
    c->m_bodyB   = b;
    c->m_anchorA = anchorLocalA;
    c->m_anchorB = anchorLocalB;
    return c;
}

void BallSocketConstraint::prepare(BodyPool& pool, float dt, float baumgarte, float slop) {
    const auto& da = pool.get(m_bodyA.id());
    const auto& db = pool.get(m_bodyB.id());

    // World-space arms from body center to anchor
    const auto rA = da.transform.rotation.rotated(m_anchorA);
    const auto rB = db.transform.rotation.rotated(m_anchorB);

    // World anchor positions
    const auto wA = da.transform.position + rA;
    const auto wB = db.transform.position + rB;

    // Position error (3 components)
    const auto err = wA - wB;

    // World constraint axes (x, y, z)
    const vm::Vector3<float> axes[3] = {
        {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}
    };

    for (int i = 0; i < 3; ++i) {
        auto& r    = m_rows[i];
        const auto& n = axes[i];
        r.J_va = n;
        r.J_wa = cross3(rA, n);
        r.J_vb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        r.J_wb = vm::Vector3<float>(-cross3(rB, n).x(), -cross3(rB, n).y(), -cross3(rB, n).z());

        float C = dot3(n, err);
        float absC = C < 0.f ? -C : C;
        r.bias = (baumgarte / dt) * (C > slop ? C - slop : (C < -slop ? C + slop : 0.f));
        (void)absC;

        r.lambdaMin = -1e30f;
        r.lambdaMax =  1e30f;
        computeEffMass(da, db, r);
    }
}

void BallSocketConstraint::warmStart(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (auto& r : m_rows) warmStartRow(da, db, r);
}

void BallSocketConstraint::solveVelocity(BodyPool& pool) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    for (auto& r : m_rows) solveRow(da, db, r);
}

void BallSocketConstraint::solvePosition(BodyPool& pool, float /*dt*/, float alpha) {
    auto& da = pool.get(m_bodyA.id());
    auto& db = pool.get(m_bodyB.id());
    const auto rA  = da.transform.rotation.rotated(m_anchorA);
    const auto rB  = db.transform.rotation.rotated(m_anchorB);
    const auto err = (da.transform.position + rA) - (db.transform.position + rB);
    const vm::Vector3<float> axes[3] = {{1.f,0.f,0.f},{0.f,1.f,0.f},{0.f,0.f,1.f}};
    for (int i = 0; i < 3; ++i)
        solvePosRow(da, db, m_rows[i], dot3(axes[i], err), alpha);
}

} // namespace campello::physics
