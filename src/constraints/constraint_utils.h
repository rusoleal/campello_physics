#pragma once

#include <campello_physics/constraint.h>
#include <algorithm>
#include <cmath>

namespace campello::physics::detail {

// ── Math helpers ──────────────────────────────────────────────────────────────

inline float dot3(const vm::Vector3<float>& a, const vm::Vector3<float>& b) {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
}

inline vm::Vector3<float> cross3(const vm::Vector3<float>& a, const vm::Vector3<float>& b) {
    return vm::Vector3<float>(
        a.y()*b.z() - a.z()*b.y(),
        a.z()*b.x() - a.x()*b.z(),
        a.x()*b.y() - a.y()*b.x());
}

inline float lenSq3(const vm::Vector3<float>& v) {
    return v.x()*v.x() + v.y()*v.y() + v.z()*v.z();
}

inline vm::Vector3<float> norm3(const vm::Vector3<float>& v) {
    float l = std::sqrt(lenSq3(v));
    return l > 1e-20f ? v * (1.f / l) : vm::Vector3<float>(1.f, 0.f, 0.f);
}

// Applies the world-space inverse inertia tensor to v:
//   I_world^-1 * v = R * (invI_local ⊙ (R^T * v))
inline vm::Vector3<float> applyInvInertia(const BodyData& d, const vm::Vector3<float>& v) {
    const auto& R   = d.transform.rotation;
    const auto  loc = R.conjugated().rotated(v);
    const auto& inv = d.invInertiaTensorLocal;
    return R.rotated(vm::Vector3<float>(
        loc.x() * inv.x(), loc.y() * inv.y(), loc.z() * inv.z()));
}

// Returns two unit vectors perpendicular to v (and to each other).
inline void perp3(const vm::Vector3<float>& v,
                  vm::Vector3<float>& p1, vm::Vector3<float>& p2) {
    float ax = std::abs(v.x()), ay = std::abs(v.y()), az = std::abs(v.z());
    vm::Vector3<float> ref = (ax <= ay && ax <= az)
        ? vm::Vector3<float>(1.f, 0.f, 0.f)
        : (ay <= az ? vm::Vector3<float>(0.f, 1.f, 0.f)
                    : vm::Vector3<float>(0.f, 0.f, 1.f));
    p1 = norm3(cross3(v, ref));
    p2 = cross3(v, p1);
}

// ── Row helpers ───────────────────────────────────────────────────────────────

// Precomputes effective mass K^-1 = 1 / (J * M^-1 * J^T) for one row.
inline void computeEffMass(const BodyData& da, const BodyData& db, ConstraintRow& r) {
    float K = dot3(r.J_va, r.J_va) * da.invMass
            + dot3(r.J_vb, r.J_vb) * db.invMass
            + dot3(r.J_wa, applyInvInertia(da, r.J_wa))
            + dot3(r.J_wb, applyInvInertia(db, r.J_wb));
    r.effMass = K > 1e-12f ? 1.f / K : 0.f;
}

// Applies one velocity-correction iteration for a single row.
inline void solveRow(BodyData& da, BodyData& db, ConstraintRow& r) {
    float Jv = dot3(r.J_va, da.linearVelocity)  + dot3(r.J_wa, da.angularVelocity)
             + dot3(r.J_vb, db.linearVelocity)  + dot3(r.J_wb, db.angularVelocity);
    float dL   = r.effMass * -(Jv + r.bias);
    float lOld = r.lambda;
    r.lambda   = std::clamp(r.lambda + dL, r.lambdaMin, r.lambdaMax);
    dL = r.lambda - lOld;

    da.linearVelocity  = da.linearVelocity  + r.J_va * (dL * da.invMass);
    da.angularVelocity = da.angularVelocity + applyInvInertia(da, r.J_wa * dL);
    db.linearVelocity  = db.linearVelocity  + r.J_vb * (dL * db.invMass);
    db.angularVelocity = db.angularVelocity + applyInvInertia(db, r.J_wb * dL);
}

// Applies the warm-start impulse at the beginning of a step.
inline void warmStartRow(BodyData& da, BodyData& db, const ConstraintRow& r) {
    const float ws = r.lambda * 0.85f;
    da.linearVelocity  = da.linearVelocity  + r.J_va * (ws * da.invMass);
    da.angularVelocity = da.angularVelocity + applyInvInertia(da, r.J_wa * ws);
    db.linearVelocity  = db.linearVelocity  + r.J_vb * (ws * db.invMass);
    db.angularVelocity = db.angularVelocity + applyInvInertia(db, r.J_wb * ws);
}

// ── Position solve ────────────────────────────────────────────────────────────
// Directly corrects body transforms (position + orientation) for one row.
// C:     current scalar constraint error (same sign convention as the velocity bias).
// alpha: correction gain per call [0, 1], typically 0.2–0.5.
// dt:    fixed timestep.
// Position impulses are NOT accumulated — no warm-starting for position solve.
// Whether this row is a pure angular row (J_va == J_vb == 0)
inline bool isPureAngularRow(const ConstraintRow& r) noexcept {
    return lenSq3(r.J_va) < 1e-12f && lenSq3(r.J_vb) < 1e-12f;
}

// Applies one position-correction step for a pure angular row.
// For linear rows we intentionally omit the angular (lever-arm) correction:
// that term uses stale Jacobians from prepare() and creates unstable torques.
inline void solvePosRow(BodyData& da, BodyData& db, const ConstraintRow& r,
                        float C, float alpha) {
    if (r.effMass == 0.f) return;

    const float lambda = r.effMass * -alpha * C;

    if (isPureAngularRow(r)) {
        // ── Angular-only row: correct orientations ────────────────────────────
        auto applyAng = [](BodyData& d, const vm::Vector3<float>& Jw, float lam) {
            if (d.invMass == 0.f || d.isSleeping) return;
            auto dw = applyInvInertia(d, Jw * lam);
            // Cap to ~11° per call to prevent high-invInertia instability
            constexpr float kMax = 0.2f;
            const float dwSq = dw.x()*dw.x() + dw.y()*dw.y() + dw.z()*dw.z();
            if (dwSq > kMax * kMax) dw = dw * (kMax / std::sqrt(dwSq));

            auto& q = d.transform.rotation;
            const auto omegaQ = vm::Quaternion<float>(dw.x(), dw.y(), dw.z(), 0.f);
            const auto dq     = omegaQ * q * 0.5f;
            q = q + dq;
            const float lenSq = q.data[0]*q.data[0] + q.data[1]*q.data[1] +
                                 q.data[2]*q.data[2] + q.data[3]*q.data[3];
            if (lenSq > 1e-20f) {
                const float invLen = 1.f / std::sqrt(lenSq);
                q = vm::Quaternion<float>(q.data[0]*invLen, q.data[1]*invLen,
                                          q.data[2]*invLen, q.data[3]*invLen);
            } else {
                q = vm::Quaternion<float>::identity();
            }
        };
        applyAng(da, r.J_wa, lambda);
        applyAng(db, r.J_wb, lambda);
    } else {
        // ── Linear row: correct positions only (skip stale lever-arm torque) ──
        auto applyLin = [](BodyData& d, const vm::Vector3<float>& Jv, float lam) {
            if (d.invMass == 0.f || d.isSleeping) return;
            d.transform.position = d.transform.position + Jv * (lam * d.invMass);
        };
        applyLin(da, r.J_va, lambda);
        applyLin(db, r.J_vb, lambda);
    }
}

} // namespace campello::physics::detail
