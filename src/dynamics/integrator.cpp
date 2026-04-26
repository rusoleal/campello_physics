#include <campello_physics/integrator.h>
#include <cmath>

namespace campello::physics {

void integrate(BodyPool& pool, const IntegratorSettings& settings, float dt) {
    const float linThreshSq  = settings.sleepLinearThreshold  * settings.sleepLinearThreshold;
    const float angThreshSq  = settings.sleepAngularThreshold * settings.sleepAngularThreshold;

    pool.forEach([&](uint32_t /*id*/, BodyData& d) {
        if (d.type != BodyType::Dynamic) return;
        if (d.isSleeping) return;

        // Damping: attenuate velocities before forces
        const float linDamp = 1.f - d.linearDamping  * dt;
        const float angDamp = 1.f - d.angularDamping * dt;
        d.linearVelocity  = d.linearVelocity  * (linDamp > 0.f ? linDamp : 0.f);
        d.angularVelocity = d.angularVelocity * (angDamp > 0.f ? angDamp : 0.f);

        // Linear integration: v += (F/m + g) * dt,  x += v * dt
        const auto linearAcc = d.forceAccum * d.invMass + settings.gravity;
        d.linearVelocity = d.linearVelocity + linearAcc * dt;
        d.transform.position = d.transform.position + d.linearVelocity * dt;

        // Angular integration: ω += I_world^-1 * τ * dt
        //   I_world^-1 * v = R * (invI_local ⊙ (R^T * v))
        {
            const auto& R   = d.transform.rotation;
            const auto  tLocal = R.conjugated().rotated(d.torqueAccum);
            const auto& inv = d.invInertiaTensorLocal;
            const auto  scaled = vm::Vector3<float>(
                tLocal.x() * inv.x(), tLocal.y() * inv.y(), tLocal.z() * inv.z());
            d.angularVelocity = d.angularVelocity + R.rotated(scaled) * dt;
        }

        // Integrate orientation: q += 0.5 * dt * [ω, 0] * q,  then normalize
        {
            auto& q = d.transform.rotation;
            const auto& w = d.angularVelocity;
            const auto omegaQ = vm::Quaternion<float>(w.x(), w.y(), w.z(), 0.f);
            const auto dq     = omegaQ * q * (0.5f * dt);
            q = q + dq;
            // Manual normalize to avoid triggering -Wsign-compare in Vec::normalize()
            const float invLen = 1.f / std::sqrt(
                q.data[0]*q.data[0] + q.data[1]*q.data[1] +
                q.data[2]*q.data[2] + q.data[3]*q.data[3]);
            q = vm::Quaternion<float>(
                q.data[0]*invLen, q.data[1]*invLen,
                q.data[2]*invLen, q.data[3]*invLen);
        }

        // Clear accumulators
        d.forceAccum  = { 0.f, 0.f, 0.f };
        d.torqueAccum = { 0.f, 0.f, 0.f };

        // Accumulate per-body sleep frames; the island manager makes the final
        // sleep/wake decision so all bodies in a connected component sleep together.
        const float vSq = d.linearVelocity.x()  * d.linearVelocity.x()
                        + d.linearVelocity.y()  * d.linearVelocity.y()
                        + d.linearVelocity.z()  * d.linearVelocity.z();
        const float wSq = d.angularVelocity.x() * d.angularVelocity.x()
                        + d.angularVelocity.y() * d.angularVelocity.y()
                        + d.angularVelocity.z() * d.angularVelocity.z();

        if (vSq <= linThreshSq && wSq <= angThreshSq) {
            ++d.sleepFrames;
        } else {
            d.sleepFrames = 0;
        }
    });
}

} // namespace campello::physics
