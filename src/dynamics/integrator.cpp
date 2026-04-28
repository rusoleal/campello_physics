#include <campello_physics/integrator.h>
#include <cmath>

#if defined(__SSE__) || defined(__AVX__)
    #include <immintrin.h>
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
    #include <arm_neon.h>
#endif

namespace campello::physics {

namespace {

inline void updateInvInertiaWorld(BodyData& d) {
    const auto& q = d.transform.rotation;
    float x = q.x(), y = q.y(), z = q.z(), w = q.w();
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;
    float r00 = 1.f - 2.f*(yy + zz);
    float r01 = 2.f*(xy - wz);
    float r02 = 2.f*(xz + wy);
    float r10 = 2.f*(xy + wz);
    float r11 = 1.f - 2.f*(xx + zz);
    float r12 = 2.f*(yz - wx);
    float r20 = 2.f*(xz - wy);
    float r21 = 2.f*(yz + wx);
    float r22 = 1.f - 2.f*(xx + yy);
    float ix = d.invInertiaTensorLocal.x();
    float iy = d.invInertiaTensorLocal.y();
    float iz = d.invInertiaTensorLocal.z();
    auto& m = d.invInertiaTensorWorld.data;
    m[0] = r00*r00*ix + r01*r01*iy + r02*r02*iz;
    m[1] = r00*r10*ix + r01*r11*iy + r02*r12*iz;
    m[2] = r00*r20*ix + r01*r21*iy + r02*r22*iz;
    m[3] = r10*r00*ix + r11*r01*iy + r12*r02*iz;
    m[4] = r10*r10*ix + r11*r11*iy + r12*r12*iz;
    m[5] = r10*r20*ix + r11*r21*iy + r12*r22*iz;
    m[6] = r20*r00*ix + r21*r01*iy + r22*r02*iz;
    m[7] = r20*r10*ix + r21*r11*iy + r22*r12*iz;
    m[8] = r20*r20*ix + r21*r21*iy + r22*r22*iz;
}

inline float fastRsqrt(float x) {
#if defined(__SSE__) || defined(__AVX__)
    __m128 xi = _mm_set_ss(x);
    __m128 r  = _mm_rsqrt_ss(xi);
    // One Newton-Raphson iteration: r *= 1.5f - 0.5f * x * r * r
    __m128 halfx = _mm_mul_ss(xi, _mm_set_ss(0.5f));
    __m128 t     = _mm_mul_ss(halfx, _mm_mul_ss(r, r));
    __m128 res   = _mm_mul_ss(r, _mm_sub_ss(_mm_set_ss(1.5f), t));
    return _mm_cvtss_f32(res);
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
    float32x4_t xv = vdupq_n_f32(x);
    float32x4_t r  = vrsqrteq_f32(xv);
    float32x4_t r2 = vmulq_f32(r, r);
    float32x4_t t  = vmulq_f32(xv, r2);
    float32x4_t res = vmulq_f32(r, vmulq_f32(vdupq_n_f32(0.5f), vsubq_f32(vdupq_n_f32(3.0f), t)));
    return vgetq_lane_f32(res, 0);
#else
    return 1.0f / std::sqrt(x);
#endif
}

inline void integrateOneBody(BodyData& d, const IntegratorSettings& settings, float dt,
                             float linThreshSq, float angThreshSq) {
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

    // Compute velocity magnitudes for sleep check and angular-work gate
    const float vSq = d.linearVelocity.x()  * d.linearVelocity.x()
                    + d.linearVelocity.y()  * d.linearVelocity.y()
                    + d.linearVelocity.z()  * d.linearVelocity.z();
    const float wSq = d.angularVelocity.x() * d.angularVelocity.x()
                    + d.angularVelocity.y() * d.angularVelocity.y()
                    + d.angularVelocity.z() * d.angularVelocity.z();
    const float tSq = d.torqueAccum.x() * d.torqueAccum.x()
                    + d.torqueAccum.y() * d.torqueAccum.y()
                    + d.torqueAccum.z() * d.torqueAccum.z();

    // Angular integration + orientation update only when there is actual rotation
    if (wSq > 1e-12f || tSq > 1e-12f) {
        // Angular integration: ω += I_world^-1 * τ * dt
        const auto& R   = d.transform.rotation;
        const auto  tLocal = R.conjugated().rotated(d.torqueAccum);
        const auto& inv = d.invInertiaTensorLocal;
        const auto  scaled = vm::Vector3<float>(
            tLocal.x() * inv.x(), tLocal.y() * inv.y(), tLocal.z() * inv.z());
        d.angularVelocity = d.angularVelocity + R.rotated(scaled) * dt;

        // Integrate orientation: q += 0.5 * dt * [ω, 0] * q,  then normalize
        auto& q = d.transform.rotation;
        const auto& w = d.angularVelocity;
        const auto omegaQ = vm::Quaternion<float>(w.x(), w.y(), w.z(), 0.f);
        const auto dq     = omegaQ * q * (0.5f * dt);
        q = q + dq;

        const float lenSqr = q.data[0]*q.data[0] + q.data[1]*q.data[1]
                           + q.data[2]*q.data[2] + q.data[3]*q.data[3];
        const float invLen = fastRsqrt(lenSqr);
        q = vm::Quaternion<float>(
            q.data[0]*invLen, q.data[1]*invLen,
            q.data[2]*invLen, q.data[3]*invLen);

        // Update world-space inverse inertia tensor for this frame's new orientation
        updateInvInertiaWorld(d);
    }

    // Clear accumulators
    d.forceAccum  = { 0.f, 0.f, 0.f };
    d.torqueAccum = { 0.f, 0.f, 0.f };

    // Accumulate per-body sleep frames; the island manager makes the final
    // sleep/wake decision so all bodies in a connected component sleep together.

    if (vSq <= linThreshSq && wSq <= angThreshSq) {
        ++d.sleepFrames;
    } else {
        d.sleepFrames = 0;
    }
}

} // namespace

void integrate(BodyPool& pool, const IntegratorSettings& settings, float dt) {
    const float linThreshSq  = settings.sleepLinearThreshold  * settings.sleepLinearThreshold;
    const float angThreshSq  = settings.sleepAngularThreshold * settings.sleepAngularThreshold;

    const auto& ids = pool.activeDynamicIds();
    for (uint32_t id : ids) {
        integrateOneBody(pool.get(id), settings, dt, linThreshSq, angThreshSq);
    }
}

void integrateSlice(BodyPool& pool, const IntegratorSettings& settings, float dt,
                    const uint32_t* ids, int count) {
    const float linThreshSq  = settings.sleepLinearThreshold  * settings.sleepLinearThreshold;
    const float angThreshSq  = settings.sleepAngularThreshold * settings.sleepAngularThreshold;

    for (int i = 0; i < count; ++i) {
        integrateOneBody(pool.get(ids[i]), settings, dt, linThreshSq, angThreshSq);
    }
}

} // namespace campello::physics
