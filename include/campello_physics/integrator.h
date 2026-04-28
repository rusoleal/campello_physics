#pragma once

#include <campello_physics/body_pool.h>

namespace campello::physics {

// ── Integrator settings ───────────────────────────────────────────────────────

struct IntegratorSettings {
    vm::Vector3<float> gravity              = { 0.f, -9.81f, 0.f };

    // A body whose linear AND angular speed both stay below these thresholds
    // for sleepFramesRequired consecutive steps will be put to sleep.
    float sleepLinearThreshold  = 0.01f;  // m/s
    float sleepAngularThreshold = 0.01f;  // rad/s
    int   sleepFramesRequired   = 60;     // ~1 s at 60 fps
};

// ── Integration ───────────────────────────────────────────────────────────────
//
// Advances every active dynamic body by dt using semi-implicit Euler:
//   v += (F/m + g) * dt
//   x += v * dt
//   ω += I⁻¹_world * τ * dt
//   q  = normalize(q + 0.5 * [0,ω] * q * dt)
//
// Linear and angular damping are applied as velocity attenuation before the
// gravity / force step.  Force and torque accumulators are cleared after use.
// The sleep system checks velocities each step and puts quiescent bodies to sleep.

void integrate(BodyPool& pool, const IntegratorSettings& settings, float dt);

// Integrate a contiguous slice of body IDs (used by the multi-threaded path).
void integrateSlice(BodyPool& pool, const IntegratorSettings& settings, float dt,
                    const uint32_t* ids, int count);

} // namespace campello::physics
