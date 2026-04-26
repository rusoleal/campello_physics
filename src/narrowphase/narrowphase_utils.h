#pragma once

#include <campello_physics/defines.h>
#include <cmath>
#include <algorithm>

// Internal math helpers for the narrow phase.
// Vector3 already supports +, -, * scalar via vector_math operators.
// This file adds dot, cross, length, normalize, and a few scalar-first utilities.

namespace campello::physics::detail {

using V3 = vm::Vector3<float>;

CAMPELLO_FORCE_INLINE float dot3(const V3& a, const V3& b) noexcept {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
}

CAMPELLO_FORCE_INLINE V3 cross3(const V3& a, const V3& b) noexcept {
    return V3(a.y()*b.z() - a.z()*b.y(),
              a.z()*b.x() - a.x()*b.z(),
              a.x()*b.y() - a.y()*b.x());
}

CAMPELLO_FORCE_INLINE float lenSq3(const V3& v) noexcept { return dot3(v, v); }
CAMPELLO_FORCE_INLINE float len3  (const V3& v) noexcept { return std::sqrt(dot3(v, v)); }

CAMPELLO_FORCE_INLINE V3 neg3(const V3& v) noexcept { return v * -1.f; }

CAMPELLO_FORCE_INLINE V3 norm3(const V3& v) noexcept {
    float l = len3(v);
    if (l < 1e-10f) return V3(1.f, 0.f, 0.f);
    return v * (1.f / l);
}

// Returns a unit vector perpendicular to v.
CAMPELLO_FORCE_INLINE V3 perp3(const V3& v) noexcept {
    float ax = std::fabs(v.x()), ay = std::fabs(v.y()), az = std::fabs(v.z());
    V3 u;
    if (ax <= ay && ax <= az) u = V3(1.f, 0.f, 0.f);
    else if (ay <= az)        u = V3(0.f, 1.f, 0.f);
    else                      u = V3(0.f, 0.f, 1.f);
    return norm3(cross3(v, u));
}

} // namespace campello::physics::detail
