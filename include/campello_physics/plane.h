#pragma once

#include <campello_physics/defines.h>

namespace campello::physics {

struct Plane {
    vm::Vector3<float> normal;    // must be unit length
    float              distance;  // signed distance from world origin along normal

    Plane() noexcept : normal(0.f, 1.f, 0.f), distance(0.f) {}

    Plane(const vm::Vector3<float>& normal, float distance) noexcept
        : normal(normal), distance(distance)
    {}

    CAMPELLO_NODISCARD static Plane fromPointNormal(
        const vm::Vector3<float>& point,
        const vm::Vector3<float>& normal) noexcept
    {
        return Plane{normal, normal.dot(point)};
    }

    CAMPELLO_NODISCARD static Plane fromPoints(
        const vm::Vector3<float>& a,
        const vm::Vector3<float>& b,
        const vm::Vector3<float>& c) noexcept
    {
        auto n = vm::Vector3<float>::cross(b - a, c - a);
        n.normalize();
        return fromPointNormal(a, n);
    }

    // Positive values mean the point is on the normal's side.
    CAMPELLO_NODISCARD float signedDistance(const vm::Vector3<float>& point) const noexcept {
        return normal.dot(point) - distance;
    }

    CAMPELLO_NODISCARD bool isOnPositiveSide(const vm::Vector3<float>& point) const noexcept {
        return signedDistance(point) >= 0.f;
    }
};

} // namespace campello::physics
