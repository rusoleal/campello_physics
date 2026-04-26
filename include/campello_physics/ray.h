#pragma once

#include <campello_physics/defines.h>
#include <limits>

namespace campello::physics {

struct Ray {
    vm::Vector3<float> origin;
    vm::Vector3<float> direction;  // must be normalized
    float              maxDistance;

    Ray() noexcept
        : origin(0.f, 0.f, 0.f)
        , direction(0.f, 0.f, 1.f)
        , maxDistance(std::numeric_limits<float>::infinity())
    {}

    Ray(const vm::Vector3<float>& origin,
        const vm::Vector3<float>& direction,
        float maxDistance = std::numeric_limits<float>::infinity()) noexcept
        : origin(origin)
        , direction(direction)
        , maxDistance(maxDistance)
    {}

    CAMPELLO_NODISCARD vm::Vector3<float> pointAt(float t) const noexcept {
        return origin + direction * t;
    }
};

} // namespace campello::physics
