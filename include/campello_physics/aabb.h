#pragma once

#include <campello_physics/defines.h>
#include <algorithm>
#include <limits>

namespace campello::physics {

struct AABB {
    vm::Vector3<float> min;
    vm::Vector3<float> max;

    CAMPELLO_NODISCARD static AABB empty() noexcept {
        constexpr float inf = std::numeric_limits<float>::infinity();
        AABB b;
        b.min = vm::Vector3<float>( inf,  inf,  inf);
        b.max = vm::Vector3<float>(-inf, -inf, -inf);
        return b;
    }

    CAMPELLO_NODISCARD static AABB fromCenterHalfExtents(
        const vm::Vector3<float>& center,
        const vm::Vector3<float>& halfExtents) noexcept
    {
        return { center - halfExtents, center + halfExtents };
    }

    CAMPELLO_NODISCARD static AABB fromMinMax(
        const vm::Vector3<float>& mn,
        const vm::Vector3<float>& mx) noexcept
    {
        return { mn, mx };
    }

    CAMPELLO_NODISCARD bool isEmpty() const noexcept {
        return min.x() > max.x() || min.y() > max.y() || min.z() > max.z();
    }

    void expand(const vm::Vector3<float>& p) noexcept {
        min = vm::Vector3<float>(std::min(min.x(), p.x()),
                                 std::min(min.y(), p.y()),
                                 std::min(min.z(), p.z()));
        max = vm::Vector3<float>(std::max(max.x(), p.x()),
                                 std::max(max.y(), p.y()),
                                 std::max(max.z(), p.z()));
    }

    void expand(const AABB& other) noexcept {
        expand(other.min);
        expand(other.max);
    }

    CAMPELLO_NODISCARD bool intersects(const AABB& other) const noexcept {
        return min.x() <= other.max.x() && max.x() >= other.min.x() &&
               min.y() <= other.max.y() && max.y() >= other.min.y() &&
               min.z() <= other.max.z() && max.z() >= other.min.z();
    }

    CAMPELLO_NODISCARD bool contains(const vm::Vector3<float>& p) const noexcept {
        return p.x() >= min.x() && p.x() <= max.x() &&
               p.y() >= min.y() && p.y() <= max.y() &&
               p.z() >= min.z() && p.z() <= max.z();
    }

    CAMPELLO_NODISCARD bool contains(const AABB& other) const noexcept {
        return contains(other.min) && contains(other.max);
    }

    CAMPELLO_NODISCARD AABB merged(const AABB& other) const noexcept {
        AABB b = *this;
        b.expand(other);
        return b;
    }

    CAMPELLO_NODISCARD vm::Vector3<float> center() const noexcept {
        return (min + max) * 0.5f;
    }

    CAMPELLO_NODISCARD vm::Vector3<float> halfExtents() const noexcept {
        return (max - min) * 0.5f;
    }

    CAMPELLO_NODISCARD vm::Vector3<float> extents() const noexcept {
        return max - min;
    }

    // Half surface area — used by BVH cost functions.
    CAMPELLO_NODISCARD float halfSurfaceArea() const noexcept {
        auto e = extents();
        return e.x() * e.y() + e.y() * e.z() + e.z() * e.x();
    }

    CAMPELLO_NODISCARD float surfaceArea() const noexcept {
        return 2.0f * halfSurfaceArea();
    }
};

} // namespace campello::physics
