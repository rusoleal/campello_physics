#pragma once

#include <campello_physics/defines.h>

namespace campello::physics {

struct Transform {
    vm::Vector3<float>    position;
    vm::Quaternion<float> rotation;

    Transform() noexcept
        : position(0.f, 0.f, 0.f)
        , rotation(vm::Quaternion<float>::identity())
    {}

    Transform(const vm::Vector3<float>&    position,
              const vm::Quaternion<float>& rotation) noexcept
        : position(position), rotation(rotation)
    {}

    CAMPELLO_NODISCARD static Transform identity() noexcept {
        return Transform{};
    }

    // Transforms a world-space point: applies rotation then translation.
    CAMPELLO_NODISCARD vm::Vector3<float> transformPoint(const vm::Vector3<float>& p) const noexcept {
        return rotation.rotated(p) + position;
    }

    // Transforms a direction vector (rotation only, no translation).
    CAMPELLO_NODISCARD vm::Vector3<float> transformVector(const vm::Vector3<float>& v) const noexcept {
        return rotation.rotated(v);
    }

    // Inverse-transforms a world-space point back to local space.
    CAMPELLO_NODISCARD vm::Vector3<float> inverseTransformPoint(const vm::Vector3<float>& p) const noexcept {
        return rotation.conjugated().rotated(p - position);
    }

    // Inverse-transforms a direction vector.
    CAMPELLO_NODISCARD vm::Vector3<float> inverseTransformVector(const vm::Vector3<float>& v) const noexcept {
        return rotation.conjugated().rotated(v);
    }

    // Returns the inverse transform.
    CAMPELLO_NODISCARD Transform inversed() const noexcept {
        auto invRot = rotation.conjugated();
        auto invPos = invRot.rotated(vm::Vector3<float>(-position.x(), -position.y(), -position.z()));
        return Transform{invPos, invRot};
    }

    // Composes this (parent) transform with a child transform.
    // Result: applies child first, then this.
    CAMPELLO_NODISCARD Transform combined(const Transform& child) const noexcept {
        return Transform{
            transformPoint(child.position),
            rotation * child.rotation
        };
    }
};

} // namespace campello::physics
