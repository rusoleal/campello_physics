#pragma once

#include <campello_physics/defines.h>
#include <campello_physics/aabb.h>
#include <campello_physics/transform.h>
#include <cstdint>

namespace campello::physics {

enum class ShapeType : uint8_t {
    Box,
    Sphere,
    Capsule,
    Cylinder,
    ConvexHull,
    TriangleMesh,
    HeightField,
    Compound
};

class Shape {
public:
    virtual ~Shape() = default;

    [[nodiscard]] virtual ShapeType type() const noexcept = 0;

    // Returns the world-space AABB when the shape is placed at worldTransform.
    [[nodiscard]] virtual AABB computeAABB(const Transform& worldTransform) const noexcept = 0;

    // Returns the diagonal of the local inertia tensor (Ixx, Iyy, Izz) for the given mass.
    // Y-axis is the primary symmetry axis for capsule/cylinder.
    // Returns zero vector for static-only shapes (TriangleMesh, HeightField).
    [[nodiscard]] virtual vm::Vector3<float> computeLocalInertiaDiagonal(float mass) const noexcept = 0;

    // Center of mass in local space. Origin for all symmetric primitive shapes.
    [[nodiscard]] virtual vm::Vector3<float> centerOfMass() const noexcept {
        return vm::Vector3<float>(0.f, 0.f, 0.f);
    }

    // GJK support function: returns the point on this shape (in local space) that is
    // furthest in direction `dir` (also in local space).
    // TriangleMesh and HeightField return zero — they are not convex and cannot be used with GJK.
    [[nodiscard]] virtual vm::Vector3<float> support(const vm::Vector3<float>& dir) const noexcept = 0;
};

} // namespace campello::physics
