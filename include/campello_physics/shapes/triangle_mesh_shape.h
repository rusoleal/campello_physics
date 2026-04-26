#pragma once

#include <campello_physics/shape.h>
#include <array>
#include <vector>

namespace campello::physics {

// Static triangle mesh with an AABB BVH.
// Only valid as a Static body shape — inertia is zero.
class TriangleMeshShape final : public Shape {
public:
    struct Triangle {
        vm::Vector3<float> v[3];
    };

    // Internal BVH node (opaque; used by broadphase/narrowphase).
    struct BVHNode {
        AABB aabb;
        int  left     = -1;  // -1 = leaf
        int  right    = -1;
        int  triStart =  0;
        int  triCount =  0;
    };

    TriangleMeshShape(std::vector<vm::Vector3<float>> vertices,
                      std::vector<std::array<uint32_t, 3>> indices);

    [[nodiscard]] ShapeType type() const noexcept override { return ShapeType::TriangleMesh; }

    [[nodiscard]] AABB computeAABB(const Transform& t) const noexcept override;

    [[nodiscard]] vm::Vector3<float> computeLocalInertiaDiagonal(float /*mass*/) const noexcept override {
        return vm::Vector3<float>(0.f, 0.f, 0.f);
    }

    // GJK is not applicable to non-convex triangle meshes.
    [[nodiscard]] vm::Vector3<float> support(const vm::Vector3<float>& /*dir*/) const noexcept override {
        return vm::Vector3<float>(0.f, 0.f, 0.f);
    }

    [[nodiscard]] const std::vector<Triangle>&   triangles() const noexcept { return m_tris; }
    [[nodiscard]] const std::vector<BVHNode>&    bvhNodes()  const noexcept { return m_bvh; }
    [[nodiscard]] const AABB&                    localAABB() const noexcept { return m_localAABB; }

private:
    std::vector<Triangle> m_tris;
    std::vector<BVHNode>  m_bvh;
    AABB                  m_localAABB;

    void buildBVH(int nodeIdx, int triStart, int triCount);
};

} // namespace campello::physics
