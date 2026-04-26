#include <campello_physics/physics_world.h>
#include <campello_physics/narrow_phase.h>
#include <vector>

namespace campello::physics {

std::vector<OverlapResult>
PhysicsWorld::overlap(const Shape& shape, const Transform& shapeTransform,
                      const QueryFilter& filter) const
{
    AABB queryAABB = shape.computeAABB(shapeTransform);
    ShapeInstance queryInst{ &shape, shapeTransform };

    std::vector<OverlapResult> results;

    m_broadPhase.queryAABB(
        queryAABB, filter.layer, filter.mask,
        [&](uint32_t bodyId) {
            const auto& bd = m_pool.get(bodyId);
            if (bd.type == BodyType::Sensor) return;
            if (!bd.shape) return;

            ShapeInstance bodyInst = m_pool.getShapeInstance(bodyId);
            auto manifold = collide(queryInst, bodyInst);
            if (manifold)
                results.push_back(OverlapResult{ m_pool.makeHandle(bodyId) });
        });

    return results;
}

} // namespace campello::physics
