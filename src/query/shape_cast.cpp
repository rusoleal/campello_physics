#include <campello_physics/physics_world.h>
#include <campello_physics/narrow_phase.h>
#include <algorithm>
#include <cmath>

namespace campello::physics {

namespace {

inline float len3sq(const vm::Vector3<float>& v) noexcept {
    return v.x()*v.x() + v.y()*v.y() + v.z()*v.z();
}

inline vm::Vector3<float> norm3(const vm::Vector3<float>& v) noexcept {
    float l = std::sqrt(len3sq(v));
    return l > 1e-20f ? v * (1.f / l) : vm::Vector3<float>(0.f, 1.f, 0.f);
}

// Step size: half the smallest radius-like dimension of the two AABBs.
float safeCastStep(const AABB& queryAABB, const AABB& bodyAABB, float maxDist) noexcept {
    auto he = [](const AABB& b) {
        float dx = (b.max.x() - b.min.x()) * 0.5f;
        float dy = (b.max.y() - b.min.y()) * 0.5f;
        float dz = (b.max.z() - b.min.z()) * 0.5f;
        float m  = dx < dy ? dx : dy;
        return m < dz ? m : dz;
    };
    float step = he(queryAABB) < he(bodyAABB) ? he(queryAABB) : he(bodyAABB);
    step = step > 0.001f ? step : 0.001f;
    return step < maxDist ? step : maxDist * 0.5f;
}

} // namespace

std::optional<ShapeCastHit>
PhysicsWorld::shapeCast(const Shape& shape, const Transform& shapeTransform,
                         const vm::Vector3<float>& direction, float maxDistance,
                         const QueryFilter& filter) const
{
    // Swept AABB: union of start and end shape AABBs
    Transform endTransform(shapeTransform.position + direction * maxDistance,
                           shapeTransform.rotation);
    AABB startAABB = shape.computeAABB(shapeTransform);
    AABB endAABB   = shape.computeAABB(endTransform);
    AABB sweptAABB = startAABB.merged(endAABB);

    ShapeInstance queryInst{ &shape, shapeTransform };

    std::optional<ShapeCastHit> best;
    float bestFraction = maxDistance + 1.f;

    m_broadPhase.queryAABB(
        sweptAABB, filter.layer, filter.mask,
        [&](uint32_t bodyId) {
            const auto& bd = m_pool.get(bodyId);
            if (bd.type == BodyType::Sensor) return;
            if (!bd.shape) return;

            ShapeInstance bodyInst = m_pool.getShapeInstance(bodyId);

            // Skip if start position already overlaps (inside the body)
            queryInst.transform = shapeTransform;
            if (collide(queryInst, bodyInst)) return;

            // Step along the cast to find the first overlap interval.
            // Step size is conservative: smaller than either shape's minimum half-extent,
            // so the cast cannot skip through a thin body.
            AABB bodyAABB = bd.shape->computeAABB(bd.transform);
            float step = safeCastStep(startAABB, bodyAABB, maxDistance);

            float firstOverlapT = -1.f;
            float prevT = 0.f;
            for (float t = step; t <= maxDistance + step * 0.5f; t += step) {
                float tc = t < maxDistance ? t : maxDistance;
                queryInst.transform = Transform(
                    shapeTransform.position + direction * tc,
                    shapeTransform.rotation);
                if (collide(queryInst, bodyInst)) {
                    firstOverlapT = tc;
                    break;
                }
                prevT = tc;
                if (tc >= maxDistance) break;
            }

            if (firstOverlapT < 0.f) return;  // no intersection found

            // Binary search in [prevT, firstOverlapT] for exact entry time
            float lo = prevT, hi = firstOverlapT;
            for (int iter = 0; iter < 24; ++iter) {
                float mid = (lo + hi) * 0.5f;
                queryInst.transform = Transform(
                    shapeTransform.position + direction * mid,
                    shapeTransform.rotation);
                if (collide(queryInst, bodyInst)) hi = mid;
                else                              lo = mid;
            }

            if (hi >= bestFraction) return;

            // Get manifold at hi for contact point and normal
            queryInst.transform = Transform(
                shapeTransform.position + direction * hi,
                shapeTransform.rotation);
            auto manifold = collide(queryInst, bodyInst);

            bestFraction = hi;
            ShapeCastHit hit;
            hit.body     = m_pool.makeHandle(bodyId);
            hit.fraction = hi;
            if (manifold && manifold->count > 0) {
                hit.point  = manifold->points[0].position;
                hit.normal = manifold->points[0].normal;
            } else {
                hit.point  = shapeTransform.position + direction * hi;
                hit.normal = norm3(vm::Vector3<float>(
                    -direction.x(), -direction.y(), -direction.z()));
            }
            best = hit;
        });

    return best;
}

} // namespace campello::physics
