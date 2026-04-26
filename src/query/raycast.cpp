#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace campello::physics {

namespace {

// ── Per-shape ray intersection ────────────────────────────────────────────────

struct RayHit { float t; vm::Vector3<float> normal; };

inline float dot3(const vm::Vector3<float>& a, const vm::Vector3<float>& b) noexcept {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
}

inline float len3(const vm::Vector3<float>& v) noexcept {
    return std::sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z());
}

inline vm::Vector3<float> norm3(const vm::Vector3<float>& v) noexcept {
    float l = len3(v);
    return l > 1e-20f ? v * (1.f / l) : vm::Vector3<float>(0.f, 1.f, 0.f);
}

// Sphere centered at transform.position with radius r.
std::optional<RayHit> raycastSphere(const vm::Vector3<float>& origin,
                                     const vm::Vector3<float>& dir,
                                     float maxT,
                                     const vm::Vector3<float>& center,
                                     float radius) noexcept
{
    vm::Vector3<float> d = origin - center;
    float b   = dot3(d, dir);
    float c   = dot3(d, d) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.f) return std::nullopt;
    float sq = std::sqrt(disc);
    float t  = -b - sq;
    if (t < 0.f) t = -b + sq;
    if (t < 0.f || t > maxT) return std::nullopt;

    vm::Vector3<float> hit = origin + dir * t;
    vm::Vector3<float> n   = norm3(hit - center);
    return RayHit{ t, n };
}

// Box with half-extents, world transform.
std::optional<RayHit> raycastBox(const vm::Vector3<float>& origin,
                                  const vm::Vector3<float>& dir,
                                  float maxT,
                                  const Transform& t,
                                  const vm::Vector3<float>& halfExtents) noexcept
{
    // Transform ray to box local space
    vm::Vector3<float> lo = t.inverseTransformPoint(origin);
    vm::Vector3<float> ld = t.inverseTransformVector(dir);

    float he[3] = { halfExtents.x(), halfExtents.y(), halfExtents.z() };
    float ox[3] = { lo.x(), lo.y(), lo.z() };
    float dx[3] = { ld.x(), ld.y(), ld.z() };

    float tNear = 0.f, tFar = maxT;
    int hitAxis = -1;
    bool hitNeg  = false;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(dx[i]) < 1e-20f) {
            if (std::abs(ox[i]) > he[i]) return std::nullopt;
            continue;
        }
        float inv = 1.f / dx[i];
        float t1  = (-he[i] - ox[i]) * inv;
        float t2  = ( he[i] - ox[i]) * inv;
        bool neg = t1 > t2;
        if (neg) { float tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > tNear) { tNear = t1; hitAxis = i; hitNeg = !neg; }
        tFar = (tFar < t2) ? tFar : t2;
        if (tFar < tNear) return std::nullopt;
    }

    if (tNear < 0.f || tNear > maxT) return std::nullopt;

    // Compute normal in local space, then transform to world space
    float ln[3] = {0.f, 0.f, 0.f};
    if (hitAxis >= 0) ln[hitAxis] = hitNeg ? -1.f : 1.f;
    vm::Vector3<float> localN(ln[0], ln[1], ln[2]);
    vm::Vector3<float> worldN = norm3(t.transformVector(localN));
    return RayHit{ tNear, worldN };
}

// Capsule: cylinder along Y in local space, radius r, from -halfH to +halfH.
std::optional<RayHit> raycastCapsule(const vm::Vector3<float>& origin,
                                      const vm::Vector3<float>& dir,
                                      float maxT,
                                      const Transform& t,
                                      float radius, float halfH) noexcept
{
    vm::Vector3<float> lo = t.inverseTransformPoint(origin);
    vm::Vector3<float> ld = t.inverseTransformVector(dir);

    float ox = lo.x(), oy = lo.y(), oz = lo.z();
    float dx = ld.x(), dy = ld.y(), dz = ld.z();
    float r = radius;

    std::optional<RayHit> best;
    float bestT = maxT + 1.f;

    // ── Cylinder body ─────────────────────────────────────────────────────────
    float a = dx*dx + dz*dz;
    if (a > 1e-20f) {
        float b   = ox*dx + oz*dz;
        float c   = ox*ox + oz*oz - r*r;
        float disc = b*b - a*c;
        if (disc >= 0.f) {
            float sq = std::sqrt(disc);
            float inv = 1.f / a;
            for (int sign : {-1, 1}) {
                float tc = (-b + sign * sq) * inv;
                if (tc < 0.f || tc > maxT) continue;
                float yHit = oy + tc * dy;
                if (yHit < -halfH || yHit > halfH) continue;
                if (tc < bestT) {
                    bestT = tc;
                    float nx = ox + tc * dx;
                    float nz = oz + tc * dz;
                    vm::Vector3<float> localN = norm3(vm::Vector3<float>(nx, 0.f, nz));
                    best = RayHit{ tc, norm3(t.transformVector(localN)) };
                }
            }
        }
    }

    // ── Hemisphere caps ───────────────────────────────────────────────────────
    for (int capSign : {-1, 1}) {
        float cy = capSign * halfH;
        vm::Vector3<float> capCenter(0.f, cy, 0.f);
        vm::Vector3<float> pd = lo - capCenter;
        float b   = dot3(pd, ld);
        float c   = dot3(pd, pd) - r*r;
        float disc = b*b - c;
        if (disc < 0.f) continue;
        float sq = std::sqrt(disc);
        float tc = -b - sq;
        if (tc < 0.f) tc = -b + sq;
        if (tc < 0.f || tc > maxT) continue;
        float yHit = oy + tc * dy;
        // Only accept cap hits in the correct hemisphere
        if ((capSign > 0 && yHit < halfH) || (capSign < 0 && yHit > -halfH)) continue;
        if (tc < bestT) {
            bestT = tc;
            vm::Vector3<float> localHit(ox + tc*dx, oy + tc*dy, oz + tc*dz);
            vm::Vector3<float> localN = norm3(localHit - capCenter);
            best = RayHit{ tc, norm3(t.transformVector(localN)) };
        }
    }

    return best;
}

// Dispatch to per-shape function.
std::optional<RayHit> raycastShape(const vm::Vector3<float>& origin,
                                    const vm::Vector3<float>& dir,
                                    float maxT,
                                    const BodyData& bd) noexcept
{
    if (!bd.shape) return std::nullopt;

    switch (bd.shape->type()) {
    case ShapeType::Sphere: {
        auto* s = static_cast<const SphereShape*>(bd.shape.get());
        return raycastSphere(origin, dir, maxT, bd.transform.position, s->radius());
    }
    case ShapeType::Box: {
        auto* b = static_cast<const BoxShape*>(bd.shape.get());
        return raycastBox(origin, dir, maxT, bd.transform, b->halfExtents());
    }
    case ShapeType::Capsule: {
        auto* c = static_cast<const CapsuleShape*>(bd.shape.get());
        return raycastCapsule(origin, dir, maxT, bd.transform, c->radius(), c->halfHeight());
    }
    default:
        return std::nullopt;
    }
}

vm::Vector3<float> safeInvDir(const vm::Vector3<float>& d) noexcept {
    constexpr float inf = std::numeric_limits<float>::infinity();
    auto safe = [](float v) { return (std::abs(v) > 1e-20f) ? 1.f / v : (v >= 0.f ? inf : -inf); };
    return vm::Vector3<float>(safe(d.x()), safe(d.y()), safe(d.z()));
}

} // namespace

// ── PhysicsWorld::raycastClosest ──────────────────────────────────────────────

std::optional<RaycastHit>
PhysicsWorld::raycastClosest(const Ray& ray, const QueryFilter& filter) const
{
    vm::Vector3<float> invDir = safeInvDir(ray.direction);
    float maxT = ray.maxDistance;

    std::optional<RaycastHit> best;
    float bestT = maxT + 1.f;

    m_broadPhase.queryRay(
        ray.origin, invDir, maxT,
        filter.layer, filter.mask,
        [&](uint32_t bodyId) {
            if (bodyId == filter.excludeBodyId) return;
            const auto& bd = m_pool.get(bodyId);
            if (bd.type == BodyType::Sensor) return;
            auto hit = raycastShape(ray.origin, ray.direction, maxT, bd);
            if (hit && hit->t < bestT) {
                bestT = hit->t;
                best = RaycastHit{
                    m_pool.makeHandle(bodyId),
                    hit->t,
                    ray.origin + ray.direction * hit->t,
                    hit->normal
                };
            }
        });

    return best;
}

// ── PhysicsWorld::raycastAll ──────────────────────────────────────────────────

std::vector<RaycastHit>
PhysicsWorld::raycastAll(const Ray& ray, const QueryFilter& filter) const
{
    vm::Vector3<float> invDir = safeInvDir(ray.direction);
    float maxT = ray.maxDistance;

    std::vector<RaycastHit> results;

    m_broadPhase.queryRay(
        ray.origin, invDir, maxT,
        filter.layer, filter.mask,
        [&](uint32_t bodyId) {
            if (bodyId == filter.excludeBodyId) return;
            const auto& bd = m_pool.get(bodyId);
            if (bd.type == BodyType::Sensor) return;
            auto hit = raycastShape(ray.origin, ray.direction, maxT, bd);
            if (hit) {
                results.push_back(RaycastHit{
                    m_pool.makeHandle(bodyId),
                    hit->t,
                    ray.origin + ray.direction * hit->t,
                    hit->normal
                });
            }
        });

    std::sort(results.begin(), results.end(),
              [](const RaycastHit& a, const RaycastHit& b) {
                  return a.fraction < b.fraction;
              });

    return results;
}

} // namespace campello::physics
