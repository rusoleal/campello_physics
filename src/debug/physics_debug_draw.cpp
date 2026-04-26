#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/shapes/cylinder_shape.h>
#include <campello_physics/shapes/compound_shape.h>
#include <cmath>

namespace campello::physics {

namespace {

// Colour coding
constexpr DebugColor kColorDynamic   = DebugColor::white();
constexpr DebugColor kColorStatic    = DebugColor::grey();
constexpr DebugColor kColorKinematic = DebugColor::cyan();
constexpr DebugColor kColorSensor    = DebugColor::yellow();
constexpr DebugColor kColorSleeping  = DebugColor{0.3f, 0.3f, 0.8f, 1.f};
constexpr DebugColor kColorAABB      = DebugColor{0.2f, 0.8f, 0.2f, 0.5f};
constexpr DebugColor kColorContact   = DebugColor::red();
constexpr DebugColor kColorNormal    = DebugColor::orange();
constexpr DebugColor kColorVelocity  = DebugColor::magenta();
constexpr DebugColor kColorConstraint= DebugColor::blue();

DebugColor bodyColor(const BodyData& d, bool sleepFlag) {
    if (sleepFlag && d.isSleeping) return kColorSleeping;
    switch (d.type) {
    case BodyType::Static:    return kColorStatic;
    case BodyType::Kinematic: return kColorKinematic;
    case BodyType::Sensor:    return kColorSensor;
    default:                  return kColorDynamic;
    }
}

void drawShape(IDebugDraw& draw, const Shape* shape,
               const Transform& t, DebugColor color) {
    if (!shape) return;

    switch (shape->type()) {
    case ShapeType::Sphere: {
        const auto* s = static_cast<const SphereShape*>(shape);
        draw.drawSphere(t.position, s->radius(), color);
        break;
    }
    case ShapeType::Box: {
        const auto* b = static_cast<const BoxShape*>(shape);
        draw.drawBox(t.position, b->halfExtents(), t.rotation, color);
        break;
    }
    case ShapeType::Capsule: {
        const auto* c = static_cast<const CapsuleShape*>(shape);
        draw.drawCapsule(t.position, t.rotation, c->radius(), c->halfHeight(), color);
        break;
    }
    case ShapeType::Cylinder: {
        const auto* cy = static_cast<const CylinderShape*>(shape);
        // Approximate cylinder as a box wireframe
        draw.drawBox(t.position,
                     { cy->radius(), cy->halfHeight(), cy->radius() },
                     t.rotation, color);
        break;
    }
    case ShapeType::Compound: {
        const auto* comp = static_cast<const CompoundShape*>(shape);
        for (const auto& child : comp->children()) {
            Transform childWorld;
            childWorld.position = t.position + t.rotation.rotated(child.localTransform.position);
            childWorld.rotation = t.rotation * child.localTransform.rotation;
            drawShape(draw, child.shape.get(), childWorld, color);
        }
        break;
    }
    default: {
        // ConvexHull, TriangleMesh, HeightField: draw AABB as fallback
        const AABB aabb = shape->computeAABB(t);
        draw.drawAABB(aabb.min, aabb.max, color);
        break;
    }
    }
}

} // namespace

// ── PhysicsWorld::debugDraw ───────────────────────────────────────────────────

void PhysicsWorld::debugDraw(IDebugDraw& draw, DebugDrawFlags flags) const {
    const bool doShapes    = flags & DebugDrawFlags::BodyShapes;
    const bool doAABBs     = flags & DebugDrawFlags::BodyAABBs;
    const bool doContacts  = flags & DebugDrawFlags::ContactPoints;
    const bool doSleep     = flags & DebugDrawFlags::SleepState;
    const bool doVelocity  = flags & DebugDrawFlags::Velocities;
    const bool doConstraints = flags & DebugDrawFlags::Constraints;

    // ── Bodies ────────────────────────────────────────────────────────────────
    if (doShapes || doAABBs || doVelocity) {
        m_pool.forEach([&](uint32_t /*id*/, const BodyData& d) {
            const DebugColor color = bodyColor(d, doSleep);

            if (doShapes && d.shape) {
                drawShape(draw, d.shape.get(), d.transform, color);
            }

            if (doAABBs && d.shape) {
                const AABB aabb = d.shape->computeAABB(d.transform);
                draw.drawAABB(aabb.min, aabb.max, kColorAABB);
            }

            if (doVelocity && d.type == BodyType::Dynamic) {
                const float speed = std::sqrt(
                    d.linearVelocity.x()*d.linearVelocity.x() +
                    d.linearVelocity.y()*d.linearVelocity.y() +
                    d.linearVelocity.z()*d.linearVelocity.z());
                if (speed > 0.01f) {
                    draw.drawArrow(d.transform.position, d.linearVelocity, speed, kColorVelocity);
                }
            }
        });
    }

    // ── Contact points ────────────────────────────────────────────────────────
    if (doContacts) {
        for (const auto& manifold : m_narrowPhase.manifolds()) {
            for (int i = 0; i < manifold.count; ++i) {
                const auto& cp = manifold.points[i];
                draw.drawPoint(cp.position, 0.04f, kColorContact);
                draw.drawArrow(cp.position, cp.normal, 0.1f, kColorNormal);
            }
        }
    }

    // ── Constraints ───────────────────────────────────────────────────────────
    if (doConstraints) {
        for (const auto& c : m_constraintSolver.constraints()) {
            if (!c) continue;
            const Body a = c->bodyA(), b = c->bodyB();
            if (!a.isValid() || !b.isValid()) continue;
            const auto& posA = m_pool.get(a.id()).transform.position;
            const auto& posB = m_pool.get(b.id()).transform.position;
            draw.drawLine(posA, posB, kColorConstraint);
            draw.drawPoint(posA, 0.05f, kColorConstraint);
            draw.drawPoint(posB, 0.05f, kColorConstraint);
        }
    }
}

} // namespace campello::physics
