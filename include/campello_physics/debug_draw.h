#pragma once

#include <campello_physics/defines.h>
#include <cstdint>

namespace campello::physics {

// ── DebugColor ────────────────────────────────────────────────────────────────

struct DebugColor {
    float r, g, b, a;

    static constexpr DebugColor white()   { return {1.f, 1.f, 1.f, 1.f}; }
    static constexpr DebugColor red()     { return {1.f, 0.f, 0.f, 1.f}; }
    static constexpr DebugColor green()   { return {0.f, 1.f, 0.f, 1.f}; }
    static constexpr DebugColor blue()    { return {0.f, 0.5f,1.f, 1.f}; }
    static constexpr DebugColor yellow()  { return {1.f, 1.f, 0.f, 1.f}; }
    static constexpr DebugColor orange()  { return {1.f, 0.5f,0.f, 1.f}; }
    static constexpr DebugColor cyan()    { return {0.f, 1.f, 1.f, 1.f}; }
    static constexpr DebugColor magenta() { return {1.f, 0.f, 1.f, 1.f}; }
    static constexpr DebugColor grey()    { return {0.5f,0.5f,0.5f,1.f}; }
};

// ── DebugDrawFlags ────────────────────────────────────────────────────────────
//
// Bit flags passed to PhysicsWorld::debugDraw() to control what is rendered.

enum class DebugDrawFlags : uint32_t {
    None           = 0,
    BodyShapes     = 1 << 0,  // wireframe shape outlines
    BodyAABBs      = 1 << 1,  // world-space AABBs
    ContactPoints  = 1 << 2,  // contact point positions + normals
    Constraints    = 1 << 3,  // anchor points and joint axes
    SleepState     = 1 << 4,  // sleeping bodies drawn in a different colour
    Velocities     = 1 << 5,  // linear velocity arrows
    All            = ~uint32_t(0)
};

inline DebugDrawFlags operator|(DebugDrawFlags a, DebugDrawFlags b) {
    return static_cast<DebugDrawFlags>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline bool operator&(DebugDrawFlags a, DebugDrawFlags b) {
    return (static_cast<uint32_t>(a) & static_cast<uint32_t>(b)) != 0;
}

// ── IDebugDraw ────────────────────────────────────────────────────────────────
//
// Pure interface.  Implement this in your renderer; pass a pointer to
// PhysicsWorld::debugDraw() each frame.
//
// All coordinates are in world space.

class IDebugDraw {
public:
    virtual ~IDebugDraw() = default;

    // Straight line between two world-space points.
    virtual void drawLine(const vm::Vector3<float>& from,
                          const vm::Vector3<float>& to,
                          DebugColor color) = 0;

    // Axis-aligned bounding box wireframe (12 edges).
    virtual void drawAABB(const vm::Vector3<float>& min,
                          const vm::Vector3<float>& max,
                          DebugColor color);

    // Sphere wireframe (three great-circle rings, default 16 segments each).
    virtual void drawSphere(const vm::Vector3<float>& centre,
                            float radius,
                            DebugColor color,
                            int segments = 16);

    // Box wireframe given centre, half-extents, and orientation.
    virtual void drawBox(const vm::Vector3<float>& centre,
                         const vm::Vector3<float>& halfExtents,
                         const vm::Quaternion<float>& rotation,
                         DebugColor color);

    // Capsule wireframe (Y-up cylinder + hemisphere caps).
    virtual void drawCapsule(const vm::Vector3<float>& centre,
                              const vm::Quaternion<float>& rotation,
                              float radius,
                              float halfHeight,
                              DebugColor color,
                              int segments = 16);

    // Arrow: line from base to base+direction scaled by length.
    virtual void drawArrow(const vm::Vector3<float>& base,
                           const vm::Vector3<float>& direction,
                           float length,
                           DebugColor color);

    // Cross marker (three short axis lines centred at point).
    virtual void drawPoint(const vm::Vector3<float>& point,
                           float size,
                           DebugColor color);
};

} // namespace campello::physics
