#pragma once

#include <campello_physics/defines.h>
#include <cstdint>

namespace campello::physics {

struct ContactPoint {
    vm::Vector3<float> position;          // World-space midpoint between the two surfaces
    vm::Vector3<float> normal;            // Unit normal from B toward A
    float              depth;             // Penetration depth (> 0 when overlapping)
    float              warmStartImpulse;  // Cached normal impulse for warm starting (Phase 6)
    float              warmStartFriction0;
    float              warmStartFriction1;
};

struct ContactManifold {
    static constexpr int kMaxPoints = 4;

    ContactPoint points[kMaxPoints];
    int          count = 0;   // Active contact point count (0 = no contact)
    uint32_t     bodyA = 0;   // Set by the pipeline; not by shape-level collision functions
    uint32_t     bodyB = 0;
};

} // namespace campello::physics
