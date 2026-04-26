#pragma once

#include <campello_physics/body.h>
#include <campello_physics/defines.h>
#include <campello_physics/shape.h>
#include <campello_physics/transform.h>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace campello::physics {

struct Ray {
    vm::Vector3<float> origin    = {0.f, 0.f, 0.f};
    vm::Vector3<float> direction = {0.f, 0.f, 1.f};  // should be normalized
    float maxDistance = std::numeric_limits<float>::infinity();
};

// Filter applied when querying the world.
// A body B passes the filter if (filter.layer & B.mask) && (B.layer & filter.mask).
// excludeBodyId: body with this ID is always skipped (~0u = no exclusion).
struct QueryFilter {
    uint32_t layer        = 0xFFFFFFFFu;
    uint32_t mask         = 0xFFFFFFFFu;
    uint32_t excludeBodyId = ~0u;
};

// Result of a raycast query.
// fraction is the distance along the ray direction from ray.origin to the hit point.
struct RaycastHit {
    Body               body;
    float              fraction = 0.f;
    vm::Vector3<float> point    = {0.f, 0.f, 0.f};
    vm::Vector3<float> normal   = {0.f, 0.f, 0.f};
};

// Result of a shape cast (sweep) query.
// fraction is the distance along the cast direction from the start position to first contact.
struct ShapeCastHit {
    Body               body;
    float              fraction = 0.f;
    vm::Vector3<float> point    = {0.f, 0.f, 0.f};
    vm::Vector3<float> normal   = {0.f, 0.f, 0.f};
};

// Result of an overlap query.
struct OverlapResult {
    Body body;
};

} // namespace campello::physics
