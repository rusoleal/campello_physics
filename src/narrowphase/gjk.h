#pragma once

#include <campello_physics/contact.h>
#include <campello_physics/narrow_phase.h>
#include <optional>

namespace campello::physics::detail {

// GJK + EPA collision detection for two convex shapes.
// Both ShapeInstances must have a shape whose support() function is meaningful
// (i.e., not TriangleMesh or HeightField).
[[nodiscard]] std::optional<ContactManifold>
collideConvex(const ShapeInstance& a, const ShapeInstance& b);

} // namespace campello::physics::detail
