#pragma once

#include <campello_physics/contact.h>
#include <campello_physics/transform.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <optional>

namespace campello::physics::detail {

[[nodiscard]] std::optional<ContactManifold>
collideSpheres(const SphereShape& a, const Transform& ta,
               const SphereShape& b, const Transform& tb);

// sphereIsA: when true the sphere is body A (normal from B->A = from box toward sphere).
[[nodiscard]] std::optional<ContactManifold>
collideSphereBox(const SphereShape& sphere, const Transform& ts,
                 const BoxShape&    box,    const Transform& tb,
                 bool               sphereIsA);

[[nodiscard]] std::optional<ContactManifold>
collideSphereCapsule(const SphereShape&  sphere,  const Transform& ts,
                     const CapsuleShape& capsule, const Transform& tc,
                     bool                sphereIsA);

[[nodiscard]] std::optional<ContactManifold>
collideCapsuleCapsule(const CapsuleShape& a, const Transform& ta,
                      const CapsuleShape& b, const Transform& tb);

// Returns up to 4 contact points for a box-box collision.
[[nodiscard]] std::optional<ContactManifold>
collideBoxBox(const BoxShape& a, const Transform& ta,
              const BoxShape& b, const Transform& tb);

} // namespace campello::physics::detail
