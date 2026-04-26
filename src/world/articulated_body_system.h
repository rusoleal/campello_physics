#pragma once

#include <campello_physics/articulated_body.h>
#include <campello_physics/body_pool.h>
#include <campello_physics/constraint.h>
#include <vector>
#include <functional>

namespace campello::physics {

// Callbacks supplied by PhysicsWorld so the system can create/destroy
// bodies and constraints without depending on PhysicsWorld directly.

using BodyCreateFn  = std::function<Body(const BodyDescriptor&)>;
using BodyDestroyFn = std::function<void(Body)>;
using ConstraintAddFn    = std::function<void(std::shared_ptr<Constraint>)>;
using ConstraintRemoveFn = std::function<void(const std::shared_ptr<Constraint>&)>;

struct ArticulatedBodyRecord {
    std::vector<Body>                        bodies;
    std::vector<std::shared_ptr<Constraint>> constraints; // [i] → joint to parent of link i
    bool active = false;
};

class ArticulatedBodySystem {
public:
    [[nodiscard]] ArticulatedBody create(
        const ArticulatedBodyDescriptor& desc,
        BodyCreateFn  createBody,
        ConstraintAddFn addConstraint);

    void destroy(ArticulatedBody ab,
                 BodyDestroyFn    destroyBody,
                 ConstraintRemoveFn removeConstraint);

private:
    std::vector<ArticulatedBodyRecord> m_records;
    std::vector<uint32_t>              m_freeList;
};

} // namespace campello::physics
