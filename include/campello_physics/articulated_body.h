#pragma once

#include <campello_physics/body.h>
#include <campello_physics/constraint.h>
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <campello_physics/constraints/hinge_constraint.h>
#include <campello_physics/constraints/fixed_constraint.h>
#include <cstdint>
#include <memory>
#include <vector>

namespace campello::physics {

// ── ArticulatedJointType ──────────────────────────────────────────────────────

enum class ArticulatedJointType {
    BallSocket,  // 3 rotational DOF — shoulder, hip
    Hinge,       // 1 rotational DOF — elbow, knee; supports limits and motor
    Fixed,       // 0 DOF — rigid weld
};

// ── ArticulatedLinkDesc ───────────────────────────────────────────────────────
//
// Describes one link (body + joint to parent) in the articulation tree.
// links[0] must have parentIndex = -1 (root, no joint created).
// All other links must have parentIndex pointing to an earlier entry.

struct ArticulatedLinkDesc {
    int            parentIndex = -1;  // -1 → root link (no joint)
    BodyDescriptor body;

    // Joint connecting this link to its parent (ignored for root)
    ArticulatedJointType jointType = ArticulatedJointType::BallSocket;

    // Anchor in parent local space (pivot point on the parent body)
    vm::Vector3<float> anchorInParent = { 0.f, 0.f, 0.f };
    // Anchor in this link's local space
    vm::Vector3<float> anchorInSelf   = { 0.f, 0.f, 0.f };

    // Hinge-only settings (ignored for BallSocket and Fixed)
    vm::Vector3<float> hingeAxisInParent   = { 0.f, 0.f, 1.f };
    vm::Vector3<float> hingeAxisInSelf     = { 0.f, 0.f, 1.f };
    float              hingeLimitMin       = 0.f;
    float              hingeLimitMax       = 0.f;
    bool               hingeLimitsActive   = false;
    bool               hingeMotorActive    = false;
    float              hingeMotorSpeed     = 0.f;
    float              hingeMotorMaxImpulse = 10.f;
};

// ── ArticulatedBodyDescriptor ─────────────────────────────────────────────────

struct ArticulatedBodyDescriptor {
    std::vector<ArticulatedLinkDesc> links;
};

// ── ArticulatedBody ───────────────────────────────────────────────────────────
//
// Thin copyable handle returned by PhysicsWorld::createArticulatedBody().
// Stores per-link Body handles and joint constraint pointers.

class ArticulatedBody {
public:
    ArticulatedBody() = default;

    [[nodiscard]] bool     isValid()   const noexcept { return m_id != kInvalid; }
    [[nodiscard]] uint32_t id()        const noexcept { return m_id; }
    [[nodiscard]] int      linkCount() const noexcept {
        return static_cast<int>(m_bodies.size());
    }

    // Body handle for link i
    [[nodiscard]] Body linkBody(int i) const noexcept {
        return (i >= 0 && i < linkCount()) ? m_bodies[i] : Body{};
    }

    // Joint constraint for link i (null if i==0 root or no constraint stored)
    [[nodiscard]] std::shared_ptr<Constraint>           constraint(int i)            const noexcept;
    [[nodiscard]] std::shared_ptr<HingeConstraint>      hingeConstraint(int i)       const noexcept;
    [[nodiscard]] std::shared_ptr<BallSocketConstraint> ballSocketConstraint(int i)  const noexcept;
    [[nodiscard]] std::shared_ptr<FixedConstraint>      fixedConstraint(int i)       const noexcept;

    bool operator==(const ArticulatedBody& o) const noexcept { return m_id == o.m_id; }
    bool operator!=(const ArticulatedBody& o) const noexcept { return m_id != o.m_id; }

private:
    static constexpr uint32_t kInvalid = ~uint32_t(0);
    explicit ArticulatedBody(uint32_t id,
                              std::vector<Body> bodies,
                              std::vector<std::shared_ptr<Constraint>> constraints)
        : m_id(id)
        , m_bodies(std::move(bodies))
        , m_constraints(std::move(constraints)) {}
    friend class ArticulatedBodySystem;

    uint32_t                              m_id          = kInvalid;
    std::vector<Body>                     m_bodies;
    std::vector<std::shared_ptr<Constraint>> m_constraints; // m_constraints[i] = joint to parent of link i (null for root)
};

} // namespace campello::physics
