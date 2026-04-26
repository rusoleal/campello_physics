#include "articulated_body_system.h"
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <campello_physics/constraints/hinge_constraint.h>
#include <campello_physics/constraints/fixed_constraint.h>
#include <cassert>

namespace campello::physics {

// ── ArticulatedBody method implementations ────────────────────────────────────

std::shared_ptr<Constraint> ArticulatedBody::constraint(int i) const noexcept {
    if (i <= 0 || i >= linkCount()) return nullptr;
    return m_constraints[i];
}

std::shared_ptr<HingeConstraint> ArticulatedBody::hingeConstraint(int i) const noexcept {
    return std::dynamic_pointer_cast<HingeConstraint>(constraint(i));
}

std::shared_ptr<BallSocketConstraint> ArticulatedBody::ballSocketConstraint(int i) const noexcept {
    return std::dynamic_pointer_cast<BallSocketConstraint>(constraint(i));
}

std::shared_ptr<FixedConstraint> ArticulatedBody::fixedConstraint(int i) const noexcept {
    return std::dynamic_pointer_cast<FixedConstraint>(constraint(i));
}

// ── ArticulatedBodySystem ─────────────────────────────────────────────────────

ArticulatedBody ArticulatedBodySystem::create(
    const ArticulatedBodyDescriptor& desc,
    BodyCreateFn     createBody,
    ConstraintAddFn  addConstraint)
{
    if (desc.links.empty()) return {};

    ArticulatedBodyRecord record;
    record.bodies.resize(desc.links.size());
    record.constraints.resize(desc.links.size());  // [0] = null (root)
    record.active = true;

    // Create all bodies first
    for (int i = 0; i < static_cast<int>(desc.links.size()); ++i) {
        record.bodies[i] = createBody(desc.links[i].body);
    }

    // Create joints (skip root at index 0)
    for (int i = 1; i < static_cast<int>(desc.links.size()); ++i) {
        const auto& ld = desc.links[i];
        assert(ld.parentIndex >= 0 && ld.parentIndex < i);

        Body parentBody = record.bodies[ld.parentIndex];
        Body childBody  = record.bodies[i];

        std::shared_ptr<Constraint> joint;

        switch (ld.jointType) {
        case ArticulatedJointType::BallSocket:
            joint = BallSocketConstraint::create(
                parentBody, ld.anchorInParent,
                childBody,  ld.anchorInSelf);
            break;

        case ArticulatedJointType::Hinge: {
            auto hinge = HingeConstraint::create(
                parentBody, ld.anchorInParent, ld.hingeAxisInParent,
                childBody,  ld.anchorInSelf,   ld.hingeAxisInSelf);
            if (ld.hingeLimitsActive)
                hinge->setLimits(ld.hingeLimitMin, ld.hingeLimitMax);
            if (ld.hingeMotorActive)
                hinge->setMotor(ld.hingeMotorSpeed, ld.hingeMotorMaxImpulse);
            joint = hinge;
            break;
        }

        case ArticulatedJointType::Fixed:
            joint = FixedConstraint::create(parentBody, childBody);
            break;
        }

        addConstraint(joint);
        record.constraints[i] = joint;
    }

    uint32_t id;
    if (!m_freeList.empty()) {
        id = m_freeList.back();
        m_freeList.pop_back();
        m_records[id] = std::move(record);
    } else {
        id = static_cast<uint32_t>(m_records.size());
        m_records.push_back(std::move(record));
    }

    return ArticulatedBody(id,
                           m_records[id].bodies,
                           m_records[id].constraints);
}

void ArticulatedBodySystem::destroy(ArticulatedBody ab,
                                     BodyDestroyFn      destroyBody,
                                     ConstraintRemoveFn removeConstraint)
{
    if (!ab.isValid() || ab.id() >= m_records.size()) return;
    auto& rec = m_records[ab.id()];
    if (!rec.active) return;

    // Remove constraints first (before destroying bodies)
    for (auto& c : rec.constraints) {
        if (c) removeConstraint(c);
    }
    for (auto& b : rec.bodies) {
        if (b.isValid()) destroyBody(b);
    }

    rec.bodies.clear();
    rec.constraints.clear();
    rec.active = false;
    m_freeList.push_back(ab.id());
}

} // namespace campello::physics
