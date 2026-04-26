#pragma once

#include <campello_physics/articulated_body.h>
#include <cstdint>

namespace campello::physics {

// ── RagdollBone ───────────────────────────────────────────────────────────────
//
// Named index constants for the 15-bone humanoid template.
// Use these as link indices when accessing ragdoll bodies/constraints.

namespace RagdollBone {
    constexpr int Pelvis       =  0;  // root — static/kinematic anchor
    constexpr int SpineLow     =  1;
    constexpr int SpineMid     =  2;
    constexpr int SpineHigh    =  3;
    constexpr int Head         =  4;
    constexpr int ShoulderL    =  5;
    constexpr int UpperArmL    =  6;
    constexpr int LowerArmL    =  7;
    constexpr int ShoulderR    =  8;
    constexpr int UpperArmR    =  9;
    constexpr int LowerArmR    = 10;
    constexpr int UpperLegL    = 11;
    constexpr int LowerLegL    = 12;
    constexpr int UpperLegR    = 13;
    constexpr int LowerLegR    = 14;
    constexpr int Count        = 15;
}

// ── RagdollDescriptor ─────────────────────────────────────────────────────────
//
// High-level descriptor for a humanoid ragdoll.  All sizes are half-lengths
// (radius or half-height in metres).  The ragdoll is built relative to
// rootTransform: pelvis centred there, spine going up, legs going down.

struct RagdollDescriptor {
    Transform rootTransform = Transform::identity();

    float totalMass    = 80.f;   // kg — distributed by bone volume

    // Torso
    float pelvisRadius   = 0.12f;
    float spineRadius    = 0.08f;
    float spineLowH      = 0.10f;  // half-height of SpineLow capsule
    float spineMidH      = 0.10f;
    float spineHighH     = 0.10f;

    // Head
    float headRadius     = 0.12f;

    // Arms
    float shoulderRadius = 0.05f;
    float upperArmH      = 0.15f;
    float upperArmRadius = 0.05f;
    float lowerArmH      = 0.13f;
    float lowerArmRadius = 0.04f;

    // Legs
    float upperLegH      = 0.22f;
    float upperLegRadius = 0.07f;
    float lowerLegH      = 0.20f;
    float lowerLegRadius = 0.05f;

    // Hinge limits (radians)
    float elbowLimitMin  = -2.0f;   // ~115° flexion
    float elbowLimitMax  =  0.05f;  // small hyperextension
    float kneeLimitMin   = -2.2f;
    float kneeLimitMax   =  0.05f;

    // Body type of the root (pelvis). Use Kinematic when driven by animation.
    BodyType rootBodyType = BodyType::Dynamic;

    // Collision layer/mask shared by all bones
    uint32_t layer = 0xFFFFFFFF;
    uint32_t mask  = 0xFFFFFFFF;
};

// ── RagdollBody ───────────────────────────────────────────────────────────────
//
// Returned by PhysicsWorld::createRagdoll().  Wraps an ArticulatedBody and
// provides named bone accessors.

class RagdollBody {
public:
    RagdollBody() = default;

    [[nodiscard]] bool isValid() const noexcept { return m_ab.isValid(); }

    // Per-bone access (use RagdollBone::* constants as index)
    [[nodiscard]] Body bone(int index) const noexcept { return m_ab.linkBody(index); }

    // Underlying articulation (gives access to raw constraints, etc.)
    [[nodiscard]] ArticulatedBody& articulation()       noexcept { return m_ab; }
    [[nodiscard]] const ArticulatedBody& articulation() const noexcept { return m_ab; }

    bool operator==(const RagdollBody& o) const noexcept { return m_ab == o.m_ab; }
    bool operator!=(const RagdollBody& o) const noexcept { return m_ab != o.m_ab; }

private:
    explicit RagdollBody(ArticulatedBody ab) : m_ab(std::move(ab)) {}
    friend class RagdollFactory;
    ArticulatedBody m_ab;
};

// ── RagdollFactory ────────────────────────────────────────────────────────────
//
// Converts a RagdollDescriptor into an ArticulatedBodyDescriptor.
// PhysicsWorld::createRagdoll() calls this internally.

class RagdollFactory {
public:
    [[nodiscard]] static RagdollBody build(
        const RagdollDescriptor& desc,
        ArticulatedBody ab);

    [[nodiscard]] static ArticulatedBodyDescriptor makeDescriptor(
        const RagdollDescriptor& desc);
};

} // namespace campello::physics
