#include <campello_physics/ragdoll.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <cmath>

namespace campello::physics {

namespace {

// Build a BodyDescriptor for a capsule-shaped bone.
// position is in world space.  mass is the per-bone share of totalMass.
BodyDescriptor capsuleBone(float halfH, float radius, float mass,
                           const vm::Vector3<float>& worldPos,
                           const vm::Quaternion<float>& worldRot,
                           BodyType type, uint32_t layer, uint32_t mask) {
    BodyDescriptor bd;
    bd.type           = type;
    bd.mass           = mass;
    bd.linearDamping  = 0.05f;
    bd.angularDamping = 0.1f;
    bd.restitution    = 0.1f;
    bd.friction       = 0.5f;
    bd.layer          = layer;
    bd.mask           = mask;
    bd.shape          = std::make_shared<CapsuleShape>(radius, halfH);
    bd.transform.position = worldPos;
    bd.transform.rotation = worldRot;
    return bd;
}

// Volume of a capsule (for mass distribution)
inline float capsuleVolume(float halfH, float r) {
    return std::numbers::pi_v<float> * r * r * (2.f * halfH + (4.f / 3.f) * r);
}

} // namespace

// ── RagdollFactory::makeDescriptor ───────────────────────────────────────────

ArticulatedBodyDescriptor RagdollFactory::makeDescriptor(const RagdollDescriptor& d) {
    ArticulatedBodyDescriptor desc;
    desc.links.resize(RagdollBone::Count);

    const auto& rt  = d.rootTransform;
    const auto& rot = rt.rotation;
    const auto& pos = rt.position;
    const auto  up  = rot.rotated(vm::Vector3<float>(0.f, 1.f, 0.f));
    const auto  dn  = up * -1.f;

    // ── Volume-proportional mass distribution ─────────────────────────────────

    float volumes[RagdollBone::Count];
    volumes[RagdollBone::Pelvis]    = capsuleVolume(d.pelvisRadius,    d.pelvisRadius);
    volumes[RagdollBone::SpineLow]  = capsuleVolume(d.spineLowH,       d.spineRadius);
    volumes[RagdollBone::SpineMid]  = capsuleVolume(d.spineMidH,       d.spineRadius);
    volumes[RagdollBone::SpineHigh] = capsuleVolume(d.spineHighH,      d.spineRadius);
    volumes[RagdollBone::Head]      = capsuleVolume(d.headRadius,       d.headRadius);
    volumes[RagdollBone::ShoulderL] = capsuleVolume(d.shoulderRadius,  d.shoulderRadius);
    volumes[RagdollBone::UpperArmL] = capsuleVolume(d.upperArmH,       d.upperArmRadius);
    volumes[RagdollBone::LowerArmL] = capsuleVolume(d.lowerArmH,       d.lowerArmRadius);
    volumes[RagdollBone::ShoulderR] = volumes[RagdollBone::ShoulderL];
    volumes[RagdollBone::UpperArmR] = volumes[RagdollBone::UpperArmL];
    volumes[RagdollBone::LowerArmR] = volumes[RagdollBone::LowerArmL];
    volumes[RagdollBone::UpperLegL] = capsuleVolume(d.upperLegH,       d.upperLegRadius);
    volumes[RagdollBone::LowerLegL] = capsuleVolume(d.lowerLegH,       d.lowerLegRadius);
    volumes[RagdollBone::UpperLegR] = volumes[RagdollBone::UpperLegL];
    volumes[RagdollBone::LowerLegR] = volumes[RagdollBone::LowerLegL];

    float totalVol = 0.f;
    for (float v : volumes) totalVol += v;
    const float massPerVol = (totalVol > 0.f) ? d.totalMass / totalVol : 0.f;

    auto mass = [&](int bone) { return volumes[bone] * massPerVol; };

    // Helper: position relative to root
    auto wpos = [&](const vm::Vector3<float>& offset) {
        return pos + rot.rotated(offset);
    };

    // Spine axis (world space)
    const auto spineAxis = up;
    const auto armAxisL  = rot.rotated(vm::Vector3<float>(-1.f, 0.f, 0.f));  // −X
    const auto armAxisR  = rot.rotated(vm::Vector3<float>( 1.f, 0.f, 0.f));  // +X
    const auto legAxisL  = dn;
    const auto legAxisR  = dn;

    // ── Bone positions (Y-up spine, legs going down) ──────────────────────────

    const float pelvisY  = 0.f;
    const float sl1      = d.spineLowH  + d.pelvisRadius;
    const float sl2      = sl1 + d.spineLowH  + d.spineMidH;
    const float sl3      = sl2 + d.spineMidH  + d.spineHighH;
    const float headY    = sl3 + d.spineHighH + d.headRadius;
    const float legStart = -(d.pelvisRadius + d.upperLegH);
    const float lowerL   = legStart - d.upperLegH - d.lowerLegH;
    const float armOffX  = d.spineRadius + d.shoulderRadius * 2.f + d.upperArmH;
    const float lowerArm = armOffX + d.upperArmH + d.lowerArmH;
    const float hipOffX  = d.pelvisRadius * 0.5f;

    // ── Define links ──────────────────────────────────────────────────────────

    auto& links = desc.links;

    // Pelvis (root)
    links[RagdollBone::Pelvis].parentIndex = -1;
    links[RagdollBone::Pelvis].body = capsuleBone(
        d.pelvisRadius, d.pelvisRadius, mass(RagdollBone::Pelvis),
        wpos({0.f, pelvisY, 0.f}), rot, d.rootBodyType, d.layer, d.mask);

    // SpineLow — ball-socket to Pelvis
    links[RagdollBone::SpineLow].parentIndex    = RagdollBone::Pelvis;
    links[RagdollBone::SpineLow].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::SpineLow].anchorInParent = { 0.f, d.pelvisRadius,  0.f };
    links[RagdollBone::SpineLow].anchorInSelf   = { 0.f, -d.spineLowH,   0.f };
    links[RagdollBone::SpineLow].body = capsuleBone(
        d.spineLowH, d.spineRadius, mass(RagdollBone::SpineLow),
        wpos({0.f, sl1, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // SpineMid — ball-socket to SpineLow
    links[RagdollBone::SpineMid].parentIndex    = RagdollBone::SpineLow;
    links[RagdollBone::SpineMid].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::SpineMid].anchorInParent = { 0.f,  d.spineLowH,  0.f };
    links[RagdollBone::SpineMid].anchorInSelf   = { 0.f, -d.spineMidH,  0.f };
    links[RagdollBone::SpineMid].body = capsuleBone(
        d.spineMidH, d.spineRadius, mass(RagdollBone::SpineMid),
        wpos({0.f, sl2, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // SpineHigh — ball-socket to SpineMid
    links[RagdollBone::SpineHigh].parentIndex    = RagdollBone::SpineMid;
    links[RagdollBone::SpineHigh].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::SpineHigh].anchorInParent = { 0.f,  d.spineMidH,   0.f };
    links[RagdollBone::SpineHigh].anchorInSelf   = { 0.f, -d.spineHighH,  0.f };
    links[RagdollBone::SpineHigh].body = capsuleBone(
        d.spineHighH, d.spineRadius, mass(RagdollBone::SpineHigh),
        wpos({0.f, sl3, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // Head — ball-socket to SpineHigh
    links[RagdollBone::Head].parentIndex    = RagdollBone::SpineHigh;
    links[RagdollBone::Head].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::Head].anchorInParent = { 0.f,  d.spineHighH, 0.f };
    links[RagdollBone::Head].anchorInSelf   = { 0.f, -d.headRadius, 0.f };
    links[RagdollBone::Head].body = capsuleBone(
        d.headRadius, d.headRadius, mass(RagdollBone::Head),
        wpos({0.f, headY, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // ShoulderL — fixed to SpineHigh (left side, -X)
    links[RagdollBone::ShoulderL].parentIndex    = RagdollBone::SpineHigh;
    links[RagdollBone::ShoulderL].jointType      = ArticulatedJointType::Fixed;
    links[RagdollBone::ShoulderL].anchorInParent = {-d.spineRadius, d.spineHighH * 0.5f, 0.f};
    links[RagdollBone::ShoulderL].anchorInSelf   = { d.shoulderRadius, 0.f, 0.f};
    links[RagdollBone::ShoulderL].body = capsuleBone(
        d.shoulderRadius, d.shoulderRadius, mass(RagdollBone::ShoulderL),
        wpos({-(d.spineRadius + d.shoulderRadius), sl3, 0.f}), rot,
        BodyType::Dynamic, d.layer, d.mask);

    // UpperArmL — ball-socket to ShoulderL
    links[RagdollBone::UpperArmL].parentIndex    = RagdollBone::ShoulderL;
    links[RagdollBone::UpperArmL].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::UpperArmL].anchorInParent = {-d.shoulderRadius, 0.f, 0.f};
    links[RagdollBone::UpperArmL].anchorInSelf   = { d.upperArmH, 0.f, 0.f};
    links[RagdollBone::UpperArmL].body = capsuleBone(
        d.upperArmH, d.upperArmRadius, mass(RagdollBone::UpperArmL),
        wpos({-armOffX, sl3, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // LowerArmL — hinge to UpperArmL (elbow)
    links[RagdollBone::LowerArmL].parentIndex       = RagdollBone::UpperArmL;
    links[RagdollBone::LowerArmL].jointType         = ArticulatedJointType::Hinge;
    links[RagdollBone::LowerArmL].anchorInParent    = {-d.upperArmH, 0.f, 0.f};
    links[RagdollBone::LowerArmL].anchorInSelf      = { d.lowerArmH, 0.f, 0.f};
    links[RagdollBone::LowerArmL].hingeAxisInParent = { 0.f, 0.f, 1.f };
    links[RagdollBone::LowerArmL].hingeAxisInSelf   = { 0.f, 0.f, 1.f };
    links[RagdollBone::LowerArmL].hingeLimitsActive = true;
    links[RagdollBone::LowerArmL].hingeLimitMin     = d.elbowLimitMin;
    links[RagdollBone::LowerArmL].hingeLimitMax     = d.elbowLimitMax;
    links[RagdollBone::LowerArmL].body = capsuleBone(
        d.lowerArmH, d.lowerArmRadius, mass(RagdollBone::LowerArmL),
        wpos({-lowerArm, sl3, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // ShoulderR — mirror of ShoulderL (+X)
    links[RagdollBone::ShoulderR].parentIndex    = RagdollBone::SpineHigh;
    links[RagdollBone::ShoulderR].jointType      = ArticulatedJointType::Fixed;
    links[RagdollBone::ShoulderR].anchorInParent = { d.spineRadius, d.spineHighH * 0.5f, 0.f};
    links[RagdollBone::ShoulderR].anchorInSelf   = {-d.shoulderRadius, 0.f, 0.f};
    links[RagdollBone::ShoulderR].body = capsuleBone(
        d.shoulderRadius, d.shoulderRadius, mass(RagdollBone::ShoulderR),
        wpos({ d.spineRadius + d.shoulderRadius, sl3, 0.f}), rot,
        BodyType::Dynamic, d.layer, d.mask);

    links[RagdollBone::UpperArmR].parentIndex    = RagdollBone::ShoulderR;
    links[RagdollBone::UpperArmR].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::UpperArmR].anchorInParent = { d.shoulderRadius, 0.f, 0.f};
    links[RagdollBone::UpperArmR].anchorInSelf   = {-d.upperArmH, 0.f, 0.f};
    links[RagdollBone::UpperArmR].body = capsuleBone(
        d.upperArmH, d.upperArmRadius, mass(RagdollBone::UpperArmR),
        wpos({ armOffX, sl3, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    links[RagdollBone::LowerArmR].parentIndex       = RagdollBone::UpperArmR;
    links[RagdollBone::LowerArmR].jointType         = ArticulatedJointType::Hinge;
    links[RagdollBone::LowerArmR].anchorInParent    = { d.upperArmH, 0.f, 0.f};
    links[RagdollBone::LowerArmR].anchorInSelf      = {-d.lowerArmH, 0.f, 0.f};
    links[RagdollBone::LowerArmR].hingeAxisInParent = { 0.f, 0.f, 1.f };
    links[RagdollBone::LowerArmR].hingeAxisInSelf   = { 0.f, 0.f, 1.f };
    links[RagdollBone::LowerArmR].hingeLimitsActive = true;
    links[RagdollBone::LowerArmR].hingeLimitMin     = d.elbowLimitMin;
    links[RagdollBone::LowerArmR].hingeLimitMax     = d.elbowLimitMax;
    links[RagdollBone::LowerArmR].body = capsuleBone(
        d.lowerArmH, d.lowerArmRadius, mass(RagdollBone::LowerArmR),
        wpos({ lowerArm, sl3, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // UpperLegL — ball-socket to Pelvis (-X hip offset, going down)
    links[RagdollBone::UpperLegL].parentIndex    = RagdollBone::Pelvis;
    links[RagdollBone::UpperLegL].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::UpperLegL].anchorInParent = {-hipOffX, -d.pelvisRadius, 0.f};
    links[RagdollBone::UpperLegL].anchorInSelf   = { 0.f,      d.upperLegH,   0.f};
    links[RagdollBone::UpperLegL].body = capsuleBone(
        d.upperLegH, d.upperLegRadius, mass(RagdollBone::UpperLegL),
        wpos({-hipOffX, legStart, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // LowerLegL — hinge to UpperLegL (knee)
    links[RagdollBone::LowerLegL].parentIndex       = RagdollBone::UpperLegL;
    links[RagdollBone::LowerLegL].jointType         = ArticulatedJointType::Hinge;
    links[RagdollBone::LowerLegL].anchorInParent    = { 0.f, -d.upperLegH, 0.f};
    links[RagdollBone::LowerLegL].anchorInSelf      = { 0.f,  d.lowerLegH, 0.f};
    links[RagdollBone::LowerLegL].hingeAxisInParent = { 1.f, 0.f, 0.f };
    links[RagdollBone::LowerLegL].hingeAxisInSelf   = { 1.f, 0.f, 0.f };
    links[RagdollBone::LowerLegL].hingeLimitsActive = true;
    links[RagdollBone::LowerLegL].hingeLimitMin     = d.kneeLimitMin;
    links[RagdollBone::LowerLegL].hingeLimitMax     = d.kneeLimitMax;
    links[RagdollBone::LowerLegL].body = capsuleBone(
        d.lowerLegH, d.lowerLegRadius, mass(RagdollBone::LowerLegL),
        wpos({-hipOffX, lowerL, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    // UpperLegR — mirror (+X hip)
    links[RagdollBone::UpperLegR].parentIndex    = RagdollBone::Pelvis;
    links[RagdollBone::UpperLegR].jointType      = ArticulatedJointType::BallSocket;
    links[RagdollBone::UpperLegR].anchorInParent = { hipOffX, -d.pelvisRadius, 0.f};
    links[RagdollBone::UpperLegR].anchorInSelf   = { 0.f,      d.upperLegH,   0.f};
    links[RagdollBone::UpperLegR].body = capsuleBone(
        d.upperLegH, d.upperLegRadius, mass(RagdollBone::UpperLegR),
        wpos({ hipOffX, legStart, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    links[RagdollBone::LowerLegR].parentIndex       = RagdollBone::UpperLegR;
    links[RagdollBone::LowerLegR].jointType         = ArticulatedJointType::Hinge;
    links[RagdollBone::LowerLegR].anchorInParent    = { 0.f, -d.upperLegH, 0.f};
    links[RagdollBone::LowerLegR].anchorInSelf      = { 0.f,  d.lowerLegH, 0.f};
    links[RagdollBone::LowerLegR].hingeAxisInParent = { 1.f, 0.f, 0.f };
    links[RagdollBone::LowerLegR].hingeAxisInSelf   = { 1.f, 0.f, 0.f };
    links[RagdollBone::LowerLegR].hingeLimitsActive = true;
    links[RagdollBone::LowerLegR].hingeLimitMin     = d.kneeLimitMin;
    links[RagdollBone::LowerLegR].hingeLimitMax     = d.kneeLimitMax;
    links[RagdollBone::LowerLegR].body = capsuleBone(
        d.lowerLegH, d.lowerLegRadius, mass(RagdollBone::LowerLegR),
        wpos({ hipOffX, lowerL, 0.f}), rot, BodyType::Dynamic, d.layer, d.mask);

    (void)spineAxis; (void)armAxisL; (void)armAxisR;
    (void)legAxisL;  (void)legAxisR;

    return desc;
}

// ── RagdollFactory::build ─────────────────────────────────────────────────────

RagdollBody RagdollFactory::build(const RagdollDescriptor& /*desc*/, ArticulatedBody ab) {
    return RagdollBody(std::move(ab));
}

} // namespace campello::physics
