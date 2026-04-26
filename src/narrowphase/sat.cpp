#include "sat.h"
#include "narrowphase_utils.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace campello::physics::detail {

// ── Sphere – Sphere ───────────────────────────────────────────────────────────

std::optional<ContactManifold> collideSpheres(
    const SphereShape& a, const Transform& ta,
    const SphereShape& b, const Transform& tb)
{
    const V3    diff  = ta.position - tb.position;
    const float dist2 = lenSq3(diff);
    const float rSum  = a.radius() + b.radius();

    if (dist2 >= rSum * rSum) return std::nullopt;

    const float dist = std::sqrt(dist2);
    const V3    normal = (dist < 1e-6f)
                       ? V3(1.f, 0.f, 0.f)
                       : diff * (1.f / dist);   // from B toward A

    const float depth  = rSum - dist;
    const V3    posA   = ta.position - normal * a.radius();
    const V3    posB   = tb.position + normal * b.radius();

    ContactManifold m;
    m.count = 1;
    m.points[0] = { (posA + posB) * 0.5f, normal, depth, 0.f, 0.f, 0.f };
    return m;
}

// ── Sphere – Box ──────────────────────────────────────────────────────────────

std::optional<ContactManifold> collideSphereBox(
    const SphereShape& sphere, const Transform& ts,
    const BoxShape&    box,    const Transform& tb,
    bool               sphereIsA)
{
    const float r   = sphere.radius();
    const V3&   he  = box.halfExtents();

    // Sphere center in box local space
    const V3 local = tb.inverseTransformPoint(ts.position);

    // Closest point on box (clamped) to sphere center
    const V3 closest = V3(
        std::clamp(local.x(), -he.x(), he.x()),
        std::clamp(local.y(), -he.y(), he.y()),
        std::clamp(local.z(), -he.z(), he.z())
    );

    const V3    delta  = local - closest;
    const float dist2  = lenSq3(delta);

    float depth;
    V3    localNormal;

    if (dist2 < 1e-8f) {
        // Sphere center inside box: push along minimum face penetration axis
        float depths[6] = {
            he.x() - local.x(), local.x() + he.x(),
            he.y() - local.y(), local.y() + he.y(),
            he.z() - local.z(), local.z() + he.z()
        };
        const V3 axes[6] = {
            V3(1,0,0), V3(-1,0,0),
            V3(0,1,0), V3(0,-1,0),
            V3(0,0,1), V3(0,0,-1)
        };
        int minIdx = 0;
        for (int i = 1; i < 6; ++i)
            if (depths[i] < depths[minIdx]) minIdx = i;

        localNormal = axes[minIdx]; // outward from box face toward sphere
        depth       = r + depths[minIdx];
    } else {
        const float dist = std::sqrt(dist2);
        if (dist >= r) return std::nullopt;
        localNormal = delta * (1.f / dist);
        depth       = r - dist;
    }

    // Transform normal to world space; ensure it points from B toward A
    const V3 worldNormal = sphereIsA
        ? tb.transformVector(localNormal)           // sphere=A, box=B: normal from box->sphere
        : neg3(tb.transformVector(localNormal));     // box=A, sphere=B: flip

    // Contact point: on box surface in world space
    const V3 contactOnBox    = tb.transformPoint(closest);
    const V3 contactOnSphere = ts.position - worldNormal * r;
    const V3 contactPos      = (contactOnBox + contactOnSphere) * 0.5f;

    ContactManifold m;
    m.count = 1;
    m.points[0] = { contactPos, worldNormal, depth, 0.f, 0.f, 0.f };
    return m;
}

// ── Sphere – Capsule ──────────────────────────────────────────────────────────

std::optional<ContactManifold> collideSphereCapsule(
    const SphereShape&  sphere,  const Transform& ts,
    const CapsuleShape& capsule, const Transform& tc,
    bool                sphereIsA)
{
    // Capsule axis in world space
    const V3 capTop    = tc.transformPoint(V3(0.f,  capsule.halfHeight(), 0.f));
    const V3 capBottom = tc.transformPoint(V3(0.f, -capsule.halfHeight(), 0.f));
    const V3 axis      = capTop - capBottom;
    const float axLen2 = lenSq3(axis);

    // Closest point on capsule axis segment to sphere center
    const V3 toSphere = ts.position - capBottom;
    const float t = (axLen2 < 1e-10f) ? 0.f
                  : std::clamp(dot3(toSphere, axis) / axLen2, 0.f, 1.f);
    const V3 closestOnAxis = capBottom + axis * t;

    const V3    diff  = ts.position - closestOnAxis;
    const float dist2 = lenSq3(diff);
    const float rSum  = sphere.radius() + capsule.radius();

    if (dist2 >= rSum * rSum) return std::nullopt;

    const float dist   = std::sqrt(dist2);
    const V3    normal = (dist < 1e-6f)
                       ? V3(1.f, 0.f, 0.f)
                       : diff * (1.f / dist);  // from capsule axis toward sphere
    const V3    worldNormal = sphereIsA ? normal : neg3(normal);
    const float depth       = rSum - dist;

    const V3 posOnCapsule = closestOnAxis + normal * capsule.radius();
    const V3 posOnSphere  = ts.position   - normal * sphere.radius();
    const V3 contactPos   = (posOnCapsule + posOnSphere) * 0.5f;

    ContactManifold m;
    m.count = 1;
    m.points[0] = { contactPos, worldNormal, depth, 0.f, 0.f, 0.f };
    return m;
}

// ── Capsule – Capsule ─────────────────────────────────────────────────────────

std::optional<ContactManifold> collideCapsuleCapsule(
    const CapsuleShape& a, const Transform& ta,
    const CapsuleShape& b, const Transform& tb)
{
    const V3 a0 = ta.transformPoint(V3(0.f, -a.halfHeight(), 0.f));
    const V3 a1 = ta.transformPoint(V3(0.f,  a.halfHeight(), 0.f));
    const V3 b0 = tb.transformPoint(V3(0.f, -b.halfHeight(), 0.f));
    const V3 b1 = tb.transformPoint(V3(0.f,  b.halfHeight(), 0.f));

    const V3    dA = a1 - a0;
    const V3    dB = b1 - b0;
    const V3    r  = a0 - b0;
    const float aa = dot3(dA, dA);
    const float ee = dot3(dB, dB);
    const float f  = dot3(dB, r);

    float s, t;

    if (aa < 1e-10f && ee < 1e-10f) {
        s = t = 0.f;
    } else if (aa < 1e-10f) {
        s = 0.f;
        t = std::clamp(f / ee, 0.f, 1.f);
    } else {
        const float c = dot3(dA, r);
        if (ee < 1e-10f) {
            t = 0.f;
            s = std::clamp(-c / aa, 0.f, 1.f);
        } else {
            const float bDot  = dot3(dA, dB);
            const float denom = aa * ee - bDot * bDot;
            s = (std::fabs(denom) > 1e-10f)
              ? std::clamp((bDot * f - c * ee) / denom, 0.f, 1.f)
              : 0.f;
            t = (bDot * s + f) / ee;
            if (t < 0.f) { t = 0.f; s = std::clamp(-c / aa, 0.f, 1.f); }
            else if (t > 1.f) { t = 1.f; s = std::clamp((bDot - c) / aa, 0.f, 1.f); }
        }
    }

    const V3    closestA = a0 + dA * s;
    const V3    closestB = b0 + dB * t;
    const V3    diff     = closestA - closestB;
    const float dist2    = lenSq3(diff);
    const float rSum     = a.radius() + b.radius();

    if (dist2 >= rSum * rSum) return std::nullopt;

    const float dist   = std::sqrt(dist2);
    const V3    normal = (dist < 1e-6f) ? perp3(norm3(dA)) : diff * (1.f / dist);
    const float depth  = rSum - dist;

    const V3 posA = closestA - normal * a.radius();
    const V3 posB = closestB + normal * b.radius();

    ContactManifold m;
    m.count = 1;
    m.points[0] = { (posA + posB) * 0.5f, normal, depth, 0.f, 0.f, 0.f };
    return m;
}

// ── Box – Box (SAT, up to 4 contact points) ───────────────────────────────────

// Sutherland-Hodgman polygon clip against a single half-space: dot(p, n) <= d
static void clipPolygon(
    const std::vector<V3>& in,
    const V3& normal, float d,
    std::vector<V3>& out)
{
    out.clear();
    if (in.empty()) return;
    const int sz = static_cast<int>(in.size());
    for (int i = 0; i < sz; ++i) {
        const V3& a  = in[i];
        const V3& b  = in[(i + 1) % sz];
        const float da = dot3(a, normal) - d;
        const float db = dot3(b, normal) - d;
        if (da <= 0.f) out.push_back(a);
        if ((da < 0.f) != (db < 0.f)) {
            const float t = da / (da - db);
            out.push_back(a + (b - a) * t);
        }
    }
}

std::optional<ContactManifold> collideBoxBox(
    const BoxShape& A, const Transform& tA,
    const BoxShape& B, const Transform& tB)
{
    const V3& heA = A.halfExtents();
    const V3& heB = B.halfExtents();

    // World-space axes of each box
    const V3 axA[3] = {
        tA.transformVector(V3(1,0,0)),
        tA.transformVector(V3(0,1,0)),
        tA.transformVector(V3(0,0,1))
    };
    const V3 axB[3] = {
        tB.transformVector(V3(1,0,0)),
        tB.transformVector(V3(0,1,0)),
        tB.transformVector(V3(0,0,1))
    };

    const float heAv[3] = { heA.x(), heA.y(), heA.z() };
    const float heBv[3] = { heB.x(), heB.y(), heB.z() };

    const V3 T = tB.position - tA.position; // center of B relative to A

    // Project a box onto an axis: returns the half-span
    auto projBox = [](const V3 axes[3], const float he[3], const V3& axis) -> float {
        return std::fabs(dot3(axes[0], axis)) * he[0]
             + std::fabs(dot3(axes[1], axis)) * he[1]
             + std::fabs(dot3(axes[2], axis)) * he[2];
    };

    float minOverlap = std::numeric_limits<float>::max();
    int   minAxisIdx = -1;
    V3    minAxis;  // contact normal: from B toward A

    // Test one SAT axis; return false immediately if it's a separating axis
    auto testAxis = [&](const V3& rawAxis, int idx) -> bool {
        const float pA    = projBox(axA, heAv, rawAxis);
        const float pB    = projBox(axB, heBv, rawAxis);
        const float dist  = std::fabs(dot3(T, rawAxis));
        const float ovlp  = pA + pB - dist;
        if (ovlp <= 0.f) return false;
        if (ovlp < minOverlap) {
            minOverlap = ovlp;
            minAxisIdx = idx;
            // minAxis points from B toward A
            const float sign = dot3(T, rawAxis) > 0.f ? -1.f : 1.f;
            minAxis = rawAxis * sign;
        }
        return true;
    };

    // 6 face axes
    for (int i = 0; i < 3; ++i) if (!testAxis(axA[i], i))     return std::nullopt;
    for (int i = 0; i < 3; ++i) if (!testAxis(axB[i], 3 + i)) return std::nullopt;

    // 9 edge-edge cross-product axes
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const V3 cx = cross3(axA[i], axB[j]);
            const float cxLen = len3(cx);
            if (cxLen < 1e-6f) continue; // parallel edges
            if (!testAxis(cx * (1.f / cxLen), 6 + i * 3 + j)) return std::nullopt;
        }
    }

    // All axes overlapping — collision detected.
    // Determine contact type: face (0..5) or edge-edge (6..14)

    if (minAxisIdx < 6) {
        // ── Face contact ──────────────────────────────────────────────────────
        const bool aIsRef = (minAxisIdx < 3);
        const int  refFaceIdx = minAxisIdx % 3;

        const V3*    refAxes = aIsRef ? axA : axB;
        const float* refHe   = aIsRef ? heAv : heBv;
        const V3*    incAxes = aIsRef ? axB : axA;
        const float* incHe   = aIsRef ? heBv : heAv;
        const V3&    refPos  = aIsRef ? tA.position : tB.position;

        // outward normal of reference face (toward incident box)
        // aIsRef: minAxis = from B to A, so from A toward B = neg(minAxis)
        // bIsRef: minAxis = from B to A = from incident(A) toward ref(B)
        const V3 refNormalOut = aIsRef ? neg3(minAxis) : minAxis;

        const V3 refFaceCenter = refPos + refNormalOut * refHe[refFaceIdx];
        const int sideA = (refFaceIdx + 1) % 3;
        const int sideB = (refFaceIdx + 2) % 3;

        // Direction toward reference from incident box
        const V3 incDir = aIsRef ? minAxis : neg3(minAxis);

        // Most-antiparallel face on incident box
        float bestDot = -1e30f;
        int   incFaceAxis = 0;
        float incFaceSign = 1.f;
        for (int i = 0; i < 3; ++i) {
            const float d = std::fabs(dot3(incAxes[i], incDir));
            if (d > bestDot) {
                bestDot    = d;
                incFaceAxis = i;
                incFaceSign = dot3(incAxes[i], incDir) > 0.f ? 1.f : -1.f;
            }
        }

        const V3&    incPos       = aIsRef ? tB.position : tA.position;
        const V3     incFaceCenter = incPos
            + incAxes[incFaceAxis] * (incFaceSign * incHe[incFaceAxis]);
        const int    incS0 = (incFaceAxis + 1) % 3;
        const int    incS1 = (incFaceAxis + 2) % 3;

        // 4 vertices of incident face (world space)
        std::vector<V3> poly = {
            incFaceCenter + incAxes[incS0] *  incHe[incS0] + incAxes[incS1] *  incHe[incS1],
            incFaceCenter + incAxes[incS0] *  incHe[incS0] + incAxes[incS1] * -incHe[incS1],
            incFaceCenter + incAxes[incS0] * -incHe[incS0] + incAxes[incS1] * -incHe[incS1],
            incFaceCenter + incAxes[incS0] * -incHe[incS0] + incAxes[incS1] *  incHe[incS1],
        };

        // Clip against 4 side planes of reference face.
        // Each plane: dot(p, clipN[k]) <= clipD[k]
        std::array<V3,    4> clipN = {
             refAxes[sideA], neg3(refAxes[sideA]),
             refAxes[sideB], neg3(refAxes[sideB])
        };
        std::array<float, 4> clipD = {
            dot3(refFaceCenter, refAxes[sideA])  + refHe[sideA],
            -(dot3(refFaceCenter, refAxes[sideA]) - refHe[sideA]),
            dot3(refFaceCenter, refAxes[sideB])  + refHe[sideB],
            -(dot3(refFaceCenter, refAxes[sideB]) - refHe[sideB]),
        };
        // Clip against half-space: dot(p, clipN[k]) <= clipD[k]

        std::vector<V3> tmp;
        for (int k = 0; k < 4; ++k) {
            clipPolygon(poly, clipN[k], clipD[k], tmp);
            std::swap(poly, tmp);
            if (poly.empty()) return std::nullopt;
        }

        // Keep vertices on the penetrating side of the reference face
        const float refFacePlaneDist = dot3(refNormalOut, refFaceCenter);

        ContactManifold m;
        for (const auto& p : poly) {
            const float depth = refFacePlaneDist - dot3(refNormalOut, p);
            if (depth > -1e-4f && m.count < ContactManifold::kMaxPoints) {
                const V3 contactPos = p + refNormalOut * (depth * 0.5f);
                m.points[m.count++] = { contactPos, minAxis, std::max(0.f, depth),
                                        0.f, 0.f, 0.f };
            }
        }

        if (m.count == 0) return std::nullopt;
        return m;

    } else {
        // ── Edge-edge contact ─────────────────────────────────────────────────
        const int edgeAIdx = (minAxisIdx - 6) / 3;
        const int edgeBIdx = (minAxisIdx - 6) % 3;

        // Pick which edge on each box by projecting the center-to-center vector
        // onto the two perpendicular axes of each box.
        const V3 negMin = neg3(minAxis); // direction from A toward B

        auto edgeCenter = [](
            const V3& boxPos, const V3 axes[3], const float he[3],
            int edgeAxis, const V3& toward) -> V3
        {
            const int s0 = (edgeAxis + 1) % 3;
            const int s1 = (edgeAxis + 2) % 3;
            return boxPos
                + axes[s0] * (dot3(axes[s0], toward) >= 0.f ?  he[s0] : -he[s0])
                + axes[s1] * (dot3(axes[s1], toward) >= 0.f ?  he[s1] : -he[s1]);
        };

        const V3 ecA = edgeCenter(tA.position, axA, heAv, edgeAIdx, negMin);
        const V3 ecB = edgeCenter(tB.position, axB, heBv, edgeBIdx, minAxis);

        // Closest points on the two finite edges
        const V3    dA    = axA[edgeAIdx];
        const V3    dB    = axB[edgeBIdx];
        const V3    rVec  = ecA - ecB;
        const float bDot  = dot3(dA, dB);
        const float denom = 1.f - bDot * bDot; // |dA|=|dB|=1

        float s = 0.f, t = 0.f;
        if (std::fabs(denom) > 1e-10f) {
            s = std::clamp((bDot * dot3(dB, rVec) - dot3(dA, rVec)) / denom,
                           -heAv[edgeAIdx], heAv[edgeAIdx]);
        }
        t = (dot3(dB, rVec) + bDot * s);
        t = std::clamp(t, -heBv[edgeBIdx], heBv[edgeBIdx]);
        if (std::fabs(denom) > 1e-10f)
            s = std::clamp((bDot * t - dot3(dA, rVec)) / denom,
                           -heAv[edgeAIdx], heAv[edgeAIdx]);

        const V3 closestA = ecA + dA * s;
        const V3 closestB = ecB + dB * t;

        ContactManifold m;
        m.count = 1;
        m.points[0] = { (closestA + closestB) * 0.5f, minAxis, minOverlap,
                        0.f, 0.f, 0.f };
        return m;
    }
}

} // namespace campello::physics::detail
