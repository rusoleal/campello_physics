#include "gjk.h"
#include "narrowphase_utils.h"
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/shapes/cylinder_shape.h>
#include <campello_physics/shapes/convex_hull_shape.h>
#include <campello_physics/shapes/compound_shape.h>
#include <algorithm>
#include <limits>
#include <vector>

namespace campello::physics::detail {

// ── Support point in Minkowski difference space ───────────────────────────────

struct SupportPt {
    V3 v;   // Minkowski difference: pA - pB (world space)
    V3 a;   // World-space point on shape A
    V3 b;   // World-space point on shape B
};

static SupportPt supportMinkowski(
    const ShapeInstance& siA, const ShapeInstance& siB, const V3& dir)
{
    V3 localDirA = siA.transform.inverseTransformVector(dir);
    V3 pA        = siA.transform.transformPoint(siA.shape->support(localDirA));

    V3 localDirB = siB.transform.inverseTransformVector(neg3(dir));
    V3 pB        = siB.transform.transformPoint(siB.shape->support(localDirB));

    return { pA - pB, pA, pB };
}

// ── GJK simplex ───────────────────────────────────────────────────────────────

struct Simplex {
    SupportPt pts[4];
    int       count = 0;

    void push(const SupportPt& p) {
        for (int i = std::min(count, 3); i > 0; --i)
            pts[i] = pts[i - 1];
        pts[0] = p;
        if (count < 4) ++count;
    }
};

static bool sameDir(const V3& a, const V3& b) noexcept { return dot3(a, b) > 0.f; }

static bool lineSimplex(Simplex& s, V3& d) noexcept {
    // pts: [0]=A (newest), [1]=B
    const V3 AB = s.pts[1].v - s.pts[0].v;
    const V3 AO = neg3(s.pts[0].v);
    if (sameDir(AB, AO)) {
        d = cross3(cross3(AB, AO), AB);
        if (lenSq3(d) < 1e-14f) d = perp3(AB); // origin on line
    } else {
        s.count = 1;
        d = AO;
    }
    return false;
}

static bool triangleSimplex(Simplex& s, V3& d) noexcept {
    // pts: [0]=A (newest), [1]=B, [2]=C
    const V3 A = s.pts[0].v, B = s.pts[1].v, C = s.pts[2].v;
    const V3 AB = B - A, AC = C - A, AO = neg3(A);
    const V3 ABC = cross3(AB, AC);

    if (sameDir(cross3(ABC, AC), AO)) {
        if (sameDir(AC, AO)) {
            s.pts[1] = s.pts[2]; s.count = 2;
            d = cross3(cross3(AC, AO), AC);
        } else {
            s.count = 2;
            return lineSimplex(s, d);
        }
    } else if (sameDir(cross3(AB, ABC), AO)) {
        s.count = 2;
        return lineSimplex(s, d);
    } else {
        if (sameDir(ABC, AO)) {
            d = ABC;
        } else {
            std::swap(s.pts[1], s.pts[2]);
            d = neg3(ABC);
        }
    }
    return false;
}

static bool tetrahedronSimplex(Simplex& s, V3& d) noexcept {
    // pts: [0]=A (newest), [1]=B, [2]=C, [3]=D
    const V3 A = s.pts[0].v, B = s.pts[1].v, C = s.pts[2].v, D = s.pts[3].v;
    const V3 AB = B - A, AC = C - A, AD = D - A, AO = neg3(A);
    const V3 ABC = cross3(AB, AC);
    const V3 ACD = cross3(AC, AD);
    const V3 ADB = cross3(AD, AB);

    if (sameDir(ABC, AO)) {
        s.count = 3;
        return triangleSimplex(s, d);
    }
    if (sameDir(ACD, AO)) {
        s.pts[1] = s.pts[2]; s.pts[2] = s.pts[3]; s.count = 3;
        return triangleSimplex(s, d);
    }
    if (sameDir(ADB, AO)) {
        SupportPt savedB = s.pts[1];
        s.pts[1] = s.pts[3]; s.pts[2] = savedB; s.count = 3;
        return triangleSimplex(s, d);
    }
    return true; // origin inside tetrahedron
}

static bool doSimplex(Simplex& s, V3& d) noexcept {
    switch (s.count) {
    case 2: return lineSimplex(s, d);
    case 3: return triangleSimplex(s, d);
    case 4: return tetrahedronSimplex(s, d);
    default: return false;
    }
}

// ── EPA ───────────────────────────────────────────────────────────────────────

struct EPAFace {
    SupportPt v[3];
    V3        normal;
    float     dist;   // distance from origin to face plane (>= 0)
};

static EPAFace makeFace(const SupportPt& a, const SupportPt& b, const SupportPt& c) noexcept {
    EPAFace f;
    f.v[0] = a; f.v[1] = b; f.v[2] = c;
    const V3 n = cross3(b.v - a.v, c.v - a.v);
    const float nlen = len3(n);
    if (nlen < 1e-10f) {
        f.normal = V3(0.f, 1.f, 0.f);
        f.dist   = 0.f;
        return f;
    }
    f.normal = n * (1.f / nlen);
    f.dist   = dot3(f.normal, a.v);
    if (f.dist < 0.f) {
        f.normal = neg3(f.normal);
        f.dist   = -f.dist;
        std::swap(f.v[1], f.v[2]);
    }
    return f;
}

static ContactManifold buildManifold(const EPAFace& f) noexcept {
    // Project origin onto the face plane: p = dist * normal
    const V3 p = f.normal * f.dist;

    // Barycentric coordinates of p within the triangle
    const V3 e01 = f.v[1].v - f.v[0].v;
    const V3 e02 = f.v[2].v - f.v[0].v;
    const V3 ep  = p - f.v[0].v;

    const float d00 = dot3(e01, e01), d01 = dot3(e01, e02), d11 = dot3(e02, e02);
    const float d20 = dot3(ep, e01),  d21 = dot3(ep, e02);
    const float denom = d00 * d11 - d01 * d01;

    float bv, bw, bu;
    if (std::fabs(denom) < 1e-10f) {
        bu = bv = bw = 1.f / 3.f;
    } else {
        bv = (d11 * d20 - d01 * d21) / denom;
        bw = (d00 * d21 - d01 * d20) / denom;
        bu = 1.f - bv - bw;
        // Clamp and re-normalise
        bv = std::clamp(bv, 0.f, 1.f);
        bw = std::clamp(bw, 0.f, 1.f);
        bu = std::clamp(1.f - bv - bw, 0.f, 1.f);
        const float s = bu + bv + bw;
        if (s > 1e-10f) { bu /= s; bv /= s; bw /= s; }
        else              { bu = bv = bw = 1.f / 3.f; }
    }

    const V3 cA = f.v[0].a * bu + f.v[1].a * bv + f.v[2].a * bw;
    const V3 cB = f.v[0].b * bu + f.v[1].b * bv + f.v[2].b * bw;

    ContactManifold m;
    m.count = 1;
    m.points[0] = { (cA + cB) * 0.5f, f.normal, f.dist, 0.f, 0.f, 0.f };
    return m;
}

static ContactManifold epa(
    Simplex& s,
    const ShapeInstance& siA,
    const ShapeInstance& siB)
{
    std::vector<EPAFace> faces;
    faces.reserve(64);

    // Seed polytope with the 4 tetrahedron faces
    auto& p = s.pts;
    faces.push_back(makeFace(p[0], p[1], p[2]));
    faces.push_back(makeFace(p[0], p[2], p[3]));
    faces.push_back(makeFace(p[0], p[3], p[1]));
    faces.push_back(makeFace(p[1], p[3], p[2]));

    for (int iter = 0; iter < 64; ++iter) {
        // Closest face to origin
        int minIdx = 0;
        for (int i = 1; i < (int)faces.size(); ++i)
            if (faces[i].dist < faces[minIdx].dist) minIdx = i;

        const EPAFace& mf = faces[minIdx];
        const SupportPt newPt = supportMinkowski(siA, siB, mf.normal);
        const float newDist   = dot3(mf.normal, newPt.v);

        if (newDist - mf.dist < 1e-4f)
            return buildManifold(mf);

        // Expand polytope: collect horizon edges from visible faces
        struct Edge { SupportPt a, b; };
        std::vector<Edge> horizon;
        std::vector<EPAFace> newFaces;
        newFaces.reserve(faces.size() + 8);

        for (auto& f : faces) {
            if (dot3(f.normal, newPt.v - f.v[0].v) > 0.f) {
                // Face visible from new point — contribute its edges to horizon
                for (int i = 0; i < 3; ++i) {
                    Edge e = { f.v[i], f.v[(i + 1) % 3] };
                    bool found = false;
                    for (auto it = horizon.begin(); it != horizon.end(); ++it) {
                        // Reverse edge = shared with another visible face → remove
                        V3 d0 = it->a.v - e.b.v;
                        V3 d1 = it->b.v - e.a.v;
                        if (lenSq3(d0) < 1e-12f && lenSq3(d1) < 1e-12f) {
                            horizon.erase(it); found = true; break;
                        }
                    }
                    if (!found) horizon.push_back(e);
                }
            } else {
                newFaces.push_back(f);
            }
        }

        for (const auto& e : horizon)
            newFaces.push_back(makeFace(newPt, e.a, e.b));

        faces = std::move(newFaces);
        if (faces.empty()) break;
    }

    // Timeout fallback: return best found face
    int minIdx = 0;
    for (int i = 1; i < (int)faces.size(); ++i)
        if (faces[i].dist < faces[minIdx].dist) minIdx = i;
    return buildManifold(faces[minIdx]);
}

// ── Public entry point ────────────────────────────────────────────────────────

std::optional<ContactManifold> collideConvex(
    const ShapeInstance& a, const ShapeInstance& b)
{
    V3 d = a.transform.position - b.transform.position;
    if (lenSq3(d) < 1e-10f) d = V3(1.f, 0.f, 0.f);

    Simplex s;
    s.push(supportMinkowski(a, b, d));
    d = neg3(s.pts[0].v);

    for (int iter = 0; iter < 64; ++iter) {
        if (lenSq3(d) < 1e-14f) break;

        const SupportPt sp = supportMinkowski(a, b, d);
        if (dot3(sp.v, d) < 0.f) return std::nullopt;

        s.push(sp);

        if (doSimplex(s, d)) {
            // Need a full tetrahedron for EPA; try to build one if we have fewer points
            if (s.count < 4) {
                const V3 extra = perp3(d);
                const SupportPt ep = supportMinkowski(a, b, extra);
                if (dot3(ep.v, extra) > 0.f) { s.push(ep); }
                // Second try with a different perpendicular if still not 4 points
                if (s.count < 4) {
                    const SupportPt ep2 = supportMinkowski(a, b, cross3(d, extra));
                    if (lenSq3(ep2.v) > 1e-12f) s.push(ep2);
                }
            }
            if (s.count < 4) {
                // Degenerate (shapes touching on a 2D manifold): return zero-depth contact
                ContactManifold m;
                m.count = 1;
                m.points[0] = { a.transform.position, norm3(d), 0.f, 0.f, 0.f, 0.f };
                return m;
            }
            return epa(s, a, b);
        }
    }

    return std::nullopt;
}

} // namespace campello::physics::detail
