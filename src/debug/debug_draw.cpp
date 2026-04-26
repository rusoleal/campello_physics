#include <campello_physics/debug_draw.h>
#include <cmath>
#include <numbers>

namespace campello::physics {

// ── IDebugDraw default implementations ────────────────────────────────────────

void IDebugDraw::drawAABB(const vm::Vector3<float>& mn,
                           const vm::Vector3<float>& mx,
                           DebugColor color) {
    // 8 corners
    vm::Vector3<float> c[8] = {
        {mn.x(), mn.y(), mn.z()}, {mx.x(), mn.y(), mn.z()},
        {mx.x(), mx.y(), mn.z()}, {mn.x(), mx.y(), mn.z()},
        {mn.x(), mn.y(), mx.z()}, {mx.x(), mn.y(), mx.z()},
        {mx.x(), mx.y(), mx.z()}, {mn.x(), mx.y(), mx.z()},
    };
    // Bottom face
    drawLine(c[0], c[1], color); drawLine(c[1], c[2], color);
    drawLine(c[2], c[3], color); drawLine(c[3], c[0], color);
    // Top face
    drawLine(c[4], c[5], color); drawLine(c[5], c[6], color);
    drawLine(c[6], c[7], color); drawLine(c[7], c[4], color);
    // Vertical edges
    drawLine(c[0], c[4], color); drawLine(c[1], c[5], color);
    drawLine(c[2], c[6], color); drawLine(c[3], c[7], color);
}

void IDebugDraw::drawSphere(const vm::Vector3<float>& centre,
                             float radius,
                             DebugColor color,
                             int segments) {
    const float step = 2.f * std::numbers::pi_v<float> / static_cast<float>(segments);
    for (int i = 0; i < segments; ++i) {
        const float a0 = step * i, a1 = step * (i + 1);
        const float c0 = std::cos(a0) * radius, s0 = std::sin(a0) * radius;
        const float c1 = std::cos(a1) * radius, s1 = std::sin(a1) * radius;

        // XY circle
        drawLine({ centre.x()+c0, centre.y()+s0, centre.z() },
                 { centre.x()+c1, centre.y()+s1, centre.z() }, color);
        // XZ circle
        drawLine({ centre.x()+c0, centre.y(), centre.z()+s0 },
                 { centre.x()+c1, centre.y(), centre.z()+s1 }, color);
        // YZ circle
        drawLine({ centre.x(), centre.y()+c0, centre.z()+s0 },
                 { centre.x(), centre.y()+c1, centre.z()+s1 }, color);
    }
}

void IDebugDraw::drawBox(const vm::Vector3<float>& centre,
                          const vm::Vector3<float>& half,
                          const vm::Quaternion<float>& rot,
                          DebugColor color) {
    // Eight local corners
    float sx[2] = {-half.x(), half.x()};
    float sy[2] = {-half.y(), half.y()};
    float sz[2] = {-half.z(), half.z()};

    vm::Vector3<float> c[8];
    int idx = 0;
    for (int xi = 0; xi < 2; ++xi)
        for (int yi = 0; yi < 2; ++yi)
            for (int zi = 0; zi < 2; ++zi)
                c[idx++] = centre + rot.rotated(vm::Vector3<float>(sx[xi], sy[yi], sz[zi]));

    // Edges: iterate axis pairs
    auto edge = [&](int a, int b) { drawLine(c[a], c[b], color); };
    edge(0,1); edge(1,3); edge(3,2); edge(2,0);  // near face
    edge(4,5); edge(5,7); edge(7,6); edge(6,4);  // far face
    edge(0,4); edge(1,5); edge(2,6); edge(3,7);  // connecting edges
}

void IDebugDraw::drawCapsule(const vm::Vector3<float>& centre,
                              const vm::Quaternion<float>& rot,
                              float radius,
                              float halfHeight,
                              DebugColor color,
                              int segments) {
    const float step = std::numbers::pi_v<float> / static_cast<float>(segments);
    const auto  up   = rot.rotated(vm::Vector3<float>(0.f, 1.f, 0.f));
    const auto  right= rot.rotated(vm::Vector3<float>(1.f, 0.f, 0.f));
    const auto  fwd  = rot.rotated(vm::Vector3<float>(0.f, 0.f, 1.f));
    const auto  topC = centre + up * halfHeight;
    const auto  botC = centre + up * (-halfHeight);

    // Cylinder lines (4 vertical)
    for (int i = 0; i < 4; ++i) {
        const float a = step * 2.f * i;
        const auto  r = right * (std::cos(a) * radius) + fwd * (std::sin(a) * radius);
        drawLine(topC + r, botC + r, color);
    }

    // Top and bottom circles
    for (int i = 0; i < segments * 2; ++i) {
        const float a0 = step * i, a1 = step * (i + 1);
        const auto  r0 = right*(std::cos(a0)*radius) + fwd*(std::sin(a0)*radius);
        const auto  r1 = right*(std::cos(a1)*radius) + fwd*(std::sin(a1)*radius);
        drawLine(topC + r0, topC + r1, color);
        drawLine(botC + r0, botC + r1, color);
    }

    // Hemisphere arcs (two perpendicular planes)
    for (int plane = 0; plane < 2; ++plane) {
        const auto tang = (plane == 0) ? right : fwd;
        for (int i = 0; i < segments; ++i) {
            // Top cap
            const float a0 = step * i, a1 = step * (i + 1);
            const auto  r0 = tang*(std::cos(a0)*radius) + up*(std::sin(a0)*radius);
            const auto  r1 = tang*(std::cos(a1)*radius) + up*(std::sin(a1)*radius);
            drawLine(topC + r0, topC + r1, color);
            // Bottom cap (sin goes negative)
            const auto  b0 = tang*(std::cos(a0)*radius) + up*(-std::sin(a0)*radius);
            const auto  b1 = tang*(std::cos(a1)*radius) + up*(-std::sin(a1)*radius);
            drawLine(botC + b0, botC + b1, color);
        }
    }
}

void IDebugDraw::drawArrow(const vm::Vector3<float>& base,
                            const vm::Vector3<float>& dir,
                            float length,
                            DebugColor color) {
    float len = std::sqrt(dir.x()*dir.x() + dir.y()*dir.y() + dir.z()*dir.z());
    if (len < 1e-10f) return;
    const auto tip = base + dir * (length / len);
    drawLine(base, tip, color);

    // Small arrowhead — two short lines in perpendicular directions
    const float headLen = length * 0.15f;
    vm::Vector3<float> ref = std::abs(dir.x()) < 0.9f
        ? vm::Vector3<float>(1.f,0.f,0.f) : vm::Vector3<float>(0.f,1.f,0.f);
    // perp1 = cross(dir, ref)
    const vm::Vector3<float> d = dir * (1.f / len);
    vm::Vector3<float> p1 = { d.y()*ref.z()-d.z()*ref.y(),
                               d.z()*ref.x()-d.x()*ref.z(),
                               d.x()*ref.y()-d.y()*ref.x() };
    float p1l = std::sqrt(p1.x()*p1.x()+p1.y()*p1.y()+p1.z()*p1.z());
    if (p1l < 1e-10f) return;
    p1 = p1 * (headLen / p1l);
    drawLine(tip, tip + (d * (-headLen)) + p1, color);
    drawLine(tip, tip + (d * (-headLen)) + p1 * -1.f, color);
}

void IDebugDraw::drawPoint(const vm::Vector3<float>& pt,
                            float size,
                            DebugColor color) {
    drawLine({pt.x()-size, pt.y(), pt.z()}, {pt.x()+size, pt.y(), pt.z()}, color);
    drawLine({pt.x(), pt.y()-size, pt.z()}, {pt.x(), pt.y()+size, pt.z()}, color);
    drawLine({pt.x(), pt.y(), pt.z()-size}, {pt.x(), pt.y(), pt.z()+size}, color);
}

} // namespace campello::physics
