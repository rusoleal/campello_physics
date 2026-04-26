
#pragma once

#include <simd/simd.h>
#include <cmath>
#include <algorithm>

// Arcball orbit camera. Right-handed coordinate system (GLTF convention).
// Projection uses Metal NDC (z in [0, 1]).
struct Camera {
    float phi   = 0.0f;
    float theta = 0.35f;
    float radius = 8.0f;
    simd_float3 target = { 0.0f, 1.0f, 0.0f };
    float fovY  = M_PI / 4.0f;
    float nearZ = 0.05f;
    float farZ  = 500.0f;

    simd_float3 position() const {
        float ct = cosf(theta);
        return {
            target.x + radius * ct * sinf(phi),
            target.y + radius * sinf(theta),
            target.z + radius * ct * cosf(phi)
        };
    }

    simd_float4x4 viewMatrix() const {
        simd_float3 eye = position();
        simd_float3 f   = simd_normalize(target - eye);
        simd_float3 up  = { 0.0f, 1.0f, 0.0f };
        simd_float3 r   = simd_normalize(simd_cross(f, up));
        simd_float3 u   = simd_cross(r, f);
        return (simd_float4x4){{
            {  r.x,  u.x, -f.x, 0.0f },
            {  r.y,  u.y, -f.y, 0.0f },
            {  r.z,  u.z, -f.z, 0.0f },
            { -simd_dot(r, eye), -simd_dot(u, eye), simd_dot(f, eye), 1.0f }
        }};
    }

    simd_float4x4 projectionMatrix(float aspect) const {
        float ys = 1.0f / tanf(fovY * 0.5f);
        float xs = ys / aspect;
        float A  = farZ  / (nearZ - farZ);
        float B  = nearZ * farZ / (nearZ - farZ);
        return (simd_float4x4){{
            { xs,   0.0f, 0.0f,  0.0f },
            { 0.0f, ys,   0.0f,  0.0f },
            { 0.0f, 0.0f, A,    -1.0f },
            { 0.0f, 0.0f, B,     0.0f }
        }};
    }

    void orbit(float dPhi, float dTheta) {
        phi   += dPhi;
        theta  = std::clamp(theta + dTheta, -(float)M_PI / 2.0f + 0.01f,
                                             (float)M_PI / 2.0f - 0.01f);
    }
    void zoom(float delta) { radius = std::max(0.5f, radius * (1.0f - delta * 0.1f)); }
    void pan(float dx, float dy) {
        simd_float3 eye = position();
        simd_float3 f   = simd_normalize(target - eye);
        simd_float3 up  = { 0.0f, 1.0f, 0.0f };
        simd_float3 r   = simd_normalize(simd_cross(f, up));
        simd_float3 u   = simd_cross(r, f);
        float s = radius * 0.001f;
        target  = target - r * (dx * s) + u * (dy * s);
    }
};
