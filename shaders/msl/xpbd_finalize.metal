#include <metal_stdlib>
using namespace metal;

struct BodyState { float4 pos_p; float4 rot; float4 predPos_p; float4 predRot; float4 vel_p; float4 angVel_p; };
struct Params    { float4 gravity_dt; float4 counts; };

float4 qmul(float4 a, float4 b) {
    return float4(
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    );
}

kernel void xpbd_finalize(
    device BodyState* states [[buffer(0)]],
    constant Params&  p      [[buffer(1)]],
    uint              id     [[thread_position_in_grid]])
{
    if (id >= uint(p.counts.x)) return;

    float dt = p.gravity_dt.w;

    float3 pos     = states[id].pos_p.xyz;
    float4 rot     = states[id].rot;
    float3 predPos = states[id].predPos_p.xyz;
    float4 predRot = states[id].predRot;

    float3 newVel = (predPos - pos) / dt;

    // Angular velocity from quaternion difference: dq = predRot * conj(rot)
    float4 rotConj = float4(-rot.x, -rot.y, -rot.z, rot.w);
    float4 dq      = qmul(predRot, rotConj);
    if (dq.w < 0.f) dq = -dq;
    float3 newAngVel = 2.f * dq.xyz / dt;

    states[id].pos_p.xyz    = predPos;
    states[id].rot          = normalize(predRot);
    states[id].vel_p.xyz    = newVel;
    states[id].angVel_p.xyz = newAngVel;
}
