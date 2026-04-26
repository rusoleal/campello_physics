#include <metal_stdlib>
using namespace metal;

struct BodyState { float4 pos_p; float4 rot; float4 predPos_p; float4 predRot; float4 vel_p; float4 angVel_p; };
struct RenderTransform { float4 position_w; float4 rotation_xyzw; };
struct Params { float4 gravity_dt; float4 counts; };

kernel void write_render_transforms(
    device const BodyState*     states   [[buffer(0)]],
    device const uint*          indexMap [[buffer(1)]],
    device RenderTransform*     render   [[buffer(2)]],
    constant Params&            p        [[buffer(3)]],
    uint                        id       [[thread_position_in_grid]])
{
    if (id >= uint(p.counts.x)) return;
    uint slot = indexMap[id];
    render[slot].position_w    = float4(states[id].pos_p.xyz, 1.f);
    render[slot].rotation_xyzw = states[id].rot;
}
