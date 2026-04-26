struct BodyState { float4 pos_p; float4 rot; float4 predPos_p; float4 predRot; float4 vel_p; float4 angVel_p; };
struct RenderTransform { float4 position_w; float4 rotation_xyzw; };
struct Params { float4 gravity_dt; float4 counts; };

StructuredBuffer<BodyState>     states   : register(t0);
StructuredBuffer<uint>          indexMap : register(t1);
RWStructuredBuffer<RenderTransform> render : register(u2);
ConstantBuffer<Params>          p        : register(b3);

[numthreads(64, 1, 1)]
void write_render_transforms(uint3 tid : SV_DispatchThreadID)
{
    uint id = tid.x;
    if (id >= (uint)p.counts.x) return;
    uint slot = indexMap[id];
    render[slot].position_w    = float4(states[id].pos_p.xyz, 1.f);
    render[slot].rotation_xyzw = states[id].rot;
}
