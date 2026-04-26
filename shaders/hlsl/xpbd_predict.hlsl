struct BodyState { float4 pos_p; float4 rot; float4 predPos_p; float4 predRot; float4 vel_p; float4 angVel_p; };
struct Params    { float4 gravity_dt; float4 counts; };

RWStructuredBuffer<BodyState> states : register(u0);
ConstantBuffer<Params>        p      : register(b1);

float4 qmul(float4 a, float4 b) {
    return float4(
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    );
}

[numthreads(64, 1, 1)]
void xpbd_predict(uint3 tid : SV_DispatchThreadID)
{
    uint id = tid.x;
    if (id >= (uint)p.counts.x) return;

    float3 gravity = p.gravity_dt.xyz;
    float  dt      = p.gravity_dt.w;

    float3 pos    = states[id].pos_p.xyz;
    float4 rot    = states[id].rot;
    float3 vel    = states[id].vel_p.xyz;
    float3 angVel = states[id].angVel_p.xyz;

    float3 predPos = pos + vel * dt + gravity * dt * dt;
    float4 omegaQ  = float4(angVel * 0.5f * dt, 0.f);
    float4 predRot = normalize(rot + qmul(omegaQ, rot));

    states[id].predPos_p.xyz = predPos;
    states[id].predRot       = predRot;
}
