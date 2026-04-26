struct BodyState { float4 pos_p; float4 rot; float4 predPos_p; float4 predRot; float4 vel_p; float4 angVel_p; };
// inv_shape_p0: .x=invInertiaY .y=invInertiaZ .z=shapeType .w=param0
// params:       .x=param1 .y=param2 .z=param3
struct BodyShape { float4 aabbMin_invMass; float4 aabbMax_invInertiaX; float4 inv_shape_p0; float4 params; };
struct Params    { float4 gravity_dt; float4 counts; };

RWStructuredBuffer<BodyState>   states    : register(u0);
StructuredBuffer<BodyShape>     shapes    : register(t1);
ByteAddressBuffer               pairsData : register(t2);
ConstantBuffer<Params>          p         : register(b3);

float4 sphereSphereContact(float3 posA, float rA, float3 posB, float rB) {
    float3 d = posA - posB;
    float dist = length(d);
    float pen = rA + rB - dist;
    if (pen <= 0.f || dist < 1e-6f) return float4(0,0,0,-1);
    return float4(d / dist, pen);
}

float4 sphereBoxContact(float3 spherePos, float sphereR, float3 boxPos, float3 boxHalf) {
    float3 d = spherePos - boxPos;
    float3 closest = boxPos + clamp(d, -boxHalf, boxHalf);
    float3 toSphere = spherePos - closest;
    float dist = length(toSphere);
    float pen = sphereR - dist;
    if (pen <= 0.f) return float4(0,0,0,-1);
    float3 n = dist < 1e-6f ? float3(0,1,0) : toSphere / dist;
    return float4(n, pen);
}

[numthreads(64, 1, 1)]
void xpbd_contacts(uint3 tid : SV_DispatchThreadID)
{
    uint k         = tid.x;
    uint pairCount = pairsData.Load(0);
    if (k >= pairCount) return;

    uint a = pairsData.Load((2u + k * 2u)      * 4u);
    uint b = pairsData.Load((2u + k * 2u + 1u) * 4u);

    float wA   = shapes[a].aabbMin_invMass.w;
    float wB   = shapes[b].aabbMin_invMass.w;
    float wSum = wA + wB;
    if (wSum < 1e-10f) return;

    float stA = shapes[a].inv_shape_p0.z;
    float stB = shapes[b].inv_shape_p0.z;

    float3 posA = states[a].predPos_p.xyz;
    float3 posB = states[b].predPos_p.xyz;

    float3 n;
    float  pen;

    if (stA == 0.f && stB == 0.f) {
        float rA = shapes[a].inv_shape_p0.w;
        float rB = shapes[b].inv_shape_p0.w;
        float4 r = sphereSphereContact(posA, rA, posB, rB);
        if (r.w < 0.f) return;
        n = r.xyz; pen = r.w;
    } else if (stA == 0.f && stB == 1.f) {
        float rA    = shapes[a].inv_shape_p0.w;
        float3 boxH = float3(shapes[b].inv_shape_p0.w, shapes[b].params.x, shapes[b].params.y);
        float4 r = sphereBoxContact(posA, rA, posB, boxH);
        if (r.w < 0.f) return;
        n = r.xyz; pen = r.w;
    } else if (stA == 1.f && stB == 0.f) {
        float rB    = shapes[b].inv_shape_p0.w;
        float3 boxH = float3(shapes[a].inv_shape_p0.w, shapes[a].params.x, shapes[a].params.y);
        float4 r = sphereBoxContact(posB, rB, posA, boxH);
        if (r.w < 0.f) return;
        n = -r.xyz; pen = r.w;
    } else {
        float rA = shapes[a].inv_shape_p0.w;
        float rB = shapes[b].inv_shape_p0.w;
        float4 r = sphereSphereContact(posA, rA, posB, rB);
        if (r.w < 0.f) return;
        n = r.xyz; pen = r.w;
    }

    float dt         = p.gravity_dt.w;
    float compliance = p.counts.z;
    float alphaTilde = compliance / (dt * dt);
    float dLambda    = pen / (wSum + alphaTilde);

    const float js = 0.4f;
    states[a].predPos_p.xyz = posA + js * wA * dLambda * n;
    states[b].predPos_p.xyz = posB - js * wB * dLambda * n;
}
