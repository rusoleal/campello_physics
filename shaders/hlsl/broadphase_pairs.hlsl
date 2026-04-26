struct BodyShape { float4 aabbMin_invMass; float4 aabbMax_invInertiaX; float4 inv_shape_p0; float4 params; };
struct Params    { float4 gravity_dt; float4 counts; };

StructuredBuffer<BodyShape> shapes    : register(t0);
RWByteAddressBuffer         pairsData : register(u1);  // [0]=count, [8+k*8]=bodyA, [12+k*8]=bodyB
ConstantBuffer<Params>      p         : register(b2);

[numthreads(64, 1, 1)]
void broadphase_pairs(uint3 tid : SV_DispatchThreadID)
{
    uint idx = tid.x;
    uint n   = (uint)p.counts.x;
    if (n < 2u) return;
    uint totalPairs = n * (n - 1u) / 2u;
    if (idx >= totalPairs) return;

    uint i = (uint)floor((-1.f + sqrt(1.f + 8.f*(float)idx)) * 0.5f);
    uint j = idx - i*(2u*n - i - 1u)/2u + i + 1u;
    if (j >= n || j <= i) return;

    float3 minA = shapes[i].aabbMin_invMass.xyz;
    float3 maxA = shapes[i].aabbMax_invInertiaX.xyz;
    float3 minB = shapes[j].aabbMin_invMass.xyz;
    float3 maxB = shapes[j].aabbMax_invInertiaX.xyz;

    if (maxA.x < minB.x || maxB.x < minA.x) return;
    if (maxA.y < minB.y || maxB.y < minA.y) return;
    if (maxA.z < minB.z || maxB.z < minA.z) return;

    uint maxP = (uint)p.counts.y;
    uint slot;
    pairsData.InterlockedAdd(0, 1u, slot);
    if (slot >= maxP) return;

    // Write pair: offset = (2 + slot*2) * 4 bytes
    pairsData.Store((2u + slot * 2u) * 4u, i);
    pairsData.Store((2u + slot * 2u + 1u) * 4u, j);
}
