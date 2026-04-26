#include <metal_stdlib>
using namespace metal;

struct BodyShape { float4 aabbMin_invMass; float4 aabbMax_invInertiaX; float4 inv_shape_p0; float4 params; };
struct Params    { float4 gravity_dt; float4 counts; };

// pairs buffer layout: [0]=count(atomic), [1]=pad, [2+k*2]=bodyA[k], [2+k*2+1]=bodyB[k]
kernel void broadphase_pairs(
    device const BodyShape* shapes    [[buffer(0)]],
    device uint*            pairsData [[buffer(1)]],
    constant Params&        p         [[buffer(2)]],
    uint                    idx       [[thread_position_in_grid]])
{
    uint n = uint(p.counts.x);
    if (n < 2u) return;
    uint totalPairs = n * (n - 1u) / 2u;
    if (idx >= totalPairs) return;

    uint i = uint(floor((-1.f + sqrt(1.f + 8.f*float(idx))) * 0.5f));
    uint j = idx - i*(2u*n - i - 1u)/2u + i + 1u;
    if (j >= n || j <= i) return;

    float3 minA = shapes[i].aabbMin_invMass.xyz;
    float3 maxA = shapes[i].aabbMax_invInertiaX.xyz;
    float3 minB = shapes[j].aabbMin_invMass.xyz;
    float3 maxB = shapes[j].aabbMax_invInertiaX.xyz;

    if (maxA.x < minB.x || maxB.x < minA.x) return;
    if (maxA.y < minB.y || maxB.y < minA.y) return;
    if (maxA.z < minB.z || maxB.z < minA.z) return;

    uint maxP = uint(p.counts.y);
    uint slot = atomic_fetch_add_explicit((device atomic_uint*)pairsData, 1u, memory_order_relaxed);
    if (slot >= maxP) return;

    pairsData[2u + slot * 2u]      = i;
    pairsData[2u + slot * 2u + 1u] = j;
}
