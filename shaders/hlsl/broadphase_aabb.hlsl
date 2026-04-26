struct BodyState { float4 pos_p; float4 rot; float4 predPos_p; float4 predRot; float4 vel_p; float4 angVel_p; };
struct BodyShape { float4 aabbMin_invMass; float4 aabbMax_invInertiaX; float4 inv_shape_p0; float4 params; };
struct Params    { float4 gravity_dt; float4 counts; };

StructuredBuffer<BodyState>   states : register(t0);
RWStructuredBuffer<BodyShape> shapes : register(u1);
ConstantBuffer<Params>        p      : register(b2);

[numthreads(64, 1, 1)]
void broadphase_aabb(uint3 tid : SV_DispatchThreadID)
{
    uint id = tid.x;
    if (id >= (uint)p.counts.x) return;

    float3 pos = states[id].pos_p.xyz;
    float4 rot = states[id].rot;
    float  st  = shapes[id].inv_shape_p0.z;
    float  p0  = shapes[id].inv_shape_p0.w;
    float  p1  = shapes[id].params.x;
    float  p2  = shapes[id].params.y;

    float3 extent;
    if (st < 0.5f) {
        extent = float3(p0, p0, p0);
    } else if (st < 1.5f) {
        float qx=rot.x, qy=rot.y, qz=rot.z, qw=rot.w;
        float hx=p0, hy=p1, hz=p2;
        float m00 = abs(1.f - 2.f*(qy*qy+qz*qz));
        float m01 = abs(2.f*(qx*qy - qz*qw));
        float m02 = abs(2.f*(qx*qz + qy*qw));
        float m10 = abs(2.f*(qx*qy + qz*qw));
        float m11 = abs(1.f - 2.f*(qx*qx+qz*qz));
        float m12 = abs(2.f*(qy*qz - qx*qw));
        float m20 = abs(2.f*(qx*qz - qy*qw));
        float m21 = abs(2.f*(qy*qz + qx*qw));
        float m22 = abs(1.f - 2.f*(qx*qx+qy*qy));
        extent.x = m00*hx + m01*hy + m02*hz;
        extent.y = m10*hx + m11*hy + m12*hz;
        extent.z = m20*hx + m21*hy + m22*hz;
    } else {
        float qx=rot.x, qy=rot.y, qz=rot.z, qw=rot.w;
        float3 yAxis = float3(2.f*(qx*qy+qz*qw), 1.f-2.f*(qx*qx+qz*qz), 2.f*(qy*qz-qx*qw));
        float3 h = abs(yAxis) * p1;
        extent = h + float3(p0, p0, p0);
    }

    shapes[id].aabbMin_invMass.xyz     = pos - extent;
    shapes[id].aabbMax_invInertiaX.xyz = pos + extent;
}
