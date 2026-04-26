#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/vehicle.h>
#include <cstdio>
#include <cmath>

namespace phys = campello::physics;
namespace vm   = systems::leal::vector_math;

int main() {
    phys::PhysicsWorld world;
    world.setGravity({0.f, -9.81f, 0.f});
    world.setSubsteps(4);

    // Floor
    {
        phys::BodyDescriptor d;
        d.type  = phys::BodyType::Static;
        d.shape = std::make_shared<phys::BoxShape>(vm::Vector3<float>(50.f, 0.5f, 50.f));
        d.transform.position = {0.f, -0.5f, 0.f};
        world.createBody(d);
    }

    // Vehicle — same parameters as the example
    phys::VehicleDescriptor vd;
    {
        phys::BodyDescriptor& cd = vd.chassis;
        cd.type           = phys::BodyType::Dynamic;
        cd.shape          = std::make_shared<phys::BoxShape>(vm::Vector3<float>(0.9f, 0.25f, 1.8f));
        cd.mass           = 1000.f;
        cd.transform.position = {0.f, 1.f, 0.f};
        cd.linearDamping  = 0.2f;
        cd.angularDamping = 4.0f;
    }
    vd.maxEngineForce = 3000.f;
    vd.maxBrakeForce  = 8000.f;

    const float kWheelAttachX[4] = { -0.85f,  0.85f, -0.85f,  0.85f };
    const float kWheelAttachZ[4] = {  1.4f,   1.4f,  -1.4f,  -1.4f  };
    const float kWheelAttachY    = -0.25f;

    for (int i = 0; i < 4; ++i) {
        phys::WheelDescriptor wd;
        wd.attachmentLocal  = { kWheelAttachX[i], kWheelAttachY, kWheelAttachZ[i] };
        wd.restLength       = 0.45f;
        wd.radius           = 0.35f;
        wd.stiffness        = 50000.f;
        wd.damping          = 12000.f;
        wd.lateralFriction  = 1.2f;
        wd.maxSteerAngle    = (i < 2) ? 0.45f : 0.f;
        vd.wheels.push_back(wd);
    }

    phys::VehicleBody vehicle = world.createVehicle(vd);

    const float dt       = 1.f / 60.f;
    const float duration = 20.f;
    const float printInterval = 0.5f;
    float       nextPrint = 0.f;

    // Apply throttle for first 5 seconds, then brake
    for (float t = 0.f; t < duration; t += dt) {
        vehicle.throttle = (t < 5.f) ? 1.f : 0.f;
        vehicle.braking  = (t >= 5.f && t < 8.f) ? 0.8f : 0.f;
        world.syncVehicleControls(vehicle);
        world.step(dt);

        if (t >= nextPrint) {
            const auto& cd = world.bodyPool().get(vehicle.chassisBody().id());
            const auto& p  = cd.transform.position;

            // Chassis orientation as Euler-ish angles (just X and Z tilt)
            const auto& q  = cd.transform.rotation;
            // Extract roll (rotation about Z in local frame)
            float roll  = std::atan2(2.f*(q.data[3]*q.data[0] + q.data[1]*q.data[2]),
                                     1.f - 2.f*(q.data[0]*q.data[0] + q.data[1]*q.data[1]));
            float pitch = std::asin(2.f*(q.data[3]*q.data[1] - q.data[2]*q.data[0]));

            std::printf("t=%5.1f  chassis y=%.4f  roll=%.3f°  pitch=%.3f°  vel_y=%.4f\n",
                        t,
                        p.y(),
                        roll  * 180.f / 3.14159f,
                        pitch * 180.f / 3.14159f,
                        cd.linearVelocity.y());

            for (int i = 0; i < 4; ++i) {
                const phys::WheelState& ws = world.vehicleWheelState(vehicle, i);
                std::printf("          wheel[%d] grounded=%d  suspLen=%.4f  suspForce=%.1f\n",
                            i, ws.isGrounded ? 1 : 0, ws.suspensionLength, ws.suspensionForce);
            }
            std::printf("\n");

            nextPrint += printInterval;
        }
    }

    return 0;
}
