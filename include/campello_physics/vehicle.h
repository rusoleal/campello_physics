#pragma once

#include <campello_physics/body.h>
#include <campello_physics/shapes.h>
#include <cstdint>
#include <vector>

namespace campello::physics {

// ── WheelDescriptor ───────────────────────────────────────────────────────────

struct WheelDescriptor {
    // Attachment point in chassis local space
    vm::Vector3<float> attachmentLocal = { 0.f, 0.f, 0.f };

    float restLength      = 0.30f;    // metres — suspension travel
    float stiffness       = 30000.f;  // N/m
    float damping         =  2000.f;  // N·s/m
    float radius          =  0.35f;   // wheel radius (metres)
    float maxSteerAngle   =  0.f;     // radians; 0 = non-steering
    float lateralFriction =  1.2f;    // sideways friction coefficient
};

// ── VehicleDescriptor ─────────────────────────────────────────────────────────

struct VehicleDescriptor {
    BodyDescriptor               chassis;
    std::vector<WheelDescriptor> wheels;
    float maxEngineForce = 3000.f;   // N
    float maxBrakeForce  = 8000.f;   // N
};

// ── WheelState ────────────────────────────────────────────────────────────────
//
// Read-only snapshot updated by VehicleSystem each substep.
// Retrieve via PhysicsWorld::vehicleWheelState(vb, i).

struct WheelState {
    bool  isGrounded       = false;
    float suspensionLength = 0.f;
    float suspensionForce  = 0.f;
    vm::Vector3<float> contactPoint  = { 0.f, 0.f, 0.f };
    vm::Vector3<float> contactNormal = { 0.f, 1.f, 0.f };
};

// ── VehicleBody ───────────────────────────────────────────────────────────────
//
// Thin copyable handle.  Controls are public fields — set before world.step().
// Wheel state is queried via PhysicsWorld::vehicleWheelState(vb, i).

class VehicleBody {
public:
    VehicleBody() = default;

    [[nodiscard]] bool     isValid()     const noexcept { return m_id != kInvalid; }
    [[nodiscard]] uint32_t id()          const noexcept { return m_id; }
    [[nodiscard]] Body     chassisBody() const noexcept { return m_chassis; }

    // Controls — set each frame before world.step()
    float throttle = 0.f;   // [0, 1]
    float braking  = 0.f;   // [0, 1]
    float steering = 0.f;   // [-1, 1]

    bool operator==(const VehicleBody& o) const noexcept { return m_id == o.m_id; }
    bool operator!=(const VehicleBody& o) const noexcept { return m_id != o.m_id; }

private:
    static constexpr uint32_t kInvalid = ~uint32_t(0);
    explicit VehicleBody(uint32_t id, Body chassis) : m_id(id), m_chassis(chassis) {}
    friend class VehicleSystem;

    uint32_t m_id      = kInvalid;
    Body     m_chassis;
};

} // namespace campello::physics
