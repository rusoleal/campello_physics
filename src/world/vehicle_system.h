#pragma once

#include <campello_physics/vehicle.h>
#include <campello_physics/body_pool.h>
#include <campello_physics/query.h>
#include <campello_physics/integrator.h>
#include <functional>
#include <optional>
#include <vector>

namespace campello::physics {

using VehicleRaycastFn = std::function<
    std::optional<RaycastHit>(const Ray&, const QueryFilter&)>;

struct VehicleRecord {
    VehicleDescriptor        desc;
    Body                     chassis;
    std::vector<WheelState>  wheelStates;
    float                    throttle = 0.f;
    float                    braking  = 0.f;
    float                    steering = 0.f;
    bool                     active   = false;
};

class VehicleSystem {
public:
    [[nodiscard]] VehicleBody create(const VehicleDescriptor& desc, Body chassisBody);
    void                      destroy(VehicleBody vb);

    // Pull controls from the VehicleBody handle into the record.
    void syncControls(const VehicleBody& vb);

    // Apply suspension + drive forces to all active vehicles.
    void update(BodyPool& pool,
                const IntegratorSettings& settings,
                float dt,
                const VehicleRaycastFn& raycast);

    [[nodiscard]] const WheelState& wheelState(const VehicleBody& vb, int i) const noexcept;
    [[nodiscard]] int                wheelCount(const VehicleBody& vb)        const noexcept;

    [[nodiscard]] bool empty() const noexcept { return m_activeCount == 0; }

private:
    std::vector<VehicleRecord> m_records;
    std::vector<uint32_t>      m_freeList;
    uint32_t                   m_activeCount = 0;

    void processWheel(BodyData& chassis,
                      VehicleRecord& rec,
                      int wheelIdx,
                      float dt,
                      const VehicleRaycastFn& raycast);
};

} // namespace campello::physics
