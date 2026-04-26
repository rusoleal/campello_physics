#pragma once

#include <campello_physics/buoyancy.h>
#include <campello_physics/body_pool.h>
#include <campello_physics/aabb.h>
#include <campello_physics/integrator.h>
#include <vector>
#include <cstdint>

namespace campello::physics {

struct BuoyancyVolumeData {
    BuoyancyDescriptor desc;
    bool               active = false;
};

// Applies Archimedes buoyancy + viscous drag to all dynamic bodies that overlap
// a registered fluid volume.  Call applyForces() once per substep, before
// broadphase, so forces are consumed by the integrator at the end of the substep.

class BuoyancySystem {
public:
    [[nodiscard]] BuoyancyVolume add(const BuoyancyDescriptor& desc);
    void                         remove(BuoyancyVolume vol);

    void applyForces(BodyPool& pool,
                     const IntegratorSettings& settings,
                     float dt) const;

    [[nodiscard]] bool empty() const noexcept { return m_activeCount == 0; }

private:
    std::vector<BuoyancyVolumeData> m_volumes;
    std::vector<uint32_t>           m_freeList;
    uint32_t                        m_activeCount = 0;

    // Returns the fraction of a body's volume submerged in this fluid volume.
    // Also returns the world-space AABB of the fluid for early-out tests.
    static float submersionFraction(const BodyData& body,
                                    const BuoyancyVolumeData& vol,
                                    float fluidTopY);

    static float sphereSubmergedVolume(float sphereR, float sphereCenterY, float fluidTopY);
};

} // namespace campello::physics
