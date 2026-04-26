#include <campello_physics/physics_world.h>
#include "gpu/gpu_backend.h"
#ifdef CAMPELLO_PHYSICS_GPU_ENABLED
#include "gpu/gpu_physics_backend.h"
#endif
#include "buoyancy_system.h"
#include "articulated_body_system.h"
#include <campello_physics/ragdoll.h>
#include "vehicle_system.h"
#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

namespace campello::physics {

// ── Internal helpers ──────────────────────────────────────────────────────────

namespace {

inline float dot3(const vm::Vector3<float>& a, const vm::Vector3<float>& b) {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
}

inline vm::Vector3<float> cross3(const vm::Vector3<float>& a, const vm::Vector3<float>& b) {
    return vm::Vector3<float>(
        a.y()*b.z() - a.z()*b.y(),
        a.z()*b.x() - a.x()*b.z(),
        a.x()*b.y() - a.y()*b.x());
}

inline float lenSq3(const vm::Vector3<float>& v) {
    return v.x()*v.x() + v.y()*v.y() + v.z()*v.z();
}

inline vm::Vector3<float> norm3(const vm::Vector3<float>& v) {
    float l = std::sqrt(lenSq3(v));
    return l > 1e-20f ? v * (1.f / l) : vm::Vector3<float>(1.f, 0.f, 0.f);
}

inline vm::Vector3<float> applyInvInertia(const BodyData& d, const vm::Vector3<float>& v) {
    const auto& R   = d.transform.rotation;
    const auto  loc = R.conjugated().rotated(v);
    const auto& inv = d.invInertiaTensorLocal;
    return R.rotated(vm::Vector3<float>(
        loc.x() * inv.x(), loc.y() * inv.y(), loc.z() * inv.z()));
}

inline void perp3(const vm::Vector3<float>& n,
                  vm::Vector3<float>& t0, vm::Vector3<float>& t1) {
    float ax = std::abs(n.x()), ay = std::abs(n.y()), az = std::abs(n.z());
    vm::Vector3<float> ref = (ax <= ay && ax <= az)
        ? vm::Vector3<float>(1.f, 0.f, 0.f)
        : (ay <= az ? vm::Vector3<float>(0.f, 1.f, 0.f)
                    : vm::Vector3<float>(0.f, 0.f, 1.f));
    t0 = norm3(cross3(n, ref));
    t1 = cross3(n, t0);
}

inline float computeEffMassRow(const BodyData& da, const BodyData& db,
                                const vm::Vector3<float>& Jva,
                                const vm::Vector3<float>& Jwa,
                                const vm::Vector3<float>& Jvb,
                                const vm::Vector3<float>& Jwb) {
    float K = dot3(Jva, Jva) * da.invMass + dot3(Jvb, Jvb) * db.invMass
            + dot3(Jwa, applyInvInertia(da, Jwa))
            + dot3(Jwb, applyInvInertia(db, Jwb));
    return K > 1e-12f ? 1.f / K : 0.f;
}

inline void applyImpulseRow(BodyData& da, BodyData& db,
                             const vm::Vector3<float>& Jva,
                             const vm::Vector3<float>& Jwa,
                             const vm::Vector3<float>& Jvb,
                             const vm::Vector3<float>& Jwb,
                             float dL) {
    da.linearVelocity  = da.linearVelocity  + Jva * (dL * da.invMass);
    da.angularVelocity = da.angularVelocity + applyInvInertia(da, Jwa * dL);
    db.linearVelocity  = db.linearVelocity  + Jvb * (dL * db.invMass);
    db.angularVelocity = db.angularVelocity + applyInvInertia(db, Jwb * dL);
}

inline void applyPosImpulse(BodyData& d,
                              const vm::Vector3<float>& Jv,
                              const vm::Vector3<float>& /*Jw*/,
                              float lam) {
    if (d.invMass == 0.f || d.isSleeping) return;
    d.transform.position = d.transform.position + Jv * (lam * d.invMass);
}

} // namespace

// ── PhysicsWorld ──────────────────────────────────────────────────────────────

PhysicsWorld::PhysicsWorld()
    : m_buoyancySystem(std::make_unique<BuoyancySystem>())
    , m_articulatedBodySystem(std::make_unique<ArticulatedBodySystem>())
    , m_vehicleSystem(std::make_unique<VehicleSystem>()) {
    m_constraintSolver.iterations = constraintIterations;
}

PhysicsWorld::~PhysicsWorld() = default;

std::shared_ptr<void> PhysicsWorld::gpuRenderBuffer() const noexcept {
    if (!m_gpuBackend) return nullptr;
    return m_gpuBackend->renderBuffer();
}

void PhysicsWorld::setBackend(PhysicsBackend backend, std::shared_ptr<void> gpuDevice) {
    m_backend = backend;
#ifdef CAMPELLO_PHYSICS_GPU_ENABLED
    if ((backend == PhysicsBackend::Gpu || backend == PhysicsBackend::Auto) && !m_gpuBackend) {
        auto gpuBackend = std::make_unique<GpuPhysicsBackend>();
        if (gpuBackend->initialize(gpuDevice)) {
            m_gpuBackend = std::move(gpuBackend);
        } else {
            m_backend = PhysicsBackend::Cpu;  // initialization failed, fall back
        }
    }
#else
    (void)gpuDevice;
    if (backend != PhysicsBackend::Cpu)
        m_backend = PhysicsBackend::Cpu;  // GPU not compiled in
#endif
}

Body PhysicsWorld::createBody(const BodyDescriptor& desc) {
    Body body = m_pool.createBody(desc);
    uint32_t id = body.id();

    AABB aabb;
    const auto& d = m_pool.get(id);
    if (d.shape) {
        aabb = d.shape->computeAABB(d.transform);
    } else {
        aabb = AABB{ d.transform.position, d.transform.position };
    }

    bool isStatic = (desc.type == BodyType::Static);
    m_broadPhase.insertBody(id, aabb, desc.layer, desc.mask, isStatic);
    return body;
}

void PhysicsWorld::destroyBody(Body body) {
    m_broadPhase.removeBody(body.id());
    m_pool.destroyBody(body.id());
}

void PhysicsWorld::addConstraint(std::shared_ptr<Constraint> c) {
    m_constraintSolver.add(std::move(c));
}

void PhysicsWorld::removeConstraint(const std::shared_ptr<Constraint>& c) {
    m_constraintSolver.remove(c);
}

BuoyancyVolume PhysicsWorld::addBuoyancyVolume(const BuoyancyDescriptor& desc) {
    return m_buoyancySystem->add(desc);
}

void PhysicsWorld::removeBuoyancyVolume(BuoyancyVolume vol) {
    m_buoyancySystem->remove(vol);
}

ArticulatedBody PhysicsWorld::createArticulatedBody(const ArticulatedBodyDescriptor& desc) {
    return m_articulatedBodySystem->create(
        desc,
        [this](const BodyDescriptor& d) { return createBody(d); },
        [this](std::shared_ptr<Constraint> c) { addConstraint(std::move(c)); });
}

void PhysicsWorld::destroyArticulatedBody(ArticulatedBody ab) {
    m_articulatedBodySystem->destroy(
        ab,
        [this](Body b) { destroyBody(b); },
        [this](const std::shared_ptr<Constraint>& c) { removeConstraint(c); });
}

RagdollBody PhysicsWorld::createRagdoll(const RagdollDescriptor& desc) {
    auto artDesc = RagdollFactory::makeDescriptor(desc);
    auto ab      = createArticulatedBody(artDesc);
    return RagdollFactory::build(desc, std::move(ab));
}

void PhysicsWorld::destroyRagdoll(RagdollBody& rb) {
    destroyArticulatedBody(rb.articulation());
}

VehicleBody PhysicsWorld::createVehicle(const VehicleDescriptor& desc) {
    Body chassis = createBody(desc.chassis);
    return m_vehicleSystem->create(desc, chassis);
}

void PhysicsWorld::destroyVehicle(VehicleBody vb) {
    if (vb.chassisBody().isValid())
        destroyBody(vb.chassisBody());
    m_vehicleSystem->destroy(vb);
}

void PhysicsWorld::syncVehicleControls(const VehicleBody& vb) {
    m_vehicleSystem->syncControls(vb);
}

const WheelState& PhysicsWorld::vehicleWheelState(const VehicleBody& vb, int i) const noexcept {
    return m_vehicleSystem->wheelState(vb, i);
}

int PhysicsWorld::vehicleWheelCount(const VehicleBody& vb) const noexcept {
    return m_vehicleSystem->wheelCount(vb);
}

void PhysicsWorld::setGravity(const vm::Vector3<float>& g) noexcept {
    m_integratorSettings.gravity = g;
}

void PhysicsWorld::setFixedTimestep(float dt) noexcept {
    m_fixedTimestep = dt > 0.f ? dt : m_fixedTimestep;
}

void PhysicsWorld::setSubsteps(int n) noexcept {
    m_substeps = n > 0 ? n : 1;
}

vm::Vector3<float> PhysicsWorld::gravity() const noexcept {
    return m_integratorSettings.gravity;
}

float PhysicsWorld::fixedTimestep() const noexcept {
    return m_fixedTimestep;
}

void PhysicsWorld::addStepListener(IStepListener* l)       { m_stepListeners.push_back(l); }
void PhysicsWorld::addContactListener(IContactListener* l) { m_contactListeners.push_back(l); }
void PhysicsWorld::addTriggerListener(ITriggerListener* l) { m_triggerListeners.push_back(l); }

void PhysicsWorld::removeStepListener(IStepListener* l) {
    auto it = std::find(m_stepListeners.begin(), m_stepListeners.end(), l);
    if (it != m_stepListeners.end()) m_stepListeners.erase(it);
}
void PhysicsWorld::removeContactListener(IContactListener* l) {
    auto it = std::find(m_contactListeners.begin(), m_contactListeners.end(), l);
    if (it != m_contactListeners.end()) m_contactListeners.erase(it);
}
void PhysicsWorld::removeTriggerListener(ITriggerListener* l) {
    auto it = std::find(m_triggerListeners.begin(), m_triggerListeners.end(), l);
    if (it != m_triggerListeners.end()) m_triggerListeners.erase(it);
}

void PhysicsWorld::step(float deltaTime) {
    std::unique_lock lock(m_bodyMutex);
    const float subDt = m_fixedTimestep / static_cast<float>(m_substeps);
    m_accumulator += deltaTime;
    while (m_accumulator >= m_fixedTimestep) {
        for (int s = 0; s < m_substeps; ++s)
            substep(subDt);
        m_accumulator -= m_fixedTimestep;
    }
}

// ── Substep pipeline ──────────────────────────────────────────────────────────

void PhysicsWorld::substep(float dt) {
    for (auto* l : m_stepListeners) l->onPreStep(dt);

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "Buoyancy");
        m_buoyancySystem->applyForces(m_pool, m_integratorSettings, dt);
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "Vehicles");
        m_vehicleSystem->update(m_pool, m_integratorSettings, dt,
            [this](const Ray& r, const QueryFilter& f) { return raycastClosest(r, f); });
    }

    // GPU path — replaces broadphase+narrowphase+solver+integration for this substep.
    // Contact/trigger events are not dispatched in GPU mode.
#ifdef CAMPELLO_PHYSICS_GPU_ENABLED
    if (m_gpuBackend) {
        bool useGpu = (m_backend == PhysicsBackend::Gpu) ||
                      (m_backend == PhysicsBackend::Auto &&
                       m_pool.activeCount() >= static_cast<uint32_t>(m_gpuBodyThreshold));
        if (useGpu) {
            CAMPELLO_PROFILE_SCOPE(m_profiler, "GpuStep");
            m_gpuBackend->step(m_pool, m_integratorSettings, dt);
            for (auto* l : m_stepListeners) l->onPostStep(dt);
            return;
        }
    }
#endif

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "BroadPhase");
        updateBroadPhase();
    }

    auto addedPairs   = m_broadPhase.addedPairs();
    auto removedPairs = m_broadPhase.removedPairs();

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "NarrowPhase");
        m_narrowPhase.clearManifolds();
        m_narrowPhase.process(
            m_broadPhase.currentPairs(),
            [this](uint32_t id) { return m_pool.getShapeInstance(id); },
            m_workerThreads);
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "BuildContacts");
        buildContacts(dt);
        buildCcdContacts(dt);
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "Islands");
        buildIslands();        // union-find grouping + wake propagation
        warmStartContacts();   // warm start after bodies are woken
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "Solver");
        m_constraintSolver.iterations = constraintIterations;
        solveIslands(dt);      // per-island constraint + contact solve
    }

    // Persist accumulated impulses for next frame's warm start
    for (const auto& p : m_contactPoints) {
        if (p.wsiN)  *p.wsiN  = p.lambdaN;
        if (p.wsiT0) *p.wsiT0 = p.lambdaT0;
        if (p.wsiT1) *p.wsiT1 = p.lambdaT1;
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "Integrate");
        integrate(m_pool, m_integratorSettings, dt);
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "PositionSolve");
        m_constraintSolver.positionIterations = positionIterations;
        m_constraintSolver.positionAlpha      = positionCorrectionAlpha;
        m_constraintSolver.solvePositions(m_pool, dt);
        solveContactPositions(dt);
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "IslandSleep");
        updateIslandSleep();   // island-level simultaneous sleep decision
    }

    for (auto* l : m_stepListeners) l->onPostStep(dt);

    dispatchContactEvents();
    dispatchTriggerEvents(addedPairs, removedPairs);
}

void PhysicsWorld::updateBroadPhase() {
    // Collect IDs of moving bodies
    struct Entry { uint32_t id; AABB aabb; };
    std::vector<Entry> entries;
    entries.reserve(m_pool.activeCount());
    m_pool.forEach([&](uint32_t id, const BodyData& d) {
        if (d.type == BodyType::Static) return;
        entries.push_back({id, {}});
    });

    // Compute AABBs — parallel when multiple worker threads are configured.
    // The BVH update itself (updateBody) is not thread-safe and remains serial.
    const int n = static_cast<int>(entries.size());
    if (m_workerThreads > 1 && n >= 32) {
        auto computeRange = [&](int first, int last) {
            for (int i = first; i < last; ++i) {
                const auto& d = m_pool.get(entries[static_cast<size_t>(i)].id);
                entries[static_cast<size_t>(i)].aabb = d.shape
                    ? d.shape->computeAABB(d.transform)
                    : AABB{ d.transform.position, d.transform.position };
            }
        };
        const int threads = std::min(m_workerThreads, n);
        const int chunk   = (n + threads - 1) / threads;
        std::vector<std::thread> workers;
        workers.reserve(static_cast<size_t>(threads - 1));
        for (int t = 0; t < threads - 1; ++t)
            workers.emplace_back(computeRange, t * chunk, std::min((t + 1) * chunk, n));
        computeRange((threads - 1) * chunk, n);
        for (auto& th : workers) th.join();
    } else {
        for (auto& e : entries) {
            const auto& d = m_pool.get(e.id);
            e.aabb = d.shape
                ? d.shape->computeAABB(d.transform)
                : AABB{ d.transform.position, d.transform.position };
        }
    }

    // Apply BVH updates serially (BVH is not thread-safe)
    for (const auto& e : entries)
        m_broadPhase.updateBody(e.id, e.aabb);

    m_broadPhase.computePairs();
}

void PhysicsWorld::buildContacts(float dt) {
    m_contactPoints.clear();

    static constexpr float kRestitutionThreshold = 1.f;  // m/s

    for (auto& manifold : m_narrowPhase.manifolds()) {
        const auto& da = m_pool.get(manifold.bodyA);
        const auto& db = m_pool.get(manifold.bodyB);

        // Sensor bodies generate events but no impulse response
        if (da.type == BodyType::Sensor || db.type == BodyType::Sensor) continue;

        // Combined material properties (average)
        float restitution = (da.restitution + db.restitution) * 0.5f;
        float friction    = (da.friction    + db.friction)    * 0.5f;

        for (int i = 0; i < manifold.count; ++i) {
            auto& cp = manifold.points[i];
            const auto& n  = cp.normal;

            ContactSolverPoint p;
            p.bodyA = manifold.bodyA;
            p.bodyB = manifold.bodyB;
            p.friction = friction;

            // Arms from body centers to contact point
            p.rA = cp.position - da.transform.position;
            p.rB = cp.position - db.transform.position;

            // Normal Jacobian (n from B toward A)
            p.J_va_n = n;
            p.J_wa_n = cross3(p.rA, n);
            p.J_vb_n = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
            p.J_wb_n = vm::Vector3<float>(-cross3(p.rB, n).x(),
                                          -cross3(p.rB, n).y(),
                                          -cross3(p.rB, n).z());

            // Friction Jacobians (two tangent axes)
            vm::Vector3<float> t0, t1;
            perp3(n, t0, t1);

            p.J_va_t0 = t0;
            p.J_wa_t0 = cross3(p.rA, t0);
            p.J_vb_t0 = vm::Vector3<float>(-t0.x(), -t0.y(), -t0.z());
            p.J_wb_t0 = vm::Vector3<float>(-cross3(p.rB, t0).x(),
                                           -cross3(p.rB, t0).y(),
                                           -cross3(p.rB, t0).z());

            p.J_va_t1 = t1;
            p.J_wa_t1 = cross3(p.rA, t1);
            p.J_vb_t1 = vm::Vector3<float>(-t1.x(), -t1.y(), -t1.z());
            p.J_wb_t1 = vm::Vector3<float>(-cross3(p.rB, t1).x(),
                                           -cross3(p.rB, t1).y(),
                                           -cross3(p.rB, t1).z());

            // Effective masses
            p.effMassN  = computeEffMassRow(da, db, p.J_va_n,  p.J_wa_n,  p.J_vb_n,  p.J_wb_n);
            p.effMassT0 = computeEffMassRow(da, db, p.J_va_t0, p.J_wa_t0, p.J_vb_t0, p.J_wb_t0);
            p.effMassT1 = computeEffMassRow(da, db, p.J_va_t1, p.J_wa_t1, p.J_vb_t1, p.J_wb_t1);

            // Baumgarte bias for normal (penetration correction)
            float posErr = cp.depth - contactSlop;
            float posBias = posErr > 0.f ? -(contactBaumgarte / dt) * posErr : 0.f;

            // Restitution bias: use closing velocity before impulses
            float Jv_n = dot3(p.J_va_n, da.linearVelocity)
                       + dot3(p.J_wa_n, da.angularVelocity)
                       + dot3(p.J_vb_n, db.linearVelocity)
                       + dot3(p.J_wb_n, db.angularVelocity);
            float restBias = 0.f;
            if (Jv_n < -kRestitutionThreshold)
                restBias = restitution * Jv_n;  // Jv_n negative → restBias negative → larger push-out impulse

            p.biasN = posBias + restBias;

            // Warm start from previous frame
            p.lambdaN  = cp.warmStartImpulse;
            p.lambdaT0 = cp.warmStartFriction0;
            p.lambdaT1 = cp.warmStartFriction1;

            // Store write-back pointers so solveContacts can persist lambdas
            p.wsiN  = &cp.warmStartImpulse;
            p.wsiT0 = &cp.warmStartFriction0;
            p.wsiT1 = &cp.warmStartFriction1;

            m_contactPoints.push_back(p);
        }
    }
    // Warm start is applied by warmStartContacts() after island wake propagation.
}

void PhysicsWorld::solveContactPositions(float /*dt*/) {
    if (positionIterations <= 0) return;
    const float alpha = positionCorrectionAlpha;

    for (const auto& manifold : m_narrowPhase.manifolds()) {
        auto& da = m_pool.get(manifold.bodyA);
        auto& db = m_pool.get(manifold.bodyB);
        if (da.type == BodyType::Sensor || db.type == BodyType::Sensor) continue;
        if (da.isSleeping && db.isSleeping) continue;

        for (int i = 0; i < manifold.count; ++i) {
            const auto& cp = manifold.points[i];
            // Only correct penetrations that Baumgarte alone is too slow to fix.
            // Using 4× slop ensures we don't fight the velocity-level correction.
            const float depth = cp.depth - 4.f * contactSlop;
            if (depth <= 0.f) continue;

            const auto& n  = cp.normal;
            const auto  rA = cp.position - da.transform.position;
            const auto  rB = cp.position - db.transform.position;

            const auto  Jva = n;
            const auto  Jwa = cross3(rA, n);
            const auto  Jvb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
            const auto  Jwb = vm::Vector3<float>(-cross3(rB, n).x(),
                                                 -cross3(rB, n).y(),
                                                 -cross3(rB, n).z());

            const float effMass = computeEffMassRow(da, db, Jva, Jwa, Jvb, Jwb);
            if (effMass == 0.f) continue;

            // Positive lambda pushes A in +n direction (out of floor).
            // Contact convention: depth > 0 → penetrating → push apart (+lambda).
            const float lambda = effMass * alpha * depth;
            applyPosImpulse(da, Jva, Jwa, lambda);
            applyPosImpulse(db, Jvb, Jwb, lambda);
        }
    }
}

void PhysicsWorld::warmStartContacts() {
    for (auto& p : m_contactPoints) {
        auto& da = m_pool.get(p.bodyA);
        auto& db = m_pool.get(p.bodyB);
        if (da.isSleeping && db.isSleeping) continue;
        const float ws = 0.85f;
        applyImpulseRow(da, db, p.J_va_n,  p.J_wa_n,  p.J_vb_n,  p.J_wb_n,  p.lambdaN  * ws);
        applyImpulseRow(da, db, p.J_va_t0, p.J_wa_t0, p.J_vb_t0, p.J_wb_t0, p.lambdaT0 * ws);
        applyImpulseRow(da, db, p.J_va_t1, p.J_wa_t1, p.J_vb_t1, p.J_wb_t1, p.lambdaT1 * ws);
    }
}

void PhysicsWorld::solveContactPoint(ContactSolverPoint& p) {
    auto& da = m_pool.get(p.bodyA);
    auto& db = m_pool.get(p.bodyB);

    // Normal row (lambda >= 0)
    {
        float Jv = dot3(p.J_va_n, da.linearVelocity)
                 + dot3(p.J_wa_n, da.angularVelocity)
                 + dot3(p.J_vb_n, db.linearVelocity)
                 + dot3(p.J_wb_n, db.angularVelocity);
        float dL  = p.effMassN * -(Jv + p.biasN);
        float lOld = p.lambdaN;
        p.lambdaN  = std::max(0.f, p.lambdaN + dL);
        dL = p.lambdaN - lOld;
        applyImpulseRow(da, db, p.J_va_n, p.J_wa_n, p.J_vb_n, p.J_wb_n, dL);
    }

    // Tangent rows (Coulomb friction)
    float limit = p.friction * p.lambdaN;
    {
        float Jv = dot3(p.J_va_t0, da.linearVelocity)
                 + dot3(p.J_wa_t0, da.angularVelocity)
                 + dot3(p.J_vb_t0, db.linearVelocity)
                 + dot3(p.J_wb_t0, db.angularVelocity);
        float dL  = p.effMassT0 * -Jv;
        float lOld = p.lambdaT0;
        p.lambdaT0 = std::clamp(p.lambdaT0 + dL, -limit, limit);
        dL = p.lambdaT0 - lOld;
        applyImpulseRow(da, db, p.J_va_t0, p.J_wa_t0, p.J_vb_t0, p.J_wb_t0, dL);
    }
    {
        float Jv = dot3(p.J_va_t1, da.linearVelocity)
                 + dot3(p.J_wa_t1, da.angularVelocity)
                 + dot3(p.J_vb_t1, db.linearVelocity)
                 + dot3(p.J_wb_t1, db.angularVelocity);
        float dL  = p.effMassT1 * -Jv;
        float lOld = p.lambdaT1;
        p.lambdaT1 = std::clamp(p.lambdaT1 + dL, -limit, limit);
        dL = p.lambdaT1 - lOld;
        applyImpulseRow(da, db, p.J_va_t1, p.J_wa_t1, p.J_vb_t1, p.J_wb_t1, dL);
    }
}

// ── Island detection ──────────────────────────────────────────────────────────

uint32_t PhysicsWorld::ufFind(uint32_t x) noexcept {
    // Path-halving for amortised O(α(n))
    while (m_ufParent[x] != x) {
        m_ufParent[x] = m_ufParent[m_ufParent[x]];
        x = m_ufParent[x];
    }
    return x;
}

void PhysicsWorld::ufUnion(uint32_t x, uint32_t y) noexcept {
    x = ufFind(x); y = ufFind(y);
    if (x == y) return;
    if (m_ufRank[x] < m_ufRank[y]) std::swap(x, y);
    m_ufParent[y] = x;
    if (m_ufRank[x] == m_ufRank[y]) ++m_ufRank[x];
}

void PhysicsWorld::buildIslands() {
    const uint32_t cap = m_pool.capacity();

    // Grow union-find arrays as pool grows (never shrink)
    if (static_cast<uint32_t>(m_ufParent.size()) < cap) {
        m_ufParent.resize(cap, kInvalidUF);
        m_ufRank.resize(cap, 0);
    }

    // Initialise one set per active dynamic/kinematic body
    m_pool.forEach([&](uint32_t id, const BodyData& d) {
        if (d.type == BodyType::Dynamic || d.type == BodyType::Kinematic) {
            m_ufParent[id] = id;
            m_ufRank[id]   = 0;
        }
    });

    // Union bodies connected by contact pairs
    for (const auto& cp : m_contactPoints) {
        const auto& da = m_pool.get(cp.bodyA);
        const auto& db = m_pool.get(cp.bodyB);
        bool aDyn = da.type == BodyType::Dynamic || da.type == BodyType::Kinematic;
        bool bDyn = db.type == BodyType::Dynamic || db.type == BodyType::Kinematic;
        if (aDyn && bDyn) ufUnion(cp.bodyA, cp.bodyB);
    }

    // Union bodies connected by constraints (dynamic↔dynamic only)
    for (const auto& c : m_constraintSolver.constraints()) {
        if (!c) continue;
        uint32_t idA = c->bodyA().id(), idB = c->bodyB().id();
        if (!m_pool.isValid(idA) || !m_pool.isValid(idB)) continue;
        const auto& da = m_pool.get(idA);
        const auto& db = m_pool.get(idB);
        bool aDyn = da.type == BodyType::Dynamic || da.type == BodyType::Kinematic;
        bool bDyn = db.type == BodyType::Dynamic || db.type == BodyType::Kinematic;
        if (aDyn && bDyn) ufUnion(idA, idB);
    }

    // Collect islands: map root → island index
    m_islands.clear();
    std::unordered_map<uint32_t, int> rootToIdx;
    rootToIdx.reserve(m_pool.activeCount());

    m_pool.forEach([&](uint32_t id, const BodyData& d) {
        if (d.type != BodyType::Dynamic && d.type != BodyType::Kinematic) return;
        uint32_t root = ufFind(id);
        auto [it, inserted] = rootToIdx.emplace(root, static_cast<int>(m_islands.size()));
        if (inserted) m_islands.push_back({});
        m_islands[it->second].bodyIds.push_back(id);
    });

    // Assign contact indices to their island
    for (int i = 0; i < static_cast<int>(m_contactPoints.size()); ++i) {
        const auto& cp = m_contactPoints[i];
        const auto& da = m_pool.get(cp.bodyA);
        bool aDyn = da.type == BodyType::Dynamic || da.type == BodyType::Kinematic;
        uint32_t rep = aDyn ? cp.bodyA : cp.bodyB;
        if (m_ufParent[rep] == kInvalidUF) continue;
        auto it = rootToIdx.find(ufFind(rep));
        if (it != rootToIdx.end())
            m_islands[it->second].contactIndices.push_back(i);
    }

    // Assign constraints to their island
    for (const auto& c : m_constraintSolver.constraints()) {
        if (!c) continue;
        uint32_t idA = c->bodyA().id(), idB = c->bodyB().id();
        if (!m_pool.isValid(idA) || !m_pool.isValid(idB)) continue;
        const auto& da = m_pool.get(idA);
        const auto& db = m_pool.get(idB);
        bool aDyn = da.type == BodyType::Dynamic || da.type == BodyType::Kinematic;
        bool bDyn = db.type == BodyType::Dynamic || db.type == BodyType::Kinematic;
        uint32_t rep = aDyn ? idA : (bDyn ? idB : kInvalidUF);
        if (rep == kInvalidUF || m_ufParent[rep] == kInvalidUF) continue;
        auto it = rootToIdx.find(ufFind(rep));
        if (it != rootToIdx.end())
            m_islands[it->second].constraintPtrs.push_back(c.get());
    }

    // Wake propagation: one awake body wakes the whole island
    for (auto& island : m_islands) {
        bool hasAwake = false;
        for (uint32_t id : island.bodyIds)
            if (!m_pool.get(id).isSleeping) { hasAwake = true; break; }
        if (!hasAwake) continue;
        for (uint32_t id : island.bodyIds) {
            auto& d = m_pool.get(id);
            if (d.isSleeping) {
                d.isSleeping  = false;
                d.sleepFrames = 0;
            }
        }
    }
}

void PhysicsWorld::solveIslands(float dt) {
    for (const auto& island : m_islands) {
        // Skip fully sleeping islands
        bool allSleeping = true;
        for (uint32_t id : island.bodyIds)
            if (!m_pool.get(id).isSleeping) { allSleeping = false; break; }
        if (allSleeping) continue;

        // Constraints: prepare → warm start → N×solve
        for (auto* c : island.constraintPtrs)
            c->prepare(m_pool, dt, m_constraintSolver.baumgarte, m_constraintSolver.slop);
        for (auto* c : island.constraintPtrs)
            c->warmStart(m_pool);
        for (int iter = 0; iter < constraintIterations; ++iter)
            for (auto* c : island.constraintPtrs)
                c->solveVelocity(m_pool);

        // Contacts: N×solve
        for (int iter = 0; iter < contactIterations; ++iter)
            for (int ci : island.contactIndices)
                solveContactPoint(m_contactPoints[ci]);
    }
}

void PhysicsWorld::updateIslandSleep() {
    const int required = m_integratorSettings.sleepFramesRequired;
    for (const auto& island : m_islands) {
        // Island can sleep only when ALL its dynamic bodies are ready
        bool allReady = true;
        for (uint32_t id : island.bodyIds) {
            const auto& d = m_pool.get(id);
            if (d.type != BodyType::Dynamic) continue; // kinematic bodies never sleep
            if (d.isSleeping) continue;                // already asleep — don't block
            if (d.sleepFrames < required) { allReady = false; break; }
        }
        if (!allReady) continue;

        // Put all dynamic bodies to sleep simultaneously
        for (uint32_t id : island.bodyIds) {
            auto& d = m_pool.get(id);
            if (d.type != BodyType::Dynamic) continue;
            if (d.isSleeping) continue;
            d.isSleeping      = true;
            d.linearVelocity  = { 0.f, 0.f, 0.f };
            d.angularVelocity = { 0.f, 0.f, 0.f };
        }
    }
}

void PhysicsWorld::dispatchContactEvents() {
    m_currContactKeys.clear();

    for (const auto& manifold : m_narrowPhase.manifolds()) {
        const auto& da = m_pool.get(manifold.bodyA);
        const auto& db = m_pool.get(manifold.bodyB);
        if (da.type == BodyType::Sensor || db.type == BodyType::Sensor) continue;

        uint64_t key = (uint64_t(manifold.bodyA) << 32) | manifold.bodyB;
        m_currContactKeys.push_back(key);

        if (!m_contactListeners.empty()) {
            Body ha = m_pool.makeHandle(manifold.bodyA);
            Body hb = m_pool.makeHandle(manifold.bodyB);
            bool wasPresent = std::find(m_prevContactKeys.begin(),
                                        m_prevContactKeys.end(), key)
                              != m_prevContactKeys.end();
            for (auto* l : m_contactListeners) {
                if (wasPresent) l->onContactPersisted(ha, hb, manifold);
                else            l->onContactAdded(ha, hb, manifold);
            }
        }
    }

    if (!m_contactListeners.empty()) {
        for (uint64_t key : m_prevContactKeys) {
            bool stillPresent = std::find(m_currContactKeys.begin(),
                                          m_currContactKeys.end(), key)
                                != m_currContactKeys.end();
            if (!stillPresent) {
                uint32_t idA = static_cast<uint32_t>(key >> 32);
                uint32_t idB = static_cast<uint32_t>(key);
                if (m_pool.isValid(idA) && m_pool.isValid(idB)) {
                    Body ha = m_pool.makeHandle(idA);
                    Body hb = m_pool.makeHandle(idB);
                    for (auto* l : m_contactListeners)
                        l->onContactRemoved(ha, hb);
                }
            }
        }
    }

    std::swap(m_prevContactKeys, m_currContactKeys);
}

void PhysicsWorld::buildCcdContacts(float dt) {
    m_pool.forEach([&](uint32_t idA, const BodyData& da) {
        if (!da.ccdEnabled) return;
        if (da.type != BodyType::Dynamic) return;
        if (da.isSleeping) return;
        if (!da.shape) return;

        float speed = std::sqrt(lenSq3(da.linearVelocity));
        if (speed < 1e-6f) return;

        // Swept AABB: union of current and end-of-step position AABBs
        AABB aabbCur = da.shape->computeAABB(da.transform);
        Transform futureT(da.transform.position + da.linearVelocity * dt,
                          da.transform.rotation);
        AABB aabbFut = da.shape->computeAABB(futureT);
        AABB swept   = aabbCur.merged(aabbFut);

        m_broadPhase.queryAABB(swept, da.layer, da.mask, [&](uint32_t idB) {
            if (idB == idA) return;
            const auto& db = m_pool.get(idB);
            if (!db.active || db.type == BodyType::Sensor || !db.shape) return;

            // One CCD contact per dynamic-dynamic pair (lower id is canonical)
            if (db.ccdEnabled && idB < idA) return;

            ShapeInstance siA = m_pool.getShapeInstance(idA);
            ShapeInstance siB = m_pool.getShapeInstance(idB);

            // Skip pairs already in discrete overlap (narrow phase handles these)
            if (collide(siA, siB)) return;

            // Conservative step size: obstacle can't be skipped if step <= its min half-extent
            AABB aabbB = db.shape->computeAABB(db.transform);
            float bHalfX = (aabbB.max.x() - aabbB.min.x()) * 0.5f;
            float bHalfY = (aabbB.max.y() - aabbB.min.y()) * 0.5f;
            float bHalfZ = (aabbB.max.z() - aabbB.min.z()) * 0.5f;
            float minHalf = std::min({bHalfX, bHalfY, bHalfZ});
            minHalf = std::max(minHalf, 0.001f);
            float stepTime = std::min(minHalf / speed, dt);

            // March from stepTime to dt to find the first overlapping time
            float prevT = 0.f, firstOverlapT = -1.f;
            for (float t = stepTime; firstOverlapT < 0.f; t += stepTime) {
                float tc = (t < dt) ? t : dt;
                ShapeInstance siA_t = siA;
                siA_t.transform.position = siA_t.transform.position + da.linearVelocity * tc;
                if (collide(siA_t, siB)) {
                    firstOverlapT = tc;
                } else {
                    prevT = tc;
                    if (tc >= dt) break;
                }
            }
            if (firstOverlapT < 0.f) return;  // no collision in this step

            // Binary search in [prevT, firstOverlapT] for the exact TOI
            float lo = prevT, hi = firstOverlapT;
            for (int iter = 0; iter < 20; ++iter) {
                float mid = (lo + hi) * 0.5f;
                ShapeInstance siA_mid = siA;
                siA_mid.transform.position = siA_mid.transform.position + da.linearVelocity * mid;
                if (collide(siA_mid, siB)) hi = mid;
                else                       lo = mid;
            }

            // Get contact normal from the first overlapping position (hi)
            ShapeInstance siA_hi = siA;
            siA_hi.transform.position = siA_hi.transform.position + da.linearVelocity * hi;
            auto manifold = collide(siA_hi, siB);
            if (!manifold || manifold->count == 0) return;

            const auto& n = manifold->points[0].normal;

            // Approximate contact arms at current (t=0) positions
            vm::Vector3<float> contactPos =
                manifold->points[0].position - da.linearVelocity * hi;
            vm::Vector3<float> rA = contactPos - da.transform.position;
            vm::Vector3<float> rB = contactPos - db.transform.position;

            // Normal Jacobian (n points from B toward A)
            auto negN = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
            auto negCrB = [&](const vm::Vector3<float>& r, const vm::Vector3<float>& v) {
                auto c = cross3(r, v);
                return vm::Vector3<float>(-c.x(), -c.y(), -c.z());
            };

            ContactSolverPoint p;
            p.bodyA    = idA;
            p.bodyB    = idB;
            p.friction = (da.friction + db.friction) * 0.5f;
            p.rA = rA;
            p.rB = rB;

            p.J_va_n = n;
            p.J_wa_n = cross3(rA, n);
            p.J_vb_n = negN;
            p.J_wb_n = negCrB(rB, n);

            // Speculative bias: allow closing at a rate proportional to remaining gap
            // biasN = Jv_n * (lo/dt) → solver reduces closing velocity just enough to
            // arrive at contact in exactly lo seconds, preventing tunnelling.
            float Jv_n = dot3(p.J_va_n, da.linearVelocity)
                       + dot3(p.J_wa_n, da.angularVelocity)
                       + dot3(p.J_vb_n, db.linearVelocity)
                       + dot3(p.J_wb_n, db.angularVelocity);
            if (Jv_n >= 0.f) return;  // separating, no constraint needed

            p.biasN = Jv_n * (lo / dt);

            // Friction Jacobians
            vm::Vector3<float> t0, t1;
            perp3(n, t0, t1);

            p.J_va_t0 = t0;           p.J_wa_t0 = cross3(rA, t0);
            p.J_vb_t0 = vm::Vector3<float>(-t0.x(), -t0.y(), -t0.z());
            p.J_wb_t0 = negCrB(rB, t0);
            p.J_va_t1 = t1;           p.J_wa_t1 = cross3(rA, t1);
            p.J_vb_t1 = vm::Vector3<float>(-t1.x(), -t1.y(), -t1.z());
            p.J_wb_t1 = negCrB(rB, t1);

            p.effMassN  = computeEffMassRow(da, db, p.J_va_n,  p.J_wa_n,  p.J_vb_n,  p.J_wb_n);
            p.effMassT0 = computeEffMassRow(da, db, p.J_va_t0, p.J_wa_t0, p.J_vb_t0, p.J_wb_t0);
            p.effMassT1 = computeEffMassRow(da, db, p.J_va_t1, p.J_wa_t1, p.J_vb_t1, p.J_wb_t1);

            p.lambdaN = p.lambdaT0 = p.lambdaT1 = 0.f;
            p.wsiN = p.wsiT0 = p.wsiT1 = nullptr;  // no warm start for speculative contacts

            m_contactPoints.push_back(p);
        });
    });
}

void PhysicsWorld::dispatchTriggerEvents(
    const std::vector<CollisionPair>& added,
    const std::vector<CollisionPair>& removed)
{
    if (m_triggerListeners.empty()) return;

    for (const auto& pair : added) {
        if (!m_pool.isValid(pair.bodyA) || !m_pool.isValid(pair.bodyB)) continue;
        const auto& da = m_pool.get(pair.bodyA);
        const auto& db = m_pool.get(pair.bodyB);
        bool aIsSensor = da.type == BodyType::Sensor;
        bool bIsSensor = db.type == BodyType::Sensor;
        if (!aIsSensor && !bIsSensor) continue;

        Body sensor = m_pool.makeHandle(aIsSensor ? pair.bodyA : pair.bodyB);
        Body other  = m_pool.makeHandle(aIsSensor ? pair.bodyB : pair.bodyA);
        for (auto* l : m_triggerListeners)
            l->onTriggerEnter(sensor, other);
    }

    for (const auto& pair : removed) {
        if (!m_pool.isValid(pair.bodyA) || !m_pool.isValid(pair.bodyB)) continue;
        const auto& da = m_pool.get(pair.bodyA);
        const auto& db = m_pool.get(pair.bodyB);
        bool aIsSensor = da.type == BodyType::Sensor;
        bool bIsSensor = db.type == BodyType::Sensor;
        if (!aIsSensor && !bIsSensor) continue;

        Body sensor = m_pool.makeHandle(aIsSensor ? pair.bodyA : pair.bodyB);
        Body other  = m_pool.makeHandle(aIsSensor ? pair.bodyB : pair.bodyA);
        for (auto* l : m_triggerListeners)
            l->onTriggerExit(sensor, other);
    }
}

} // namespace campello::physics
