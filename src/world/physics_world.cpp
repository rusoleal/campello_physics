#include <campello_physics/physics_world.h>
#include "gpu/gpu_backend.h"
#ifdef CAMPELLO_PHYSICS_GPU_ENABLED
#include "gpu/gpu_physics_backend.h"
#endif
#include "buoyancy_system.h"
#include "foundation/thread_pool.h"
#include "articulated_body_system.h"
#include <campello_physics/ragdoll.h>
#include "vehicle_system.h"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>
#include "constraints/constraint_utils.h"

#if defined(__SSE__) || defined(__AVX__)
    #include <immintrin.h>
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
    #include <arm_neon.h>
#endif

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
            + dot3(Jwa, detail::applyInvInertiaWorld(da, Jwa))
            + dot3(Jwb, detail::applyInvInertiaWorld(db, Jwb));
    return K > 1e-12f ? 1.f / K : 0.f;
}

inline void applyImpulseRow(BodyData& da, BodyData& db,
                             const vm::Vector3<float>& Jva,
                             const vm::Vector3<float>& Jwa,
                             const vm::Vector3<float>& Jvb,
                             const vm::Vector3<float>& Jwb,
                             float dL) {
    da.linearVelocity  = da.linearVelocity  + Jva * (dL * da.invMass);
    da.angularVelocity = da.angularVelocity + detail::applyInvInertiaWorld(da, Jwa * dL);
    db.linearVelocity  = db.linearVelocity  + Jvb * (dL * db.invMass);
    db.angularVelocity = db.angularVelocity + detail::applyInvInertiaWorld(db, Jwb * dL);
}

inline void applyPosImpulse(BodyData& d,
                              const vm::Vector3<float>& Jv,
                              const vm::Vector3<float>& /*Jw*/,
                              float lam) {
    if (d.invMass == 0.f || d.isSleeping) return;
    d.transform.position = d.transform.position + Jv * (lam * d.invMass);
}

// Fast-path AABB for the two most common primitive shapes.
// Falls back to virtual dispatch for everything else.
inline AABB computeBodyAABB(const Shape* shape, const Transform& transform) {
    if (!shape)
        return AABB{ transform.position, transform.position };

    switch (shape->type()) {
        case ShapeType::Sphere: {
            const auto* s = static_cast<const SphereShape*>(shape);
            const float r = s->radius();
            return AABB::fromCenterHalfExtents(transform.position,
                                               vm::Vector3<float>(r, r, r));
        }
        case ShapeType::Box: {
            const auto* b = static_cast<const BoxShape*>(shape);
            const float hx = b->halfExtents().x();
            const float hy = b->halfExtents().y();
            const float hz = b->halfExtents().z();
            auto ax = transform.rotation.rotated(vm::Vector3<float>(hx, 0.f, 0.f));
            auto ay = transform.rotation.rotated(vm::Vector3<float>(0.f, hy, 0.f));
            auto az = transform.rotation.rotated(vm::Vector3<float>(0.f, 0.f, hz));
            vm::Vector3<float> he(
                std::fabs(ax.x()) + std::fabs(ay.x()) + std::fabs(az.x()),
                std::fabs(ax.y()) + std::fabs(ay.y()) + std::fabs(az.y()),
                std::fabs(ax.z()) + std::fabs(ay.z()) + std::fabs(az.z()));
            return AABB::fromCenterHalfExtents(transform.position, he);
        }
        default:
            return shape->computeAABB(transform);
    }
}

inline AABB computeBodyAABB(const BodyData& d) {
    return computeBodyAABB(d.shape.get(), d.transform);
}

} // namespace

// ── PhysicsWorld ──────────────────────────────────────────────────────────────

PhysicsWorld::PhysicsWorld()
    : m_buoyancySystem(std::make_unique<BuoyancySystem>())
    , m_articulatedBodySystem(std::make_unique<ArticulatedBodySystem>())
    , m_vehicleSystem(std::make_unique<VehicleSystem>())
    , m_threadPool(std::make_unique<ThreadPool>(1)) {
    m_constraintSolver.iterations = constraintIterations;
}

void PhysicsWorld::setWorkerThreads(int n) noexcept {
    int desired = n > 1 ? n : 1;
    if (desired == m_workerThreads) return;
    m_workerThreads = desired;
    m_threadPool = std::make_unique<ThreadPool>(desired);
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

    AABB aabb = computeBodyAABB(m_pool.get(id));

    bool isStatic = (desc.type == BodyType::Static);
    m_broadPhase.insertBody(id, aabb, desc.layer, desc.mask, isStatic);
    return body;
}

void PhysicsWorld::destroyBody(Body body) {
    if (!body.isValid()) return;
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
        if (m_workerThreads > 1) {
            m_narrowPhase.process(
                m_broadPhase.currentPairs(),
                [this](uint32_t id) { return m_pool.getShapeInstance(id); },
                m_workerThreads,
                [this](int total, const std::function<void(int first, int last)>& fn) {
                    m_threadPool->parallelFor(total, fn);
                });
        } else {
            m_narrowPhase.process(
                m_broadPhase.currentPairs(),
                [this](uint32_t id) { return m_pool.getShapeInstance(id); },
                1);
        }
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "BuildContacts");
        buildContacts(dt);
        if (m_pool.ccdEnabledCount() > 0)
            buildCcdContacts(dt);
        buildContactColors();
    }
    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "BuildGlobalContactColors");
        if (!m_contactPoints.empty())
            buildGlobalContactColors();
    }

    const bool hasContacts    = !m_contactPoints.empty();
    const bool hasConstraints = !m_constraintSolver.constraints().empty();

    if (hasContacts || hasConstraints) {
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
            CAMPELLO_PROFILE_SCOPE(m_profiler, "PositionSolve");
            solveIslandPositions(dt);
        }

        {
            CAMPELLO_PROFILE_SCOPE(m_profiler, "IslandSleep");
            updateIslandSleep();   // island-level simultaneous sleep decision
        }
    } else {
        m_contactPoints.clear();
        // Fallback per-body sleep for isolated bodies (no contacts / no constraints)
        const int required = m_integratorSettings.sleepFramesRequired;
        m_pool.forEach([&](uint32_t /*id*/, BodyData& d) {
            if (d.type != BodyType::Dynamic || d.isSleeping) return;
            if (d.sleepFrames >= required) {
                d.isSleeping = true;
                d.linearVelocity  = { 0.f, 0.f, 0.f };
                d.angularVelocity = { 0.f, 0.f, 0.f };
            }
        });
    }

    {
        CAMPELLO_PROFILE_SCOPE(m_profiler, "Integrate");
        const auto& dynIds = m_pool.activeDynamicIds();
        if (m_workerThreads > 1 && dynIds.size() >= 64) {
            m_threadPool->parallelFor(static_cast<int>(dynIds.size()), [&](int first, int last) {
                integrateSlice(m_pool, m_integratorSettings, dt,
                               dynIds.data() + first, last - first);
            });
        } else {
            integrate(m_pool, m_integratorSettings, dt);
        }
    }

    for (auto* l : m_stepListeners) l->onPostStep(dt);

    if (!m_contactListeners.empty() || !m_prevContactKeys.empty())
        dispatchContactEvents();
    if (!m_triggerListeners.empty())
        dispatchTriggerEvents(addedPairs, removedPairs);
}

void PhysicsWorld::updateBroadPhase() {
    // Collect IDs of moving bodies and compute AABBs in one pass for the serial path.
    // Parallel path still needs the entries buffer because AABB compute and BVH update
    // cannot overlap (BVH is not thread-safe).
    struct Entry { uint32_t id; AABB aabb; };
    std::vector<Entry> entries;
    entries.reserve(m_pool.activeCount());
    m_pool.forEach([&](uint32_t id, const BodyData& d) {
        if (d.type == BodyType::Static) return;
        entries.push_back({id, {}});
    });

    const int n = static_cast<int>(entries.size());
    if (m_workerThreads > 1 && n >= 32) {
        auto computeRange = [&](int first, int last) {
            for (int i = first; i < last; ++i) {
                const auto& d = m_pool.get(entries[static_cast<size_t>(i)].id);
                entries[static_cast<size_t>(i)].aabb = computeBodyAABB(d);
            }
        };
        m_threadPool->parallelFor(n, computeRange);
        // Apply BVH updates serially
        for (const auto& e : entries)
            m_broadPhase.updateBody(e.id, e.aabb);
    } else {
        // Serial path: fuse AABB compute + BVH update to avoid second pass
        for (auto& e : entries) {
            const auto& d = m_pool.get(e.id);
            m_broadPhase.updateBody(e.id, computeBodyAABB(d));
        }
    }

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
            p.depth    = cp.depth;

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

    for (auto& p : m_contactPoints) {
        auto& da = m_pool.get(p.bodyA);
        auto& db = m_pool.get(p.bodyB);
        if (da.isSleeping && db.isSleeping) continue;

        const float depth = p.depth - 4.f * contactSlop;
        if (depth <= 0.f) continue;

        const auto& n  = p.J_va_n;
        const auto& rA = p.rA;
        const auto& rB = p.rB;

        const auto  Jva = n;
        const auto  Jwa = cross3(rA, n);
        const auto  Jvb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
        const auto  Jwb = vm::Vector3<float>(-cross3(rB, n).x(),
                                             -cross3(rB, n).y(),
                                             -cross3(rB, n).z());

        const float effMass = computeEffMassRow(da, db, Jva, Jwa, Jvb, Jwb);
        if (effMass == 0.f) continue;

        const float lambda = effMass * alpha * depth;
        applyPosImpulse(da, Jva, Jwa, lambda);
        applyPosImpulse(db, Jvb, Jwb, lambda);
    }
}

void PhysicsWorld::solveIslandPositions(float dt) {
    auto solveOne = [&](int first, int last) {
        for (int i = first; i < last; ++i) {
            const auto& island = m_islands[static_cast<size_t>(i)];
            bool allSleeping = true;
            for (uint32_t id : island.bodyIds)
                if (!m_pool.get(id).isSleeping) { allSleeping = false; break; }
            if (allSleeping) continue;

            // Constraint position solve
            for (int iter = 0; iter < positionIterations; ++iter)
                for (auto* c : island.constraintPtrs)
                    c->solvePosition(m_pool, dt, positionCorrectionAlpha);

            // Contact position solve (island-local)
            const float alpha = positionCorrectionAlpha;
            for (int iter = 0; iter < positionIterations; ++iter) {
                for (int ci : island.contactIndices) {
                    auto& p = m_contactPoints[ci];
                    auto& da = m_pool.get(p.bodyA);
                    auto& db = m_pool.get(p.bodyB);
                    if (da.isSleeping && db.isSleeping) continue;

                    const float depth = p.depth - 4.f * contactSlop;
                    if (depth <= 0.f) continue;

                    const auto& n  = p.J_va_n;
                    const auto& rA = p.rA;
                    const auto& rB = p.rB;

                    const auto  Jva = n;
                    const auto  Jwa = cross3(rA, n);
                    const auto  Jvb = vm::Vector3<float>(-n.x(), -n.y(), -n.z());
                    const auto  Jwb = vm::Vector3<float>(-cross3(rB, n).x(),
                                                         -cross3(rB, n).y(),
                                                         -cross3(rB, n).z());

                    const float effMass = computeEffMassRow(da, db, Jva, Jwa, Jvb, Jwb);
                    if (effMass == 0.f) continue;

                    const float lambda = effMass * alpha * depth;
                    applyPosImpulse(da, Jva, Jwa, lambda);
                    applyPosImpulse(db, Jvb, Jwb, lambda);
                }
            }
        }
    };

    const int islandCount = static_cast<int>(m_islands.size());
    if (m_workerThreads > 1 && islandCount >= 2) {
        m_threadPool->parallelFor(islandCount, solveOne);
    } else {
        solveOne(0, islandCount);
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

// ── SoA SIMD batch contact solver ─────────────────────────────────────────────

namespace {

#if defined(__SSE__) || defined(__AVX__)
inline __m128 mm_dot3_4wide(const float* ax, const float* ay, const float* az,
                            const float* bx, const float* by, const float* bz) {
#if defined(__FMA__)
    __m128 mx = _mm_mul_ps(_mm_loadu_ps(ax), _mm_loadu_ps(bx));
    __m128 my = _mm_fmadd_ps(_mm_loadu_ps(ay), _mm_loadu_ps(by), mx);
    return _mm_fmadd_ps(_mm_loadu_ps(az), _mm_loadu_ps(bz), my);
#else
    __m128 mx = _mm_mul_ps(_mm_loadu_ps(ax), _mm_loadu_ps(bx));
    __m128 my = _mm_mul_ps(_mm_loadu_ps(ay), _mm_loadu_ps(by));
    __m128 mz = _mm_mul_ps(_mm_loadu_ps(az), _mm_loadu_ps(bz));
    return _mm_add_ps(_mm_add_ps(mx, my), mz);
#endif
}

// Solve 4 contacts simultaneously using SSE for dot products & clamping.
// Body velocity updates remain scalar (applyImpulseRow handles invMass/static).
[[maybe_unused]] CAMPELLO_FORCE_INLINE void solveContactBatch4Sse(
        campello::physics::PhysicsWorld::ContactSolverSoA& soa,
        campello::physics::BodyPool& pool, int i) {
    using namespace campello::physics;

    uint32_t idA[4] = { soa.bodyA[i], soa.bodyA[i+1], soa.bodyA[i+2], soa.bodyA[i+3] };
    uint32_t idB[4] = { soa.bodyB[i], soa.bodyB[i+1], soa.bodyB[i+2], soa.bodyB[i+3] };
    BodyData* da[4] = { &pool.get(idA[0]), &pool.get(idA[1]), &pool.get(idA[2]), &pool.get(idA[3]) };
    BodyData* db[4] = { &pool.get(idB[0]), &pool.get(idB[1]), &pool.get(idB[2]), &pool.get(idB[3]) };

    auto gatherVel = [&](float* vax, float* vay, float* vaz,
                         float* wax, float* way, float* waz,
                         float* vbx, float* vby, float* vbz,
                         float* wbx, float* wby, float* wbz) {
        for (int k = 0; k < 4; ++k) {
            vax[k] = da[k]->linearVelocity.x();  vay[k] = da[k]->linearVelocity.y();  vaz[k] = da[k]->linearVelocity.z();
            wax[k] = da[k]->angularVelocity.x(); way[k] = da[k]->angularVelocity.y(); waz[k] = da[k]->angularVelocity.z();
            vbx[k] = db[k]->linearVelocity.x();  vby[k] = db[k]->linearVelocity.y();  vbz[k] = db[k]->linearVelocity.z();
            wbx[k] = db[k]->angularVelocity.x(); wby[k] = db[k]->angularVelocity.y(); wbz[k] = db[k]->angularVelocity.z();
        }
    };

    alignas(16) float vax[4], vay[4], vaz[4];
    alignas(16) float wax[4], way[4], waz[4];
    alignas(16) float vbx[4], vby[4], vbz[4];
    alignas(16) float wbx[4], wby[4], wbz[4];
    alignas(16) float dLArr[4];
    alignas(16) float lambdaArr[4];

    // ---- Normal ----
    gatherVel(vax, vay, vaz, wax, way, waz, vbx, vby, vbz, wbx, wby, wbz);

    __m128 jv = _mm_add_ps(
        _mm_add_ps(mm_dot3_4wide(vax, vay, vaz, &soa.J_va_nx[i], &soa.J_va_ny[i], &soa.J_va_nz[i]),
                   mm_dot3_4wide(wax, way, waz, &soa.J_wa_nx[i], &soa.J_wa_ny[i], &soa.J_wa_nz[i])),
        _mm_add_ps(mm_dot3_4wide(vbx, vby, vbz, &soa.J_vb_nx[i], &soa.J_vb_ny[i], &soa.J_vb_nz[i]),
                   mm_dot3_4wide(wbx, wby, wbz, &soa.J_wb_nx[i], &soa.J_wb_ny[i], &soa.J_wb_nz[i]))
    );

    __m128 effMass = _mm_loadu_ps(&soa.effMassN[i]);
    __m128 bias    = _mm_loadu_ps(&soa.biasN[i]);
    __m128 lambdaOld = _mm_loadu_ps(&soa.lambdaN[i]);

    __m128 dL = _mm_sub_ps(_mm_setzero_ps(), _mm_mul_ps(effMass, _mm_add_ps(jv, bias)));
    __m128 lambdaNew = _mm_max_ps(_mm_setzero_ps(), _mm_add_ps(lambdaOld, dL));
    dL = _mm_sub_ps(lambdaNew, lambdaOld);

    _mm_storeu_ps(dLArr, dL);
    _mm_storeu_ps(lambdaArr, lambdaNew);

    for (int k = 0; k < 4; ++k) {
        soa.lambdaN[i + k] = lambdaArr[k];
        applyImpulseRow(*da[k], *db[k],
            vm::Vector3<float>(soa.J_va_nx[i+k], soa.J_va_ny[i+k], soa.J_va_nz[i+k]),
            vm::Vector3<float>(soa.J_wa_nx[i+k], soa.J_wa_ny[i+k], soa.J_wa_nz[i+k]),
            vm::Vector3<float>(soa.J_vb_nx[i+k], soa.J_vb_ny[i+k], soa.J_vb_nz[i+k]),
            vm::Vector3<float>(soa.J_wb_nx[i+k], soa.J_wb_ny[i+k], soa.J_wb_nz[i+k]),
            dLArr[k]);
    }

    // ---- Tangent 0 ----
    gatherVel(vax, vay, vaz, wax, way, waz, vbx, vby, vbz, wbx, wby, wbz);

    __m128 limit = _mm_mul_ps(
        _mm_loadu_ps(&soa.friction[i]),
        _mm_loadu_ps(&soa.lambdaN[i]));

    jv = _mm_add_ps(
        _mm_add_ps(mm_dot3_4wide(vax, vay, vaz, &soa.J_va_t0x[i], &soa.J_va_t0y[i], &soa.J_va_t0z[i]),
                   mm_dot3_4wide(wax, way, waz, &soa.J_wa_t0x[i], &soa.J_wa_t0y[i], &soa.J_wa_t0z[i])),
        _mm_add_ps(mm_dot3_4wide(vbx, vby, vbz, &soa.J_vb_t0x[i], &soa.J_vb_t0y[i], &soa.J_vb_t0z[i]),
                   mm_dot3_4wide(wbx, wby, wbz, &soa.J_wb_t0x[i], &soa.J_wb_t0y[i], &soa.J_wb_t0z[i]))
    );

    effMass = _mm_loadu_ps(&soa.effMassT0[i]);
    lambdaOld = _mm_loadu_ps(&soa.lambdaT0[i]);

    dL = _mm_sub_ps(_mm_setzero_ps(), _mm_mul_ps(effMass, jv));
    lambdaNew = _mm_min_ps(limit, _mm_max_ps(_mm_sub_ps(_mm_setzero_ps(), limit), _mm_add_ps(lambdaOld, dL)));
    dL = _mm_sub_ps(lambdaNew, lambdaOld);

    _mm_storeu_ps(dLArr, dL);
    _mm_storeu_ps(lambdaArr, lambdaNew);

    for (int k = 0; k < 4; ++k) {
        soa.lambdaT0[i + k] = lambdaArr[k];
        applyImpulseRow(*da[k], *db[k],
            vm::Vector3<float>(soa.J_va_t0x[i+k], soa.J_va_t0y[i+k], soa.J_va_t0z[i+k]),
            vm::Vector3<float>(soa.J_wa_t0x[i+k], soa.J_wa_t0y[i+k], soa.J_wa_t0z[i+k]),
            vm::Vector3<float>(soa.J_vb_t0x[i+k], soa.J_vb_t0y[i+k], soa.J_vb_t0z[i+k]),
            vm::Vector3<float>(soa.J_wb_t0x[i+k], soa.J_wb_t0y[i+k], soa.J_wb_t0z[i+k]),
            dLArr[k]);
    }

    // ---- Tangent 1 ----
    gatherVel(vax, vay, vaz, wax, way, waz, vbx, vby, vbz, wbx, wby, wbz);

    limit = _mm_mul_ps(
        _mm_loadu_ps(&soa.friction[i]),
        _mm_loadu_ps(&soa.lambdaN[i]));

    jv = _mm_add_ps(
        _mm_add_ps(mm_dot3_4wide(vax, vay, vaz, &soa.J_va_t1x[i], &soa.J_va_t1y[i], &soa.J_va_t1z[i]),
                   mm_dot3_4wide(wax, way, waz, &soa.J_wa_t1x[i], &soa.J_wa_t1y[i], &soa.J_wa_t1z[i])),
        _mm_add_ps(mm_dot3_4wide(vbx, vby, vbz, &soa.J_vb_t1x[i], &soa.J_vb_t1y[i], &soa.J_vb_t1z[i]),
                   mm_dot3_4wide(wbx, wby, wbz, &soa.J_wb_t1x[i], &soa.J_wb_t1y[i], &soa.J_wb_t1z[i]))
    );

    effMass = _mm_loadu_ps(&soa.effMassT1[i]);
    lambdaOld = _mm_loadu_ps(&soa.lambdaT1[i]);

    dL = _mm_sub_ps(_mm_setzero_ps(), _mm_mul_ps(effMass, jv));
    lambdaNew = _mm_min_ps(limit, _mm_max_ps(_mm_sub_ps(_mm_setzero_ps(), limit), _mm_add_ps(lambdaOld, dL)));
    dL = _mm_sub_ps(lambdaNew, lambdaOld);

    _mm_storeu_ps(dLArr, dL);
    _mm_storeu_ps(lambdaArr, lambdaNew);

    for (int k = 0; k < 4; ++k) {
        soa.lambdaT1[i + k] = lambdaArr[k];
        applyImpulseRow(*da[k], *db[k],
            vm::Vector3<float>(soa.J_va_t1x[i+k], soa.J_va_t1y[i+k], soa.J_va_t1z[i+k]),
            vm::Vector3<float>(soa.J_wa_t1x[i+k], soa.J_wa_t1y[i+k], soa.J_wa_t1z[i+k]),
            vm::Vector3<float>(soa.J_vb_t1x[i+k], soa.J_vb_t1y[i+k], soa.J_vb_t1z[i+k]),
            vm::Vector3<float>(soa.J_wb_t1x[i+k], soa.J_wb_t1y[i+k], soa.J_wb_t1z[i+k]),
            dLArr[k]);
    }
}

// Solve 4 constraint rows simultaneously using SSE.
[[maybe_unused]] CAMPELLO_FORCE_INLINE void solveConstraintBatch4Sse(
        campello::physics::PhysicsWorld::ConstraintRowSoA& soa,
        campello::physics::BodyPool& pool, int i) {
    using namespace campello::physics;

    uint32_t idA[4] = { soa.bodyA[i], soa.bodyA[i+1], soa.bodyA[i+2], soa.bodyA[i+3] };
    uint32_t idB[4] = { soa.bodyB[i], soa.bodyB[i+1], soa.bodyB[i+2], soa.bodyB[i+3] };
    BodyData* da[4] = { &pool.get(idA[0]), &pool.get(idA[1]), &pool.get(idA[2]), &pool.get(idA[3]) };
    BodyData* db[4] = { &pool.get(idB[0]), &pool.get(idB[1]), &pool.get(idB[2]), &pool.get(idB[3]) };

    alignas(16) float vax[4], vay[4], vaz[4];
    alignas(16) float wax[4], way[4], waz[4];
    alignas(16) float vbx[4], vby[4], vbz[4];
    alignas(16) float wbx[4], wby[4], wbz[4];
    alignas(16) float dLArr[4];

    for (int k = 0; k < 4; ++k) {
        vax[k] = da[k]->linearVelocity.x();  vay[k] = da[k]->linearVelocity.y();  vaz[k] = da[k]->linearVelocity.z();
        wax[k] = da[k]->angularVelocity.x(); way[k] = da[k]->angularVelocity.y(); waz[k] = da[k]->angularVelocity.z();
        vbx[k] = db[k]->linearVelocity.x();  vby[k] = db[k]->linearVelocity.y();  vbz[k] = db[k]->linearVelocity.z();
        wbx[k] = db[k]->angularVelocity.x(); wby[k] = db[k]->angularVelocity.y(); wbz[k] = db[k]->angularVelocity.z();
    }

    __m128 jv = _mm_add_ps(
        _mm_add_ps(mm_dot3_4wide(vax, vay, vaz, &soa.J_va_x[i], &soa.J_va_y[i], &soa.J_va_z[i]),
                   mm_dot3_4wide(wax, way, waz, &soa.J_wa_x[i], &soa.J_wa_y[i], &soa.J_wa_z[i])),
        _mm_add_ps(mm_dot3_4wide(vbx, vby, vbz, &soa.J_vb_x[i], &soa.J_vb_y[i], &soa.J_vb_z[i]),
                   mm_dot3_4wide(wbx, wby, wbz, &soa.J_wb_x[i], &soa.J_wb_y[i], &soa.J_wb_z[i]))
    );

    __m128 effMass   = _mm_loadu_ps(&soa.effMass[i]);
    __m128 bias      = _mm_loadu_ps(&soa.bias[i]);
    __m128 lambdaOld = _mm_loadu_ps(&soa.lambda[i]);
    __m128 lambdaMin = _mm_loadu_ps(&soa.lambdaMin[i]);
    __m128 lambdaMax = _mm_loadu_ps(&soa.lambdaMax[i]);

    __m128 dL = _mm_sub_ps(_mm_setzero_ps(), _mm_mul_ps(effMass, _mm_add_ps(jv, bias)));
    __m128 lambdaNew = _mm_min_ps(lambdaMax, _mm_max_ps(lambdaMin, _mm_add_ps(lambdaOld, dL)));
    dL = _mm_sub_ps(lambdaNew, lambdaOld);

    _mm_storeu_ps(dLArr, dL);
    _mm_storeu_ps(&soa.lambda[i], lambdaNew);

    for (int k = 0; k < 4; ++k) {
        applyImpulseRow(*da[k], *db[k],
            vm::Vector3<float>(soa.J_va_x[i+k], soa.J_va_y[i+k], soa.J_va_z[i+k]),
            vm::Vector3<float>(soa.J_wa_x[i+k], soa.J_wa_y[i+k], soa.J_wa_z[i+k]),
            vm::Vector3<float>(soa.J_vb_x[i+k], soa.J_vb_y[i+k], soa.J_vb_z[i+k]),
            vm::Vector3<float>(soa.J_wb_x[i+k], soa.J_wb_y[i+k], soa.J_wb_z[i+k]),
            dLArr[k]);
    }
}

#endif // __SSE__ || __AVX__

} // namespace

static void solveConstraintSoAScalar(
        campello::physics::PhysicsWorld::ConstraintRowSoA& soa,
        campello::physics::BodyPool& pool, int i) {
    using namespace campello::physics;
    auto& da = pool.get(soa.bodyA[i]);
    auto& db = pool.get(soa.bodyB[i]);

    float Jv = soa.J_va_x[i] * da.linearVelocity.x() + soa.J_va_y[i] * da.linearVelocity.y() + soa.J_va_z[i] * da.linearVelocity.z()
             + soa.J_wa_x[i] * da.angularVelocity.x() + soa.J_wa_y[i] * da.angularVelocity.y() + soa.J_wa_z[i] * da.angularVelocity.z()
             + soa.J_vb_x[i] * db.linearVelocity.x() + soa.J_vb_y[i] * db.linearVelocity.y() + soa.J_vb_z[i] * db.linearVelocity.z()
             + soa.J_wb_x[i] * db.angularVelocity.x() + soa.J_wb_y[i] * db.angularVelocity.y() + soa.J_wb_z[i] * db.angularVelocity.z();

    float dL  = soa.effMass[i] * -(Jv + soa.bias[i]);
    float lOld = soa.lambda[i];
    soa.lambda[i] = std::clamp(soa.lambda[i] + dL, soa.lambdaMin[i], soa.lambdaMax[i]);
    dL = soa.lambda[i] - lOld;

    applyImpulseRow(da, db,
        vm::Vector3<float>(soa.J_va_x[i], soa.J_va_y[i], soa.J_va_z[i]),
        vm::Vector3<float>(soa.J_wa_x[i], soa.J_wa_y[i], soa.J_wa_z[i]),
        vm::Vector3<float>(soa.J_vb_x[i], soa.J_vb_y[i], soa.J_vb_z[i]),
        vm::Vector3<float>(soa.J_wb_x[i], soa.J_wb_y[i], soa.J_wb_z[i]),
        dL);
}

void PhysicsWorld::buildContactSoA() {
    const size_t n = m_contactPoints.size();
    if (n == 0) {
        m_contactSoA.clear();
        m_contactSoA.colorOffsets.clear();
        m_contactSoA.colorCounts.clear();
        return;
    }
    m_contactSoA.resize(n);
    m_contactSoA.colorOffsets.clear();
    m_contactSoA.colorCounts.clear();
    size_t soaIdx = 0;
    for (const auto& color : m_globalContactColors) {
        m_contactSoA.colorOffsets.push_back(static_cast<int>(soaIdx));
        m_contactSoA.colorCounts.push_back(static_cast<int>(color.size()));
        for (int cpIdx : color) {
            const auto& p = m_contactPoints[cpIdx];
            m_contactSoA.contactIndex[soaIdx] = cpIdx;
            m_contactSoA.bodyA[soaIdx] = p.bodyA;
            m_contactSoA.bodyB[soaIdx] = p.bodyB;
            m_contactSoA.J_va_nx[soaIdx] = p.J_va_n.x(); m_contactSoA.J_va_ny[soaIdx] = p.J_va_n.y(); m_contactSoA.J_va_nz[soaIdx] = p.J_va_n.z();
            m_contactSoA.J_wa_nx[soaIdx] = p.J_wa_n.x(); m_contactSoA.J_wa_ny[soaIdx] = p.J_wa_n.y(); m_contactSoA.J_wa_nz[soaIdx] = p.J_wa_n.z();
            m_contactSoA.J_vb_nx[soaIdx] = p.J_vb_n.x(); m_contactSoA.J_vb_ny[soaIdx] = p.J_vb_n.y(); m_contactSoA.J_vb_nz[soaIdx] = p.J_vb_n.z();
            m_contactSoA.J_wb_nx[soaIdx] = p.J_wb_n.x(); m_contactSoA.J_wb_ny[soaIdx] = p.J_wb_n.y(); m_contactSoA.J_wb_nz[soaIdx] = p.J_wb_n.z();
            m_contactSoA.J_va_t0x[soaIdx] = p.J_va_t0.x(); m_contactSoA.J_va_t0y[soaIdx] = p.J_va_t0.y(); m_contactSoA.J_va_t0z[soaIdx] = p.J_va_t0.z();
            m_contactSoA.J_wa_t0x[soaIdx] = p.J_wa_t0.x(); m_contactSoA.J_wa_t0y[soaIdx] = p.J_wa_t0.y(); m_contactSoA.J_wa_t0z[soaIdx] = p.J_wa_t0.z();
            m_contactSoA.J_vb_t0x[soaIdx] = p.J_vb_t0.x(); m_contactSoA.J_vb_t0y[soaIdx] = p.J_vb_t0.y(); m_contactSoA.J_vb_t0z[soaIdx] = p.J_vb_t0.z();
            m_contactSoA.J_wb_t0x[soaIdx] = p.J_wb_t0.x(); m_contactSoA.J_wb_t0y[soaIdx] = p.J_wb_t0.y(); m_contactSoA.J_wb_t0z[soaIdx] = p.J_wb_t0.z();
            m_contactSoA.J_va_t1x[soaIdx] = p.J_va_t1.x(); m_contactSoA.J_va_t1y[soaIdx] = p.J_va_t1.y(); m_contactSoA.J_va_t1z[soaIdx] = p.J_va_t1.z();
            m_contactSoA.J_wa_t1x[soaIdx] = p.J_wa_t1.x(); m_contactSoA.J_wa_t1y[soaIdx] = p.J_wa_t1.y(); m_contactSoA.J_wa_t1z[soaIdx] = p.J_wa_t1.z();
            m_contactSoA.J_vb_t1x[soaIdx] = p.J_vb_t1.x(); m_contactSoA.J_vb_t1y[soaIdx] = p.J_vb_t1.y(); m_contactSoA.J_vb_t1z[soaIdx] = p.J_vb_t1.z();
            m_contactSoA.J_wb_t1x[soaIdx] = p.J_wb_t1.x(); m_contactSoA.J_wb_t1y[soaIdx] = p.J_wb_t1.y(); m_contactSoA.J_wb_t1z[soaIdx] = p.J_wb_t1.z();
            m_contactSoA.effMassN[soaIdx]  = p.effMassN;
            m_contactSoA.effMassT0[soaIdx] = p.effMassT0;
            m_contactSoA.effMassT1[soaIdx] = p.effMassT1;
            m_contactSoA.biasN[soaIdx]     = p.biasN;
            m_contactSoA.friction[soaIdx]  = p.friction;
            m_contactSoA.lambdaN[soaIdx]   = p.lambdaN;
            m_contactSoA.lambdaT0[soaIdx]  = p.lambdaT0;
            m_contactSoA.lambdaT1[soaIdx]  = p.lambdaT1;
            ++soaIdx;
        }
    }
}

// Scalar fallback that operates directly on SoA data (avoids AoS↔SoA mismatch)
static void solveContactSoAScalar(
        campello::physics::PhysicsWorld::ContactSolverSoA& soa,
        campello::physics::BodyPool& pool, int i) {
    using namespace campello::physics;
    auto& da = pool.get(soa.bodyA[i]);
    auto& db = pool.get(soa.bodyB[i]);

    // Normal row
    {
        float Jv = soa.J_va_nx[i] * da.linearVelocity.x() + soa.J_va_ny[i] * da.linearVelocity.y() + soa.J_va_nz[i] * da.linearVelocity.z()
                 + soa.J_wa_nx[i] * da.angularVelocity.x() + soa.J_wa_ny[i] * da.angularVelocity.y() + soa.J_wa_nz[i] * da.angularVelocity.z()
                 + soa.J_vb_nx[i] * db.linearVelocity.x() + soa.J_vb_ny[i] * db.linearVelocity.y() + soa.J_vb_nz[i] * db.linearVelocity.z()
                 + soa.J_wb_nx[i] * db.angularVelocity.x() + soa.J_wb_ny[i] * db.angularVelocity.y() + soa.J_wb_nz[i] * db.angularVelocity.z();
        float dL  = soa.effMassN[i] * -(Jv + soa.biasN[i]);
        float lOld = soa.lambdaN[i];
        soa.lambdaN[i] = std::max(0.f, soa.lambdaN[i] + dL);
        dL = soa.lambdaN[i] - lOld;
        applyImpulseRow(da, db,
            vm::Vector3<float>(soa.J_va_nx[i], soa.J_va_ny[i], soa.J_va_nz[i]),
            vm::Vector3<float>(soa.J_wa_nx[i], soa.J_wa_ny[i], soa.J_wa_nz[i]),
            vm::Vector3<float>(soa.J_vb_nx[i], soa.J_vb_ny[i], soa.J_vb_nz[i]),
            vm::Vector3<float>(soa.J_wb_nx[i], soa.J_wb_ny[i], soa.J_wb_nz[i]),
            dL);
    }

    float limit = soa.friction[i] * soa.lambdaN[i];
    {
        float Jv = soa.J_va_t0x[i] * da.linearVelocity.x() + soa.J_va_t0y[i] * da.linearVelocity.y() + soa.J_va_t0z[i] * da.linearVelocity.z()
                 + soa.J_wa_t0x[i] * da.angularVelocity.x() + soa.J_wa_t0y[i] * da.angularVelocity.y() + soa.J_wa_t0z[i] * da.angularVelocity.z()
                 + soa.J_vb_t0x[i] * db.linearVelocity.x() + soa.J_vb_t0y[i] * db.linearVelocity.y() + soa.J_vb_t0z[i] * db.linearVelocity.z()
                 + soa.J_wb_t0x[i] * db.angularVelocity.x() + soa.J_wb_t0y[i] * db.angularVelocity.y() + soa.J_wb_t0z[i] * db.angularVelocity.z();
        float dL  = soa.effMassT0[i] * -Jv;
        float lOld = soa.lambdaT0[i];
        soa.lambdaT0[i] = std::clamp(soa.lambdaT0[i] + dL, -limit, limit);
        dL = soa.lambdaT0[i] - lOld;
        applyImpulseRow(da, db,
            vm::Vector3<float>(soa.J_va_t0x[i], soa.J_va_t0y[i], soa.J_va_t0z[i]),
            vm::Vector3<float>(soa.J_wa_t0x[i], soa.J_wa_t0y[i], soa.J_wa_t0z[i]),
            vm::Vector3<float>(soa.J_vb_t0x[i], soa.J_vb_t0y[i], soa.J_vb_t0z[i]),
            vm::Vector3<float>(soa.J_wb_t0x[i], soa.J_wb_t0y[i], soa.J_wb_t0z[i]),
            dL);
    }
    {
        float Jv = soa.J_va_t1x[i] * da.linearVelocity.x() + soa.J_va_t1y[i] * da.linearVelocity.y() + soa.J_va_t1z[i] * da.linearVelocity.z()
                 + soa.J_wa_t1x[i] * da.angularVelocity.x() + soa.J_wa_t1y[i] * da.angularVelocity.y() + soa.J_wa_t1z[i] * da.angularVelocity.z()
                 + soa.J_vb_t1x[i] * db.linearVelocity.x() + soa.J_vb_t1y[i] * db.linearVelocity.y() + soa.J_vb_t1z[i] * db.linearVelocity.z()
                 + soa.J_wb_t1x[i] * db.angularVelocity.x() + soa.J_wb_t1y[i] * db.angularVelocity.y() + soa.J_wb_t1z[i] * db.angularVelocity.z();
        float dL  = soa.effMassT1[i] * -Jv;
        float lOld = soa.lambdaT1[i];
        soa.lambdaT1[i] = std::clamp(soa.lambdaT1[i] + dL, -limit, limit);
        dL = soa.lambdaT1[i] - lOld;
        applyImpulseRow(da, db,
            vm::Vector3<float>(soa.J_va_t1x[i], soa.J_va_t1y[i], soa.J_va_t1z[i]),
            vm::Vector3<float>(soa.J_wa_t1x[i], soa.J_wa_t1y[i], soa.J_wa_t1z[i]),
            vm::Vector3<float>(soa.J_vb_t1x[i], soa.J_vb_t1y[i], soa.J_vb_t1z[i]),
            vm::Vector3<float>(soa.J_wb_t1x[i], soa.J_wb_t1y[i], soa.J_wb_t1z[i]),
            dL);
    }
}

void PhysicsWorld::solveContactSoA(float /*dt*/) {
    const int n = static_cast<int>(m_contactPoints.size());
    if (n == 0) return;

    const int colorCount = static_cast<int>(m_contactSoA.colorCounts.size());
    for (int iter = 0; iter < contactIterations; ++iter) {
        for (int c = 0; c < colorCount; ++c) {
            int offset = m_contactSoA.colorOffsets[c];
            int count  = m_contactSoA.colorCounts[c];
            int i = offset;
#if defined(__SSE__) || defined(__AVX__)
            for (; i + 4 <= offset + count; i += 4) {
                solveContactBatch4Sse(m_contactSoA, m_pool, i);
            }
#endif
            for (; i < offset + count; ++i) {
                solveContactSoAScalar(m_contactSoA, m_pool, i);
            }
        }
    }

    // Copy lambdas back to m_contactPoints for warm-start persistence
    for (size_t i = 0; i < m_contactPoints.size(); ++i) {
        int cpIdx = m_contactSoA.contactIndex[i];
        m_contactPoints[cpIdx].lambdaN  = m_contactSoA.lambdaN[i];
        m_contactPoints[cpIdx].lambdaT0 = m_contactSoA.lambdaT0[i];
        m_contactPoints[cpIdx].lambdaT1 = m_contactSoA.lambdaT1[i];
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
    const int solverIterations = m_constraintSolver.iterations;

    // ── Constraint solve (per-island, constraint-level color + row-batch SIMD) ───
    auto solveConstraints = [&](int first, int last) {
        // Thread-local temporary buffers (reused across islands)
        ConstraintRowSoA soa;
        std::vector<ConstraintRow*> rowPtrs;
        std::vector<std::vector<int>> colorRowGroupEnds;
        std::vector<Constraint*> awakeConstraints;
        std::vector<std::vector<Constraint*>> constraintColors;

        for (int i = first; i < last; ++i) {
            const auto& island = m_islands[static_cast<size_t>(i)];
            // Skip fully sleeping islands
            bool allSleeping = true;
            for (uint32_t id : island.bodyIds)
                if (!m_pool.get(id).isSleeping) { allSleeping = false; break; }
            if (allSleeping) continue;

            // Collect awake constraints for this island
            awakeConstraints.clear();
            awakeConstraints.reserve(island.constraintPtrs.size());
            for (auto* c : island.constraintPtrs) {
                bool aSleep = m_pool.get(c->bodyA().id()).isSleeping;
                bool bSleep = m_pool.get(c->bodyB().id()).isSleeping;
                if (!aSleep || !bSleep) awakeConstraints.push_back(c);
            }

            // Scalar prepare
            for (auto* c : awakeConstraints)
                c->prepare(m_pool, dt, m_constraintSolver.baumgarte, m_constraintSolver.slop);

            if (awakeConstraints.empty()) continue;

            // Greedy coloring on CONSTRAINTS (not rows).
            // Two constraints conflict if they share a writable body.
            for (auto& cc : constraintColors) cc.clear();
            constraintColors.clear();
            for (auto* c : awakeConstraints) {
                uint32_t idA = c->bodyA().id();
                uint32_t idB = c->bodyB().id();
                bool aWritable = m_pool.get(idA).invMass != 0.f;
                bool bWritable = m_pool.get(idB).invMass != 0.f;

                int chosenColor = 0;
                bool placed = false;
                while (!placed) {
                    if (chosenColor >= static_cast<int>(constraintColors.size()))
                        constraintColors.emplace_back();
                    bool conflict = false;
                    for (auto* other : constraintColors[chosenColor]) {
                        uint32_t oA = other->bodyA().id();
                        uint32_t oB = other->bodyB().id();
                        if (aWritable && (idA == oA || idA == oB)) { conflict = true; break; }
                        if (bWritable && (idB == oA || idB == oB)) { conflict = true; break; }
                    }
                    if (!conflict) {
                        constraintColors[chosenColor].push_back(c);
                        placed = true;
                    } else {
                        ++chosenColor;
                    }
                }
            }

            // Count total rows and build SoA in row-index-major order per color.
            // Within each color, all row-0s come first, then row-1s, etc.
            // This lets us batch 4-wide SSE on rows from different constraints
            // while preserving Gauss-Seidel convergence within each constraint.
            int totalRows = 0;
            for (auto& color : constraintColors) {
                for (auto* c : color) totalRows += c->rowCount();
            }

            soa.resize(totalRows);
            rowPtrs.resize(totalRows);
            colorRowGroupEnds.resize(constraintColors.size());
            soa.colorOffsets.clear();
            soa.colorCounts.clear();
            int soaIdx = 0;

            for (int ci = 0; ci < static_cast<int>(constraintColors.size()); ++ci) {
                auto& color = constraintColors[ci];
                int maxRows = 0;
                for (auto* c : color) maxRows = std::max(maxRows, c->rowCount());

                soa.colorOffsets.push_back(soaIdx);
                colorRowGroupEnds[ci].clear();

                for (int rowIdx = 0; rowIdx < maxRows; ++rowIdx) {
                    for (auto* c : color) {
                        if (c->rowCount() <= rowIdx) continue;
                        auto* rows = c->rows();
                        const auto& row = rows[rowIdx];
                        uint32_t idA = c->bodyA().id();
                        uint32_t idB = c->bodyB().id();

                        soa.bodyA[soaIdx]      = idA;
                        soa.bodyB[soaIdx]      = idB;
                        soa.J_va_x[soaIdx]     = row.J_va.x();
                        soa.J_va_y[soaIdx]     = row.J_va.y();
                        soa.J_va_z[soaIdx]     = row.J_va.z();
                        soa.J_wa_x[soaIdx]     = row.J_wa.x();
                        soa.J_wa_y[soaIdx]     = row.J_wa.y();
                        soa.J_wa_z[soaIdx]     = row.J_wa.z();
                        soa.J_vb_x[soaIdx]     = row.J_vb.x();
                        soa.J_vb_y[soaIdx]     = row.J_vb.y();
                        soa.J_vb_z[soaIdx]     = row.J_vb.z();
                        soa.J_wb_x[soaIdx]     = row.J_wb.x();
                        soa.J_wb_y[soaIdx]     = row.J_wb.y();
                        soa.J_wb_z[soaIdx]     = row.J_wb.z();
                        soa.effMass[soaIdx]    = row.effMass;
                        soa.bias[soaIdx]       = row.bias;
                        soa.lambdaMin[soaIdx]  = row.lambdaMin;
                        soa.lambdaMax[soaIdx]  = row.lambdaMax;
                        soa.lambda[soaIdx]     = row.lambda;
                        rowPtrs[soaIdx] = &rows[rowIdx];
                        ++soaIdx;
                    }
                    colorRowGroupEnds[ci].push_back(soaIdx);
                }
                soa.colorCounts.push_back(soaIdx - soa.colorOffsets.back());
            }

            // Scalar warm start (apply previous lambdas scaled by 0.85f)
            for (int r = 0; r < totalRows; ++r) {
                auto& da = m_pool.get(soa.bodyA[r]);
                auto& db = m_pool.get(soa.bodyB[r]);
                applyImpulseRow(da, db,
                    vm::Vector3<float>(soa.J_va_x[r], soa.J_va_y[r], soa.J_va_z[r]),
                    vm::Vector3<float>(soa.J_wa_x[r], soa.J_wa_y[r], soa.J_wa_z[r]),
                    vm::Vector3<float>(soa.J_vb_x[r], soa.J_vb_y[r], soa.J_vb_z[r]),
                    vm::Vector3<float>(soa.J_wb_x[r], soa.J_wb_y[r], soa.J_wb_z[r]),
                    soa.lambda[r] * 0.85f);
            }

            // Solve iterations: row-group-major within each color, 4-wide SSE batches
            for (int iter = 0; iter < solverIterations; ++iter) {
                for (int ci = 0; ci < static_cast<int>(constraintColors.size()); ++ci) {
                    int colorOffset = soa.colorOffsets[ci];
                    int prevEnd = colorOffset;
                    for (int groupEnd : colorRowGroupEnds[ci]) {
                        int j = prevEnd;
#if defined(__SSE__) || defined(__AVX__)
                        for (; j + 4 <= groupEnd; j += 4) {
                            solveConstraintBatch4Sse(soa, m_pool, j);
                        }
#endif
                        for (; j < groupEnd; ++j) {
                            solveConstraintSoAScalar(soa, m_pool, j);
                        }
                        prevEnd = groupEnd;
                    }
                }
            }

            // Copy lambdas back to constraint rows for warm-start persistence
            for (int r = 0; r < totalRows; ++r) {
                rowPtrs[r]->lambda = soa.lambda[r];
            }
        }
    };

    const int islandCount = static_cast<int>(m_islands.size());
    if (m_workerThreads > 1 && islandCount >= 2) {
        m_threadPool->parallelFor(islandCount, solveConstraints);
    } else {
        solveConstraints(0, islandCount);
    }

    // ── Contact solve (SoA SIMD batch) ──────────────────────────────────────────
    buildContactSoA();
    solveContactSoA(dt);
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

void PhysicsWorld::buildContactColors() {
    const int n = static_cast<int>(m_contactPoints.size());
    if (n == 0) return;

    // Reset colors
    for (auto& p : m_contactPoints) p.color = 0;

    // Greedy graph coloring: two contacts conflict if they share a writable body.
    // Static bodies (invMass == 0) are skipped because applyImpulseRow never writes to them.
    std::vector<std::vector<int>> colors; // colors[c] = list of contact indices with color c

    for (int i = 0; i < n; ++i) {
        const auto& cp = m_contactPoints[i];
        const bool aWritable = m_pool.get(cp.bodyA).invMass != 0.f;
        const bool bWritable = m_pool.get(cp.bodyB).invMass != 0.f;

        int chosenColor = 0;
        bool placed = false;
        while (!placed) {
            if (chosenColor >= static_cast<int>(colors.size())) {
                colors.emplace_back();
            }
            bool conflict = false;
            for (int j : colors[chosenColor]) {
                const auto& other = m_contactPoints[j];
                if (aWritable && (cp.bodyA == other.bodyA || cp.bodyA == other.bodyB)) { conflict = true; break; }
                if (bWritable && (cp.bodyB == other.bodyA || cp.bodyB == other.bodyB)) { conflict = true; break; }
            }
            if (!conflict) {
                colors[chosenColor].push_back(i);
                m_contactPoints[i].color = static_cast<uint8_t>(chosenColor);
                placed = true;
            } else {
                ++chosenColor;
            }
        }
    }
}

void PhysicsWorld::buildGlobalContactColors() {
    m_globalContactColors.clear();
    for (int i = 0; i < static_cast<int>(m_contactPoints.size()); ++i) {
        uint8_t c = m_contactPoints[i].color;
        if (c >= m_globalContactColors.size())
            m_globalContactColors.resize(c + 1);
        m_globalContactColors[c].push_back(i);
    }
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
        AABB aabbCur = computeBodyAABB(da);
        Transform futureT(da.transform.position + da.linearVelocity * dt,
                          da.transform.rotation);
        AABB aabbFut = computeBodyAABB(da.shape.get(), futureT);
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
            AABB aabbB = computeBodyAABB(db);
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
            p.depth = 0.f;  // speculative contacts have no penetration

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
