#pragma once

#include <campello_physics/body_pool.h>
#include <campello_physics/body_interface.h>
#include <campello_physics/broad_phase.h>
#include <campello_physics/narrow_phase.h>
#include <campello_physics/constraint.h>
#include <campello_physics/integrator.h>
#include <campello_physics/listeners.h>
#include <campello_physics/query.h>
#include <campello_physics/buoyancy.h>
#include <campello_physics/articulated_body.h>
#include <campello_physics/ragdoll.h>
#include <campello_physics/vehicle.h>
#include <campello_physics/debug_draw.h>
#include <campello_physics/profiler.h>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <vector>

namespace campello::physics {

// ── GPU backend selection ────────────────────────────────────────────────────
// Cpu  — always uses the Sequential Impulse CPU solver (default).
// Gpu  — uses the XPBD GPU solver via campello_gpu (requires CAMPELLO_PHYSICS_GPU=ON).
//        Falls back to Cpu if campello_gpu is unavailable or device creation fails.
// Auto — uses Gpu when active dynamic body count >= gpuBodyThreshold, else Cpu.
//        Behavioral note: CPU (SI) and GPU (XPBD) are different algorithms;
//        switching between them changes simulation results.
enum class PhysicsBackend { Cpu, Gpu, Auto };

// Forward-declare the internal GPU backend interface (full type in src/gpu/)
class IGpuBackend;

// Forward-declare the internal thread pool (full type in src/foundation/thread_pool.h)
class ThreadPool;

// ── PhysicsWorld ──────────────────────────────────────────────────────────────
//
// Simulation root.  Owns a BodyPool, BroadPhase, NarrowPhase,
// ConstraintSolver, and Integrator.  Drives the pipeline every step:
//
//   step(dt):
//     accumulate dt → run N fixed-timestep substeps
//
//   substep(fixedDt):
//     onPreStep → broadphase update → narrow phase → contact solve
//     → constraint solve → integrate → onPostStep
//     → dispatch contact / trigger events

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();  // defined in physics_world.cpp (requires IGpuBackend to be complete)

    // ── Body lifecycle ────────────────────────────────────────────────────────
    [[nodiscard]] Body createBody(const BodyDescriptor& desc);
    void               destroyBody(Body body);

    // ── Constraints ───────────────────────────────────────────────────────────
    void addConstraint   (std::shared_ptr<Constraint> c);
    void removeConstraint(const std::shared_ptr<Constraint>& c);

    // ── Simulation settings ───────────────────────────────────────────────────
    void setGravity       (const vm::Vector3<float>& g) noexcept;
    void setFixedTimestep (float dt) noexcept;
    void setSubsteps      (int n)    noexcept;
    // Number of worker threads used for the parallel broadphase, narrowphase,
    // and island solver. 1 = serial (default). Set before the first step() call.
    void setWorkerThreads (int n)    noexcept;

    [[nodiscard]] vm::Vector3<float> gravity()        const noexcept;
    [[nodiscard]] float              fixedTimestep()  const noexcept;

    // Solver iteration counts (applied inside each substep)
    int contactIterations    = 10;
    int constraintIterations = 10;

    // Contact solver parameters
    float contactBaumgarte = 0.2f;
    float contactSlop      = 0.005f;

    // Position solve (split-impulse pass after integration).
    // positionIterations > 0 enables it.  positionCorrectionAlpha: gain [0,1].
    int   positionIterations      = 3;
    float positionCorrectionAlpha = 0.2f;

    // ── Buoyancy volumes ──────────────────────────────────────────────────────
    [[nodiscard]] BuoyancyVolume addBuoyancyVolume   (const BuoyancyDescriptor& desc);
    void                         removeBuoyancyVolume(BuoyancyVolume vol);

    // ── Articulated bodies ────────────────────────────────────────────────────
    [[nodiscard]] ArticulatedBody createArticulatedBody (const ArticulatedBodyDescriptor& desc);
    void                          destroyArticulatedBody(ArticulatedBody ab);

    // ── Ragdoll ───────────────────────────────────────────────────────────────
    [[nodiscard]] RagdollBody createRagdoll (const RagdollDescriptor& desc);
    void                      destroyRagdoll(RagdollBody& rb);

    // ── Profiler ──────────────────────────────────────────────────────────────
    // Optional: set before first step(). Pass nullptr to disable.
    void setProfiler(IPhysicsProfiler* profiler) noexcept { m_profiler = profiler; }

    // ── Debug draw ────────────────────────────────────────────────────────────
    // Draws the current simulation state via the supplied IDebugDraw.
    // flags controls what is rendered (see DebugDrawFlags).
    void debugDraw(IDebugDraw& draw,
                   DebugDrawFlags flags = DebugDrawFlags::All) const;

    // ── Serialization ─────────────────────────────────────────────────────────
    // serialize() returns a JSON string capturing world settings and all body
    // states (transform, velocity, shape, material properties).
    //
    // Limitations: constraints, buoyancy volumes, articulated bodies, vehicles,
    // and ragdolls are NOT serialized.  TriangleMesh/HeightField shapes are
    // written as a type-name stub; on deserialize their shape will be null —
    // the caller must re-attach the mesh after loading.
    [[nodiscard]] std::string serialize() const;

    // deserialize() replaces the current world state with the snapshot.
    // All existing bodies are destroyed first.  Returns false on parse error
    // (world is left in a partially reset state).
    bool deserialize(std::string_view json);

    // ── Vehicles ──────────────────────────────────────────────────────────────
    // createVehicle creates the chassis body internally (using desc.chassis).
    // The caller controls the vehicle by setting vb.throttle/braking/steering
    // and calling syncVehicleControls(vb) before world.step().
    [[nodiscard]] VehicleBody createVehicle (const VehicleDescriptor& desc);
    void                      destroyVehicle(VehicleBody vb);
    // Must be called each frame after updating vb.throttle/braking/steering.
    void syncVehicleControls(const VehicleBody& vb);
    [[nodiscard]] const WheelState& vehicleWheelState(const VehicleBody& vb, int i) const noexcept;
    [[nodiscard]] int               vehicleWheelCount(const VehicleBody& vb)        const noexcept;

    // ── Listeners ─────────────────────────────────────────────────────────────
    void addStepListener   (IStepListener*    l);
    void removeStepListener(IStepListener*    l);
    void addContactListener   (IContactListener* l);
    void removeContactListener(IContactListener* l);
    void addTriggerListener   (ITriggerListener* l);
    void removeTriggerListener(ITriggerListener* l);

    // ── GPU backend ───────────────────────────────────────────────────────────
    // Select the compute backend. Gpu/Auto require CAMPELLO_PHYSICS_GPU=ON.
    // gpuDevice: optional shared_ptr<cg::Device> from the graphics layer.
    //   Pass the same device the renderer uses so physics and graphics share
    //   one GPU context and can alias buffers without a CPU round-trip.
    //   If null (default), the backend creates and owns its own device.
    //   Falls back to Cpu silently if the device is unusable.
    void setBackend(PhysicsBackend backend,
                    std::shared_ptr<void> gpuDevice = nullptr);
    void setGpuBodyThreshold(int n) noexcept { m_gpuBodyThreshold = n; }
    [[nodiscard]] PhysicsBackend backend() const noexcept { return m_backend; }

    // Returns the GPU render transform buffer as a type-erased shared_ptr<cg::Buffer>.
    // Layout: GpuRenderTransform[pool.capacity()] — one {float4 pos_w, float4 rot_xyzw}
    // per pool slot, indexed by body ID. Bind as a read-only storage buffer in your
    // graphics shader. Valid (non-null) after the first step() on the Gpu/Auto backend.
    // Reallocated only when pool capacity grows; re-query each frame.
    [[nodiscard]] std::shared_ptr<void> gpuRenderBuffer() const noexcept;

    // ── Advance simulation ────────────────────────────────────────────────────
    // Accumulates dt and fires one substep per fixedTimestep elapsed.
    void step(float deltaTime);

    // ── Scene queries ─────────────────────────────────────────────────────────
    [[nodiscard]] std::optional<RaycastHit>
    raycastClosest(const Ray& ray, const QueryFilter& filter = {}) const;

    [[nodiscard]] std::vector<RaycastHit>
    raycastAll(const Ray& ray, const QueryFilter& filter = {}) const;

    [[nodiscard]] std::optional<ShapeCastHit>
    shapeCast(const Shape& shape, const Transform& shapeTransform,
              const vm::Vector3<float>& direction, float maxDistance,
              const QueryFilter& filter = {}) const;

    [[nodiscard]] std::vector<OverlapResult>
    overlap(const Shape& shape, const Transform& shapeTransform,
            const QueryFilter& filter = {}) const;

    // ── Thread-safe body interface ────────────────────────────────────────────
    // createBody / destroyBody / state mutations are guarded by an exclusive
    // lock that also serialises with step().  Reads are lock-free.
    [[nodiscard]] BodyInterface&       bodyInterface()       noexcept { return m_bodyInterface; }
    [[nodiscard]] const BodyInterface& bodyInterface() const noexcept { return m_bodyInterface; }

    // ── Direct access (for queries, debug rendering) ──────────────────────────
    [[nodiscard]] BodyPool&             bodyPool()         noexcept { return m_pool; }
    [[nodiscard]] const BodyPool&       bodyPool()   const noexcept { return m_pool; }
    [[nodiscard]] ConstraintSolver&     constraintSolver() noexcept { return m_constraintSolver; }
    [[nodiscard]] const NarrowPhase&    narrowPhase()const noexcept { return m_narrowPhase; }

private:
    PhysicsBackend m_backend         = PhysicsBackend::Cpu;
    int            m_gpuBodyThreshold = 1000;
    std::unique_ptr<IGpuBackend> m_gpuBackend;  // null when GPU unavailable

    BodyPool           m_pool;
    std::shared_mutex  m_bodyMutex;
    BodyInterface      m_bodyInterface{*this, m_pool, m_bodyMutex};  // must follow m_pool and m_bodyMutex
    BroadPhase         m_broadPhase;
    NarrowPhase        m_narrowPhase;
    ConstraintSolver   m_constraintSolver;
    IntegratorSettings m_integratorSettings;
    IPhysicsProfiler*                            m_profiler = nullptr;
    std::unique_ptr<class BuoyancySystem>        m_buoyancySystem;
    std::unique_ptr<class ArticulatedBodySystem> m_articulatedBodySystem;
    std::unique_ptr<class VehicleSystem>         m_vehicleSystem;

    float m_fixedTimestep  = 1.f / 60.f;
    int   m_substeps       = 1;
    float m_accumulator    = 0.f;
    std::unique_ptr<ThreadPool> m_threadPool;
    int   m_workerThreads  = 1;

    std::vector<IStepListener*>    m_stepListeners;
    std::vector<IContactListener*> m_contactListeners;
    std::vector<ITriggerListener*> m_triggerListeners;

    // Contact solver working data rebuilt each substep
    struct ContactSolverPoint {
        uint32_t bodyA, bodyB;
        vm::Vector3<float> rA, rB;
        vm::Vector3<float> J_va_n, J_wa_n, J_vb_n, J_wb_n;  // normal Jacobian
        vm::Vector3<float> J_va_t0, J_wa_t0, J_vb_t0, J_wb_t0;
        vm::Vector3<float> J_va_t1, J_wa_t1, J_vb_t1, J_wb_t1;
        float biasN, lambdaN, effMassN;
        float lambdaT0, effMassT0;
        float lambdaT1, effMassT1;
        float friction;
        float depth;   // penetration depth (for position solve)
        // warm-start back-pointers into the manifold cache
        float* wsiN;   float* wsiT0;   float* wsiT1;
        uint8_t color = 0;  // independent-set color for parallel solve
    };
    std::vector<ContactSolverPoint> m_contactPoints;

public:
    // SoA contact solver data (populated from m_contactPoints each substep)
    struct ContactSolverSoA {
        std::vector<uint32_t> bodyA, bodyB;
        // Normal Jacobians
        std::vector<float> J_va_nx, J_va_ny, J_va_nz;
        std::vector<float> J_wa_nx, J_wa_ny, J_wa_nz;
        std::vector<float> J_vb_nx, J_vb_ny, J_vb_nz;
        std::vector<float> J_wb_nx, J_wb_ny, J_wb_nz;
        // Tangent 0 Jacobians
        std::vector<float> J_va_t0x, J_va_t0y, J_va_t0z;
        std::vector<float> J_wa_t0x, J_wa_t0y, J_wa_t0z;
        std::vector<float> J_vb_t0x, J_vb_t0y, J_vb_t0z;
        std::vector<float> J_wb_t0x, J_wb_t0y, J_wb_t0z;
        // Tangent 1 Jacobians
        std::vector<float> J_va_t1x, J_va_t1y, J_va_t1z;
        std::vector<float> J_wa_t1x, J_wa_t1y, J_wa_t1z;
        std::vector<float> J_vb_t1x, J_vb_t1y, J_vb_t1z;
        std::vector<float> J_wb_t1x, J_wb_t1y, J_wb_t1z;
        // Solver state
        std::vector<float> effMassN, effMassT0, effMassT1;
        std::vector<float> biasN;
        std::vector<float> friction;
        std::vector<float> lambdaN, lambdaT0, lambdaT1;
        // Color-major ordering metadata (built from m_globalContactColors)
        std::vector<int> colorOffsets;   // start index in SoA for each color
        std::vector<int> colorCounts;    // number of contacts in each color
        std::vector<int> contactIndex;   // maps SoA index -> m_contactPoints index

        void resize(size_t n) {
            bodyA.resize(n); bodyB.resize(n);
            J_va_nx.resize(n); J_va_ny.resize(n); J_va_nz.resize(n);
            J_wa_nx.resize(n); J_wa_ny.resize(n); J_wa_nz.resize(n);
            J_vb_nx.resize(n); J_vb_ny.resize(n); J_vb_nz.resize(n);
            J_wb_nx.resize(n); J_wb_ny.resize(n); J_wb_nz.resize(n);
            J_va_t0x.resize(n); J_va_t0y.resize(n); J_va_t0z.resize(n);
            J_wa_t0x.resize(n); J_wa_t0y.resize(n); J_wa_t0z.resize(n);
            J_vb_t0x.resize(n); J_vb_t0y.resize(n); J_vb_t0z.resize(n);
            J_wb_t0x.resize(n); J_wb_t0y.resize(n); J_wb_t0z.resize(n);
            J_va_t1x.resize(n); J_va_t1y.resize(n); J_va_t1z.resize(n);
            J_wa_t1x.resize(n); J_wa_t1y.resize(n); J_wa_t1z.resize(n);
            J_vb_t1x.resize(n); J_vb_t1y.resize(n); J_vb_t1z.resize(n);
            J_wb_t1x.resize(n); J_wb_t1y.resize(n); J_wb_t1z.resize(n);
            effMassN.resize(n); effMassT0.resize(n); effMassT1.resize(n);
            biasN.resize(n); friction.resize(n);
            lambdaN.resize(n); lambdaT0.resize(n); lambdaT1.resize(n);
            contactIndex.resize(n);
        }
        void clear() {
            bodyA.clear(); bodyB.clear();
            J_va_nx.clear(); J_va_ny.clear(); J_va_nz.clear();
            J_wa_nx.clear(); J_wa_ny.clear(); J_wa_nz.clear();
            J_vb_nx.clear(); J_vb_ny.clear(); J_vb_nz.clear();
            J_wb_nx.clear(); J_wb_ny.clear(); J_wb_nz.clear();
            J_va_t0x.clear(); J_va_t0y.clear(); J_va_t0z.clear();
            J_wa_t0x.clear(); J_wa_t0y.clear(); J_wa_t0z.clear();
            J_vb_t0x.clear(); J_vb_t0y.clear(); J_vb_t0z.clear();
            J_wb_t0x.clear(); J_wb_t0y.clear(); J_wb_t0z.clear();
            J_va_t1x.clear(); J_va_t1y.clear(); J_va_t1z.clear();
            J_wa_t1x.clear(); J_wa_t1y.clear(); J_wa_t1z.clear();
            J_vb_t1x.clear(); J_vb_t1y.clear(); J_vb_t1z.clear();
            J_wb_t1x.clear(); J_wb_t1y.clear(); J_wb_t1z.clear();
            effMassN.clear(); effMassT0.clear(); effMassT1.clear();
            biasN.clear(); friction.clear();
            lambdaN.clear(); lambdaT0.clear(); lambdaT1.clear();
            colorOffsets.clear(); colorCounts.clear(); contactIndex.clear();
        }
    };

    // SoA constraint row solver data (populated from constraints each substep)
    struct ConstraintRowSoA {
        std::vector<uint32_t> bodyA, bodyB;
        std::vector<float> J_va_x, J_va_y, J_va_z;
        std::vector<float> J_wa_x, J_wa_y, J_wa_z;
        std::vector<float> J_vb_x, J_vb_y, J_vb_z;
        std::vector<float> J_wb_x, J_wb_y, J_wb_z;
        std::vector<float> effMass, bias, lambdaMin, lambdaMax, lambda;
        std::vector<int> colorOffsets;
        std::vector<int> colorCounts;
        std::vector<int> rowIndex; // maps SoA index -> (constraint, row) via m_constraintRowMap

        void resize(size_t n) {
            bodyA.resize(n); bodyB.resize(n);
            J_va_x.resize(n); J_va_y.resize(n); J_va_z.resize(n);
            J_wa_x.resize(n); J_wa_y.resize(n); J_wa_z.resize(n);
            J_vb_x.resize(n); J_vb_y.resize(n); J_vb_z.resize(n);
            J_wb_x.resize(n); J_wb_y.resize(n); J_wb_z.resize(n);
            effMass.resize(n); bias.resize(n); lambdaMin.resize(n); lambdaMax.resize(n); lambda.resize(n);
            rowIndex.resize(n);
        }
        void clear() {
            bodyA.clear(); bodyB.clear();
            J_va_x.clear(); J_va_y.clear(); J_va_z.clear();
            J_wa_x.clear(); J_wa_y.clear(); J_wa_z.clear();
            J_vb_x.clear(); J_vb_y.clear(); J_vb_z.clear();
            J_wb_x.clear(); J_wb_y.clear(); J_wb_z.clear();
            effMass.clear(); bias.clear(); lambdaMin.clear(); lambdaMax.clear(); lambda.clear();
            colorOffsets.clear(); colorCounts.clear(); rowIndex.clear();
        }
    };

private:
    ContactSolverSoA m_contactSoA;
    ConstraintRowSoA m_constraintSoA;
    std::vector<std::pair<Constraint*, int>> m_constraintRowMap;
    std::vector<std::vector<int>> m_globalConstraintColors;

    // Previous-frame manifold body-ID pairs (for event tracking)
    std::vector<uint64_t> m_prevContactKeys;
    std::vector<uint64_t> m_currContactKeys;

    // ── Island detection ──────────────────────────────────────────────────────
    // One island = one connected component of the contact/constraint graph.
    // Bodies in a sleeping island are skipped by the solver and integrator.
    struct Island {
        std::vector<uint32_t>   bodyIds;        // dynamic + kinematic only
        std::vector<int>        contactIndices; // into m_contactPoints
        std::vector<Constraint*> constraintPtrs; // non-owning; valid for one substep
    };
    std::vector<Island>    m_islands;
    std::vector<std::vector<int>> m_globalContactColors; // independent contact sets for parallel solve
    std::vector<uint32_t>  m_ufParent;  // union-find arrays, indexed by body pool ID
    std::vector<uint32_t>  m_ufRank;

    static constexpr uint32_t kInvalidUF = ~uint32_t(0);
    uint32_t ufFind(uint32_t x) noexcept;
    void     ufUnion(uint32_t x, uint32_t y) noexcept;

    void substep(float dt);
    void updateBroadPhase();
    void solveContactPositions(float dt);
    void solveIslandPositions(float dt);
    void buildContacts(float dt);   // builds m_contactPoints; warm start is separate
    void warmStartContacts();       // applies warm start after wake propagation
    void buildCcdContacts(float dt);
    void buildContactColors();      // greedy coloring for parallel contact solve
    void buildGlobalContactColors(); // flatten per-color contacts across all islands
    void buildIslands();            // union-find grouping + wake propagation
    void solveIslands(float dt);    // per-island constraint + contact solve
    void solveContactPoint(ContactSolverPoint& p);
    void buildContactSoA();         // AoS → SoA copy for SIMD batch solver
    void solveContactSoA(float dt); // SSE 4-wide batch solve
    void buildConstraintColors();      // greedy coloring for parallel constraint solve
    void buildGlobalConstraintColors(); // flatten per-color constraint rows
    void buildConstraintSoA();          // AoS → SoA copy for SIMD batch solver
    void solveConstraintSoA(float dt);  // SSE 4-wide batch solve
    void warmStartConstraints();        // SoA warm start (replaces per-constraint scalar)
    void updateIslandSleep();       // island-level simultaneous sleep decision
    void dispatchContactEvents();
    void dispatchTriggerEvents(const std::vector<CollisionPair>& added,
                               const std::vector<CollisionPair>& removed);
};

} // namespace campello::physics
