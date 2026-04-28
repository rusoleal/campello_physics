# todo.md — campello_physics

Implementation roadmap. Work through phases in order — each phase's output is required by the next.

---

## Phase 0 — Project Scaffolding

- [x] Root `CMakeLists.txt` with FetchContent for `vector_math` and GoogleTest
- [x] CMake options: `CAMPELLO_PHYSICS_BUILD_TESTS`, `CAMPELLO_PHYSICS_BUILD_EXAMPLES`, `CAMPELLO_PHYSICS_BUILD_BENCHMARKS`
- [x] Directory structure: `include/campello_physics/`, `src/`, `tests/`, `examples/`, `docs/`, `cmake/`
- [x] `.gitignore` (build dirs, IDE files, OS artifacts)
- [x] GitHub Actions CI: build + test on Ubuntu, macOS, Windows
- [x] iOS CMake toolchain (`cmake/ios.toolchain.cmake`)
- [x] Android CMake toolchain notes in README

---

## Phase 1 — Foundation & Primitives

- [x] Platform detection macros (`CAMPELLO_PLATFORM_WINDOWS`, `_MACOS`, `_LINUX`, `_IOS`, `_ANDROID`)
- [x] `CAMPELLO_FORCE_INLINE`, `CAMPELLO_NODISCARD`, `CAMPELLO_LIKELY/UNLIKELY` macros
- [x] Memory pool allocator (fixed-size block allocator for bodies and shapes)
- [x] `AABB` struct (min/max, expand, intersect, merge, surface area)
- [x] `Ray` struct (origin + direction + maxDistance)
- [x] `Transform` struct (position: `Vector3`, rotation: `Quaternion<float>`)
- [x] `Plane` struct (normal + distance)

---

## Phase 2 — Collision Shapes

- [x] `Shape` abstract base class with ref-counting (`std::shared_ptr`-compatible)
- [x] `ShapeType` enum (Box, Sphere, Capsule, Cylinder, ConvexHull, TriangleMesh, HeightField, Compound)
- [x] `BoxShape` (half-extents)
- [x] `SphereShape` (radius)
- [x] `CapsuleShape` (radius + half-height)
- [x] `CylinderShape` (radius + half-height)
- [x] `ConvexHullShape` (point cloud → AABB from points; full hull in Phase 4)
- [x] `TriangleMeshShape` (triangle soup + median-split BVH, static only)
- [x] `HeightFieldShape` (grid of heights, configurable scale)
- [x] `CompoundShape` (list of `{shared_ptr<Shape>, Transform}` sub-shapes)
- [x] `Shape::computeAABB(Transform) -> AABB` for all shape types
- [x] `Shape::computeLocalInertiaDiagonal(float mass) -> Vector3` for all shape types

---

## Phase 3 — Broad Phase

- [x] `BroadPhase` interface
- [x] Dynamic AABB BVH implementation (branch-and-bound sibling selection, fat AABB amortisation)
- [x] Static bucket (never moves) + dynamic bucket (updated each step)
- [x] `CollisionPair` struct (two body IDs, canonical ordering bodyA < bodyB)
- [x] Pair list: add new pairs, remove stale pairs each step (added/removed/persisted)
- [x] Layer/mask filtering at broad phase (skip pair if `(layerA & maskB) == 0`)
- [x] Unit tests: N bodies, verify expected pair count (14 tests)

---

## Phase 4 — Narrow Phase

- [x] `ContactManifold` struct (up to 4 contact points, normals, penetration depth)
- [x] `ContactPoint` struct (world position, normal, depth, warm-start impulse cache)
- [x] GJK algorithm (convex-convex closest point / intersection)
- [x] EPA algorithm (penetration depth + normal from GJK simplex)
- [x] SAT fast path: sphere-sphere
- [x] SAT fast path: sphere-box
- [x] SAT fast path: box-box (up to 4 contact points via Sutherland-Hodgman)
- [x] Capsule-capsule narrow phase (segment closest-point)
- [x] Dispatch table: `collide(ShapeInstance, ShapeInstance) -> optional<ContactManifold>`
- [x] Persistent contact cache (match new contacts to previous frame, preserve warm-start data)
- [x] Unit tests: known overlapping pairs, verify manifold correctness

---

## Phase 5 — Rigid Body Dynamics

- [x] `BodyData` struct (mass, inverse mass, inertia tensor, inverse inertia, transform, linear/angular velocity, force/torque accumulators, sleep state)
- [x] `BodyPool` (flat contiguous array, ID-based access)
- [x] `Body` handle class (wraps uint32_t ID, provides method interface)
- [x] `BodyType` enum (Static, Kinematic, Dynamic, Sensor)
- [x] Semi-implicit Euler integrator (`v += (F/m + g) * dt`, `x += v * dt`, `ω += I⁻¹ * τ * dt`, `q = normalize(q + 0.5 * Ω * q * dt)`)
- [x] Linear and angular damping
- [x] Sleep system (velocity below threshold for N frames → sleep)
- [x] Force/impulse/torque application methods
- [x] Unit tests: free-fall matches `y = 0.5 * g * t²`, damping reduces energy

---

## Phase 6 — Constraint Solver

- [x] `Constraint` abstract base with two body handles and `solve(dt)` interface
- [x] Sequential Impulse (SI) solver loop (N iterations, configurable)
- [x] Warm starting (restore previous impulses at start of each step)
- [x] Baumgarte position correction (configurable bias factor + slop)
- [x] Split-impulse position solve (post-integration direct position/orientation correction; `positionIterations`, `positionCorrectionAlpha`)
- [x] Stress / torture tests: box tower, sphere tower, mass-ratio extremes, deep penetration, fast-moving objects, long constraint chains, high angular velocity, random impulses, constraint drift measurement, kinematic push (15 tests in `test_stress.cpp`)
- [x] `FixedConstraint` (locks all 6 DOF)
- [x] `BallSocketConstraint` (3 rotational DOF free, 3 translational locked)
- [x] `HingeConstraint` (1 rotational DOF free, limits, motor with target velocity + max force)
- [x] `SliderConstraint` (1 translational DOF free, limits, motor)
- [x] `DistanceConstraint` (min/max distance between anchor points)
- [x] Unit tests: two bodies linked by FixedConstraint stay together, Hinge swings within limits

---

## Phase 7 — PhysicsWorld & Pipeline

- [x] `PhysicsWorld` class: owns `BodyPool`, `BroadPhase`, `NarrowPhase`, `ConstraintSolver`
- [x] `PhysicsWorld::createBody(BodyDescriptor) -> Body`
- [x] `PhysicsWorld::destroyBody(Body)`
- [x] `PhysicsWorld::setGravity(Vector3)`
- [x] `PhysicsWorld::setSubsteps(int)` (default 1)
- [x] `PhysicsWorld::setFixedTimestep(float)` (default 1/60)
- [x] `PhysicsWorld::step(float deltaTime)` — fixed-timestep loop with remainder accumulation
- [x] Thread-safe `BodyInterface` (lock-free reads, guarded writes for add/remove) — `createBody`/`destroyBody`/mutations take exclusive lock; reads (`getTransform`, `getLinearVelocity`, …) are lock-free; 18 tests including concurrent create, concurrent mutate, step-vs-create, step-vs-mutate-vs-destroy
- [x] Sensor/trigger handling (collide in narrowphase but skip impulse response)
- [x] Integration test: sphere dropped from height, bounces on static plane, contact events fire

---

## Phase 8 — Query System

- [x] `QueryFilter` struct (`uint32_t layer`, `uint32_t mask`)
- [x] `RaycastHit` struct (body, fraction, point, normal)
- [x] `ShapeCastHit` struct (body, fraction, point, normal)
- [x] `OverlapResult` struct (body)
- [x] `PhysicsWorld::raycastClosest(Ray, QueryFilter) -> std::optional<RaycastHit>`
- [x] `PhysicsWorld::raycastAll(Ray, QueryFilter) -> std::vector<RaycastHit>`
- [x] `PhysicsWorld::shapeCast(Shape&, Transform, Vector3 dir, float maxDist, QueryFilter) -> std::optional<ShapeCastHit>`
- [x] `PhysicsWorld::overlap(Shape&, Transform, QueryFilter) -> std::vector<OverlapResult>`
- [x] Unit tests: ray hits sphere/box at expected position/normal; overlap returns correct bodies; shape cast finds body in path

---

## Phase 9 — Events & Callbacks

- [x] `IContactListener` interface (`onContactAdded`, `onContactPersisted`, `onContactRemoved`)
- [x] `ITriggerListener` interface (`onTriggerEnter`, `onTriggerExit`)
- [x] `IStepListener` interface (`onPreStep(float dt)`, `onPostStep(float dt)`)
- [x] `PhysicsWorld::addContactListener(IContactListener*)` / `removeContactListener`
- [x] `PhysicsWorld::addTriggerListener(ITriggerListener*)` / `removeTriggerListener`
- [x] `PhysicsWorld::addStepListener(IStepListener*)` / `removeStepListener`
- [x] Dispatch contact events after narrowphase each substep
- [x] Dispatch trigger events when sensor pairs appear/disappear
- [x] Unit tests: step listener dt, pre/post order, removal; contact added/persisted/removed lifecycle, multi-listener, manifold data; trigger enter/exit/removal

---

## Phase 10 — Platform & Performance

- [x] SIMD integration: use `Matrix4f` (vector_math) in broadphase AABB transforms and integrator
- [x] Multi-threaded broadphase AABB pre-compute (parallel body AABB computation, serial BVH update)
- [x] Multi-threaded narrowphase (parallel pair evaluation via std::thread, threshold 32 pairs)
- [x] Solver remains single-threaded (SI is sequential by nature)
- [x] Mobile validation: compile and link on iOS (Apple Silicon) and Android (AArch64)
- [x] Benchmark: 1000 dynamic bodies simulate at >60Hz on target desktop hardware
  - FreeFall/1000: ~1092Hz | Pile/1000: ~461Hz | PileSteadyState/1000: ~506Hz
  - ConstraintChains/100 (1100 bodies): ~241Hz | MixedScene/1000: ~226Hz
  - Expanded suite: `BM_FreeFall`, `BM_Pile`, `BM_PileSteadyState`, `BM_ConstraintChains`, `BM_MixedScene`, `BM_PileParallel`
  - Hz counter reports `total_steps / wall_time` for direct simulation throughput comparison

---

## Phase 11 — Continuous Collision Detection (CCD)

- [x] Per-body `ccdEnabled` flag (BodyDescriptor + BodyData)
- [x] Speculative contacts via swept-AABB broadphase query + conservative step march + binary-search TOI
- [x] Integrate CCD contacts into the SI solver as velocity constraints with speculative bias (no Baumgarte, no warm start)
- [x] Sensor bodies correctly ignored by CCD
- [x] Dynamic-dynamic CCD pairs deduplicated (lower id is canonical)
- [x] Unit tests: tunneling demonstrated without CCD, prevented with CCD; slow-body fall, separating body, opposite approach, sensor passthrough (6 tests)

---

## Phase 12 — Advanced Features

- [x] **Buoyancy:** `BuoyancyVolume` — Archimedes force (spherical cap formula for spheres, AABB fallback for others) + viscous drag; applied per substep before broadphase (6 tests)
- [x] **Articulated bodies:** `ArticulatedBodyDescriptor` tree of links with `BallSocket`, `Hinge` (limits + motor), and `Fixed` joint types; `createArticulatedBody` / `destroyArticulatedBody` (6 tests)
- [x] **Ragdoll helper:** 15-bone humanoid template (`RagdollBone::Pelvis`…`LowerLegR`); volume-proportional mass; capsule shapes; ball-socket spine/shoulders/hips; hinge elbows/knees with limits (6 tests)
- [x] **Vehicle physics:** raycast-based wheels (no separate wheel bodies); spring-damper suspension; engine drive, braking, lateral friction; `createVehicle` / `destroyVehicle` / `syncVehicleControls` / `vehicleWheelState` (6 tests)
- [x] `D6Constraint` (per-axis limits and drives for all 6 DOF) — deferred from Phase 6

---

## Phase 13 — Debug, Tooling & Docs

- [x] `IDebugDraw` interface (`drawLine`, `drawBox`, `drawSphere`, `drawCapsule`, `drawAABB`, `drawArrow`, `drawPoint`) with default implementations composed from `drawLine`
- [x] `PhysicsWorld::debugDraw(IDebugDraw&, DebugDrawFlags)` — renders bodies, AABBs, contacts, constraints, velocity arrows (19 tests)
- [x] `DebugDrawFlags` bitmask (`BodyShapes`, `BodyAABBs`, `ContactPoints`, `Constraints`, `SleepState`, `Velocities`, `All`)
- [x] Profiling hooks (`IPhysicsProfiler` with `beginScope`/`endScope`, `ProfileScope` RAII guard, `CAMPELLO_PROFILE_SCOPE` macro)
- [x] Profiler scope calls wired into `substep` pipeline (BroadPhase, NarrowPhase, BuildContacts, Solver, Integrate, Buoyancy, Vehicles)
- [x] World state serialization: `PhysicsWorld::serialize() -> std::string` (JSON, hand-rolled writer, 17 tests)
- [x] World state deserialization: `PhysicsWorld::deserialize(std::string_view)` (recursive-descent parser, all shapes, full body state)
- [x] Doxygen configuration (`docs/Doxyfile`)
- [x] Per-platform build guide (`docs/building.md`)
- [x] Example: `examples/falling_sphere/`
- [x] Example: `examples/character_controller/`
- [x] Example: `examples/vehicle/`
- [x] Example: `examples/ragdoll/`

---

## Phase 15 — Island Detection & Coordinated Sleep

- [x] Union-Find (disjoint sets, path-halving + union-by-rank) for grouping connected components
- [x] `buildIslands()`: one island per connected component of the contact/constraint graph; dynamic/kinematic bodies only in UF; static bodies serve as anchors but are not in any island
- [x] Contact-based edges: every dynamic-dynamic contact pair unions its two bodies
- [x] Constraint-based edges: every constraint between two dynamic/kinematic bodies unions them
- [x] Wake propagation: if any body in an island is awake, all sleeping bodies in that island are immediately woken
- [x] Island-level sleep: `updateIslandSleep()` puts ALL bodies in an island to sleep simultaneously, only when every dynamic body in the island has `sleepFrames >= required`
- [x] `solveIslands()`: sleeping islands are entirely skipped by the constraint + contact solver; warm-start also skips fully-sleeping pairs
- [x] Integrator: accumulates `sleepFrames` but delegates final sleep decision to island manager
- [x] 8 unit tests: isolated body, independent islands, wake-via-contact, connected bodies sleep together, constraint merges islands, sleeping island frozen by gravity, stack wakes as one unit, sleepFrames reset on wake

---

## Phase 14 — GPU Compute Acceleration

> Requires integration with `campello_gpu` (WebGPU-modeled abstraction layer).
> Professional engines (PhysX 5, Chaos) use compute shaders at scale (>1 000–10 000 dynamic bodies).
> GPU broadphase + XPBD/PBD solvers are embarrassingly parallel; GJK/SI solver are not suitable.

### Layer A — Structure (complete)
- [x] `PhysicsBackend` enum `{Cpu, Gpu, Auto}` added to `PhysicsWorld` public API
- [x] `world.setBackend(PhysicsBackend)` — selects backend; falls back to Cpu if GPU init fails
- [x] `world.setGpuBodyThreshold(int)` — Auto switches at this body count
- [x] `campello_gpu` v0.13.0 FetchContent integration (conditional on `CAMPELLO_PHYSICS_GPU=ON`)
- [x] `IGpuBackend` interface (`src/gpu/gpu_backend.h`)
- [x] `GpuBodyState` / `GpuBodyShape` / `GpuShaderParams` / `GpuPair` GPU buffer structs
- [x] `GpuPhysicsBackend` skeleton (`src/gpu/gpu_physics_backend.h/.cpp`)

### Layer B — GPU Pipeline (complete)
- [x] `cmake/CompileShaders.cmake` — platform-conditional shader compilation
  - macOS/iOS: `xcrun metal + metallib` → `.metallib`
  - Linux/Android: `glslc` → `.spv`
  - Windows: `fxc` → `.cso`
- [x] Five GPU kernels, three platforms (15 shader files):
  - `broadphase_aabb` — parallel world AABB update from transform + shape params
  - `broadphase_pairs` — O(N²/2) AABB overlap detection with atomic pair counter
  - `xpbd_predict` — semi-implicit Euler position/rotation prediction
  - `xpbd_contacts` — Jacobi XPBD contact solve (sphere-radius approximation, 10 iterations, 0.4× scale)
  - `xpbd_finalize` — commit predicted positions, derive new velocities
- [x] CPU↔GPU sync: upload at step start, download after finalize, `waitForIdle` once per step

### Algorithm notes
- CPU path: Sequential Impulse (SI) — unchanged
- GPU path: XPBD (different algorithm, different behavior) — documented difference
- Auto mode decides once per `step()` call; no mid-step switching
- GPU broadphase is O(N²) — suitable for ≤ ~2 000 bodies; sort+sweep deferred
- Contact events not dispatched in GPU path (no manifold data on CPU)

### Deferred
- [ ] **GPU particle system:** position-based particle simulation (cloth, fluid)
- [ ] **GPU soft bodies:** tetrahedral mesh XPBD
- [ ] Benchmark: CPU vs GPU at 1k / 10k / 100k dynamic bodies
- [ ] GPU broadphase sort+sweep (replaces O(N²) naive pair detection)
