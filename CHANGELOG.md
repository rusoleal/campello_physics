# Changelog

All notable changes to `campello_physics` are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [1.2.0] — 2026-04-28

### Added

- **Generation counters on `Body` handles**: `Body` now stores a `generation` counter; `BodyPool` increments it on destroy. Stale handles correctly report `isValid() == false` even after pool-slot reuse. `PhysicsWorld::destroyBody()` and `BodyPool::destroyBody(Body)` are now no-ops on invalid handles.
- **Fuzz / adversarial test suite** (`tests/test_fuzz.cpp`): 20 tests covering zero-mass dynamics, NaN/Inf states, degenerate shapes (zero-radius sphere, zero-extent box), identical overlapping bodies, 1:1e6 mass ratios, negative damping, null shapes, random impulse spam, same-body constraints, extreme restitution/friction, massive deep penetration, extreme solver iterations, rapid create/destroy cycles, sleep/wake churn, and serialization of degenerate worlds.

### Fixed

- **ASan container-overflow** in `PhysicsWorld::buildContactSoA()`: when `m_contactPoints` was empty but constraints still existed (e.g., constraint-chain stress tests), stale `m_globalContactColors` from a previous frame caused out-of-bounds writes. Added an early-return guard when `n == 0`.

### Changed

- **GPU dependency**: `campello_gpu` upgraded from v0.9.0 to v0.13.0.

---

## [1.1.0] — 2026-04-27

### Added

- **WebAssembly (Emscripten) support**: `CAMPELLO_PLATFORM_WASM`, pthreads-enabled builds, SIMD128, CI job via `mymindstorm/setup-emsdk`, `scripts/build_wasm.sh` convenience script

---

## [1.0.0] — 2026-04-26

First stable release. All planned phases complete.

### Added

**CI / Platform (v1.0.0 update)**
- Desktop CI expanded: Linux x86_64 + ARM64 (GCC 13 & Clang 17), macOS x86_64 (macos-13) + ARM64 (macos-15), Windows x86_64 (windows-2022) + ARM64 (windows-11-arm)
- New `ci-android.yml`: all 4 ABIs (armeabi-v7a, arm64-v8a, x86, x86_64) × API levels 21 / 29 / 35
- New `ci-ios.yml`: iOS (OS64, SIMULATOR64, SIMULATORARM64), tvOS, watchOS, visionOS device + simulator targets
- `cmake/ios.toolchain.cmake` expanded from 3 to 9 Apple platform targets (added tvOS, watchOS, visionOS + their simulators)

**Foundation & Primitives (Phase 1)**
- Platform detection macros (`CAMPELLO_PLATFORM_WINDOWS/MACOS/LINUX/IOS/ANDROID`)
- `CAMPELLO_FORCE_INLINE`, `CAMPELLO_NODISCARD`, `CAMPELLO_LIKELY/UNLIKELY`
- Fixed-size memory pool allocator for bodies and shapes
- `AABB`, `Ray`, `Transform`, `Plane` structs

**Collision Shapes (Phase 2)**
- `BoxShape`, `SphereShape`, `CapsuleShape`, `CylinderShape`
- `ConvexHullShape`, `TriangleMeshShape` (static, BVH-accelerated), `HeightFieldShape`
- `CompoundShape` (N sub-shapes with local transforms)
- `computeAABB(Transform)` and `computeLocalInertiaDiagonal(mass)` for all types

**Broad Phase (Phase 3)**
- Dynamic AABB BVH with branch-and-bound sibling selection and fat-AABB amortisation
- Static + dynamic bucket separation; added/removed/persisted pair tracking
- Layer/mask filtering (`(layerA & maskB) == 0` → skip)

**Narrow Phase (Phase 4)**
- GJK closest-point and intersection test
- EPA penetration depth and contact normal
- SAT fast paths: sphere-sphere, sphere-box, box-box (Sutherland-Hodgman clipping, up to 4 contacts)
- Capsule-capsule via segment closest-point
- Persistent contact cache with warm-start impulse carry-over
- `collide(ShapeInstance, ShapeInstance) -> optional<ContactManifold>` dispatch table

**Rigid Body Dynamics (Phase 5)**
- `BodyData`, `BodyPool`, `Body` handle; `BodyType` enum (Static/Kinematic/Dynamic/Sensor)
- Semi-implicit Euler integrator with linear/angular damping
- Sleep/wake threshold system; force, impulse, and torque application

**Constraint Solver (Phase 6)**
- Sequential Impulse solver with warm starting and configurable iteration count
- Baumgarte velocity-level position correction
- Split-impulse post-integration position solve (`positionIterations`, `positionCorrectionAlpha`)
- `FixedConstraint`, `BallSocketConstraint`, `HingeConstraint` (limits + motor)
- `SliderConstraint` (limits + motor), `DistanceConstraint`, `D6Constraint` (6DOF)

**PhysicsWorld & Pipeline (Phase 7)**
- `PhysicsWorld` owning `BodyPool`, `BroadPhase`, `NarrowPhase`, `ConstraintSolver`
- Fixed-timestep loop with remainder accumulation; configurable sub-steps
- Thread-safe `BodyInterface`: exclusive lock for mutations/structural changes, lock-free reads
- Sensor/trigger handling (narrow phase collide, no impulse response)

**Query System (Phase 8)**
- `PhysicsWorld::raycastClosest`, `raycastAll`, `shapeCast`, `overlap`
- `QueryFilter` (layer + mask)

**Events & Callbacks (Phase 9)**
- `IContactListener` (`onContactAdded`, `onContactPersisted`, `onContactRemoved`)
- `ITriggerListener` (`onTriggerEnter`, `onTriggerExit`)
- `IStepListener` (`onPreStep`, `onPostStep`)

**Performance (Phase 10)**
- SIMD via `Matrix4f` (SSE4.2 / NEON) in broadphase AABB transforms and integrator
- Multi-threaded broadphase AABB pre-compute and narrowphase pair evaluation (`std::thread`, threshold 32 pairs)
- Benchmark suite: `BM_FreeFall`, `BM_Pile`, `BM_PileSteadyState`, `BM_ConstraintChains`, `BM_MixedScene`, `BM_PileParallel`
- Reference numbers (Release, 6-core 3GHz): FreeFall/1k ≈ 1092 Hz · Pile/1k ≈ 461 Hz · MixedScene/1k ≈ 226 Hz

**Continuous Collision Detection (Phase 11)**
- Per-body `ccdEnabled` flag
- Speculative contacts via swept-AABB + conservative march + binary-search TOI
- CCD contacts integrated as velocity constraints with speculative bias (no Baumgarte, no warm start)

**Advanced Features (Phase 12)**
- Buoyancy: `BuoyancyVolume` — Archimedes + viscous drag; spherical cap formula for spheres, AABB fallback
- Articulated bodies: `ArticulatedBodyDescriptor` tree with BallSocket, Hinge, Fixed joints
- Ragdoll helper: 15-bone humanoid template with volume-proportional mass and anatomical joint limits
- Vehicle physics: raycast-based wheels, spring-damper suspension, engine/brake/lateral friction

**Debug, Tooling & Docs (Phase 13)**
- `IDebugDraw` interface with default line-composed implementations for all primitive types
- `PhysicsWorld::debugDraw(IDebugDraw&, DebugDrawFlags)` — bodies, AABBs, contacts, constraints, velocities, sleep state
- `IPhysicsProfiler` with `beginScope`/`endScope` and `CAMPELLO_PROFILE_SCOPE` RAII macro; wired into substep pipeline
- World state serialization/deserialization: `serialize() -> std::string` (hand-rolled JSON) and `deserialize(string_view)`
- Doxygen configuration (`docs/Doxyfile`), per-platform build guide (`docs/building.md`)
- Examples: `falling_sphere`, `character_controller`, `vehicle`, `ragdoll`

**Island Detection & Coordinated Sleep (Phase 15)**
- Union-Find (path-halving + union-by-rank) connected-component grouping
- Contact and constraint edges merged into islands each step
- Wake propagation: any awake body wakes its entire island
- Island-level sleep: all bodies in an island sleep simultaneously
- Sleeping islands fully skipped by constraint solver and warm-start

**GPU Compute (Phase 14 — experimental)**
- `PhysicsBackend` enum `{Cpu, Gpu, Auto}`; `world.setBackend()`, `world.setGpuBodyThreshold()`
- `IGpuBackend` interface; `GpuPhysicsBackend` backed by `campello_gpu` v0.13.0
- Five GPU kernels (broadphase AABB update, O(N²) pair detection, XPBD predict/contact-solve/finalize)
- Shader compilation for macOS/iOS (Metal), Linux/Android (SPIR-V), Windows (HLSL)
- Note: GPU path uses XPBD (behavior differs from CPU SI path); contact events not dispatched on GPU; broadphase is O(N²), suitable for ≤ ~2000 bodies

### Known Limitations
- GPU broadphase O(N²) — sort+sweep deferred
- GPU particle and soft-body simulation deferred
- No deterministic cross-platform fixed-point mode
