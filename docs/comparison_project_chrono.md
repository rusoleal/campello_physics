# campello_physics vs Project Chrono

> Reference document for future feature planning.
> Project Chrono: https://projectchrono.org — BSD-3, University of Wisconsin-Madison + University of Parma.

---

## Purpose & Scope

| | campello_physics | Project Chrono |
|---|---|---|
| **Target** | Game / real-time engine | Scientific / engineering simulation |
| **Primary use** | Games, interactive apps | Research, robotics, vehicle engineering, academia |
| **Origin** | Commercial product | Academic (UW-Madison + Parma) |

This is the most fundamental difference. Chrono optimizes for **accuracy and generality**; campello_physics optimizes for **real-time performance and game integration**.

---

## Physics Capabilities

| | campello_physics | Project Chrono |
|---|---|---|
| Rigid body dynamics | ✓ | ✓ |
| Soft bodies / FEM | ✗ (Phase 12, deferred) | ✓ beams, shells, tetrahedra, hexahedra |
| Fluid (SPH) | ✗ | ✓ Chrono::FSI |
| Vehicle dynamics | ✗ (Phase 12, deferred) | ✓ Chrono::Vehicle — full suspension + drivetrain models |
| Granular / DEM | ✗ | ✓ GPU-accelerated DEM-Engine (CUDA) |
| Cloth / cables | ✗ | ✓ |
| Buoyancy | ✗ (Phase 12, deferred) | ✓ |
| Ragdoll helper | ✗ (Phase 12, deferred) | ✗ (manual constraint graphs) |

---

## Solver Architecture

| | campello_physics | Project Chrono |
|---|---|---|
| CPU solver | Sequential Impulse (SI), warm-started | Complementarity (DVI/DEM), iterative APGD, Minres, Barzilai-Borwein |
| GPU solver | XPBD — broadphase + contact (campello_gpu) | External DEM-Engine (CUDA only) |
| Constraint handling | SI + Baumgarte position correction | Lagrangian multipliers, complementarity |
| Timestep | Fixed + accumulator, sub-stepping | Variable; many integrators: Euler, HHT, Newmark, Runge-Kutta |
| Friction model | Coulomb approximation (SI) | Proper cone friction via complementarity |

**Note:** Chrono's complementarity solvers handle friction and contact as a proper optimization problem (more accurate). SI is an approximation but runs an order of magnitude faster per step — standard in all game engines for this reason.

---

## Collision Detection

| | campello_physics | Project Chrono |
|---|---|---|
| Broadphase | Dynamic AABB BVH | AABBs, sweep-and-prune, multicore variants |
| Narrowphase | GJK+EPA, SAT fast paths (sphere-sphere, sphere-box, box-box) | GJK+EPA via Bullet/custom, analytical primitives |
| Shapes | Box, Sphere, Capsule, Cylinder, ConvexHull, TriangleMesh, HeightField, Compound | Same + FEM surfaces, analytical geometry |
| CCD | ✓ speculative contacts (per-body opt-in) | ✓ |
| Persistent contact cache | ✓ warm-start impulses | ✓ |

Roughly equivalent at the collision detection level for standard rigid body shapes.

---

## GPU

| | campello_physics | Project Chrono |
|---|---|---|
| GPU backend | campello_gpu (Metal / Vulkan / D3D12) | CUDA only (DEM-Engine) |
| GPU solver algorithm | XPBD broadphase + contact solve | DEM particle simulation |
| Shared device with renderer | ✓ (`setBackend(Gpu, device)`) | ✗ CUDA is separate from graphics |
| Render buffer API | ✓ `gpuRenderBuffer()` — pool-slot transform buffer | ✗ |
| Cross-platform GPU | ✓ macOS / Linux / Windows | ✗ NVIDIA only |

campello_physics is ahead for game use: cross-platform GPU with direct render buffer sharing. Chrono's GPU targets scientific particle simulation, not game rendering pipelines.

---

## Integration & API

| | campello_physics | Project Chrono |
|---|---|---|
| Language | C++20 | C++17, Python (PyChrono) |
| Math library | vector_math (campello ecosystem) | ChVector, ChQuaternion (own) |
| Build system | CMake 3.22+ + FetchContent | CMake |
| Python bindings | ✗ | ✓ PyChrono (Anaconda) |
| ROS integration | ✗ | ✓ Chrono::ROS |
| CAD import | ✗ | ✓ SolidWorks, Blender plugins |
| Sensor simulation | ✗ | ✓ Chrono::Sensor (lidar, camera, IMU) |
| Co-simulation (CFD/FEA) | ✗ | ✓ |
| Event system | ✓ contact / trigger / step listeners | Limited |
| Query system | ✓ raycast, shapecast, overlap | Partial |
| Debug draw interface | ✗ (Phase 13, deferred) | ✓ |
| Serialization | ✗ (Phase 13, deferred) | ✓ |

---

## Real-time Suitability

| | campello_physics | Project Chrono |
|---|---|---|
| Fixed timestep loop | ✓ | Optional |
| Sleep system | ✓ | ✓ |
| Designed for 60Hz | ✓ benchmarked (>60Hz at 1k bodies) | ✗ simulation speed varies by problem |
| Multithreaded narrowphase | ✓ (std::thread, threshold 32 pairs) | ✓ Chrono::Multicore |
| Memory model | Pool allocator, contiguous arrays | General heap allocation |
| Mobile (iOS / Android) | ✓ | ✗ |

---

## Features Worth Adopting from Chrono

Roughly in priority order for a game engine context:

### Near-term (Phase 12 overlap)
- **Vehicle physics** — Chrono::Vehicle has a mature wheel/suspension/drivetrain model with well-tested tire models (Pacejka, Fiala). Worth studying their `ChTire` / `ChSuspension` abstraction.
- **Articulated bodies** — Chrono handles reduced-coordinate chains well. Reference for Phase 12 articulated body work.

### Medium-term
- **Soft bodies / FEM** — Chrono's FEM module (beams, cables, shells) covers cloth, ropes, and deformable objects. XPBD-based cloth is the game-friendly variant.
- **Buoyancy volumes** — Chrono computes submerged volume analytically; worth referencing their approach.

### Long-term / research
- **Complementarity solver (DVI)** — For simulations requiring high accuracy (engineering, robotics), a complementarity solver as an alternative to SI would be valuable. Chrono's APGD solver is a good reference.
- **SPH fluid simulation** — Chrono::FSI uses SPH + TDPF; a GPU SPH module would be a natural extension of the XPBD GPU path.
- **Granular / DEM** — Chrono's DEM-Engine (GPU) handles millions of particles. Long-term candidate if campello_gpu is extended.
- **Sensor simulation** — Chrono::Sensor models physical sensors (lidar point clouds, camera, IMU). Relevant if campello becomes a full simulation platform.
- **Distributed / multi-agent** — SynChrono handles large-scale multi-agent scenarios over MPI. Out of scope for games but relevant for simulation infrastructure.

---

## Summary

Chrono is a **scientific simulation platform** covering a vastly broader physics domain (FEM, fluids, vehicles, granular) with more rigorous solvers. campello_physics is a **game physics engine** — faster per step, fixed-timestep-first, with a clean game-oriented API and cross-platform GPU rendering integration.

They are not competing for the same use case. The features most worth adopting from Chrono into campello_physics are the ones that have a game-friendly real-time variant: vehicle physics, soft bodies via XPBD, and articulated body chains.
