# campello_physics

**Version 1.0.0** — A full-featured, from-scratch, multiplatform **C++20 3D physics engine** — part of the [campello](https://github.com/rusoleal/campello) ecosystem.

---

## Overview

`campello_physics` is a standalone physics simulation library designed to integrate seamlessly with the campello game engine suite. It is built from the ground up in C++20, with no dependency on third-party physics engines, and targets all major desktop and mobile platforms.

The engine is modeled after the best ideas from **Unity (PhysX)**, **Unreal (Chaos)**, **Godot Physics**, **Bullet**, **NVIDIA PhysX**, and **Jolt Physics**, synthesized into a clean, modern C++20 public API.

---

## Features

### Rigid Body Dynamics
- Static, Kinematic, Dynamic, and Sensor body types
- Forces, impulses, torques, and velocity control
- Linear and angular damping
- Island-based sleep/wake system — entire connected component sleeps or wakes together

### Collision Shapes
- **Primitives:** Box, Sphere, Capsule, Cylinder
- **Complex:** ConvexHull, TriangleMesh (static, BVH-accelerated), HeightField (terrain)
- **Composite:** CompoundShape with local sub-shape transforms
- Reference-counted, shareable shapes

### Collision Detection
- Dynamic AABB BVH broad phase with static/dynamic bucket separation and fat-AABB amortisation
- GJK + EPA narrow phase for convex bodies
- SAT fast paths for sphere-sphere, sphere-box, box-box (Sutherland-Hodgman, up to 4 contacts)
- Persistent contact manifolds with warm starting
- Continuous Collision Detection (CCD): speculative contacts + binary-search TOI sweep

### Constraint System
- `FixedConstraint`, `HingeConstraint` (limits + motor), `SliderConstraint` (limits + motor)
- `BallSocketConstraint`, `DistanceConstraint`, `D6Constraint` (full 6DOF)
- Sequential Impulse solver with warm starting, Baumgarte velocity correction, and split-impulse position solve

### Scene Queries
- Raycasts (closest hit and all-hits), shape sweeps (ShapeCast), overlap queries
- Per-query `QueryFilter` with layer/mask filtering

### Events & Callbacks
- `IContactListener` — collision added/persisted/removed
- `ITriggerListener` — sensor enter/exit
- `IStepListener` — pre/post simulation step

### Performance
- SIMD acceleration (SSE4.2 on x86, NEON on ARM/Apple Silicon) via [vector_math](https://github.com/rusoleal/vector_math)
- Multi-threaded broad phase and narrow phase (`std::thread`, configurable worker count)
- Fixed-timestep loop with configurable sub-stepping
- Island detection: sleeping islands are entirely skipped by the solver

**Benchmark results** (Release build, 6-core 3 GHz, macOS):

| Scenario | Bodies | Throughput |
|---|---|---|
| Free-fall (no contacts) | 1 000 | ~1 092 Hz |
| Pile (dense contacts) | 1 000 | ~461 Hz |
| Settled pile (islands sleeping) | 1 000 | ~506 Hz |
| Constraint chains (100 × 10 links) | 1 100 | ~241 Hz |
| Mixed scene (pile + chains) | 1 000 | ~226 Hz |
| Free-fall | 5 000 | ~175 Hz |

### Advanced
- Articulated body chains (ball-socket, hinge, fixed joints)
- Vehicle physics (raycast wheels, spring-damper suspension, engine/brake/lateral friction)
- Buoyancy / fluid volume detection (Archimedes + viscous drag)
- Ragdoll helper API (15-bone humanoid template)
- Debug draw interface (`IDebugDraw`) with `DebugDrawFlags` bitmask
- Profiling hooks (`IPhysicsProfiler`, `CAMPELLO_PROFILE_SCOPE` macro)
- World state serialization/deserialization (JSON)
- Thread-safe `BodyInterface` (lock-free reads, guarded writes)
- GPU compute path via `campello_gpu` — experimental, see [GPU notes](#gpu-compute-experimental)

---

## Platform Support

| Platform | Architecture | Compiler | CI |
|---|---|---|---|
| Linux | x86_64, ARM64 | GCC 13, Clang 17 | `ubuntu-24.04`, `ubuntu-24.04-arm` |
| macOS | x86_64 (Intel), ARM64 (Apple Silicon) | Clang | `macos-13`, `macos-15` |
| Windows | x86_64, ARM64 | MSVC | `windows-2022`, `windows-11-arm` |
| iOS | arm64 device, x86_64 sim, arm64 sim | Clang (Xcode 16) | `macos-15` |
| tvOS | arm64 device, arm64 sim | Clang (Xcode 16) | `macos-15` |
| watchOS | arm64 device, arm64 sim | Clang (Xcode 16) | `macos-15` |
| visionOS | arm64 device, arm64 sim | Clang (Xcode 16) | `macos-15` |
| Android | armeabi-v7a, arm64-v8a, x86, x86_64 | NDK Clang | API 21 / 29 / 35 |

---

## Building

### Requirements
- CMake 3.22+
- C++20-capable compiler (GCC 11+, Clang 13+, MSVC 19.30+)
- Internet connection (CMake FetchContent fetches dependencies automatically)

### Desktop (Linux / macOS / Windows)

```bash
cmake -B build -S .
cmake --build build
```

### Run Tests

```bash
cmake -B build -S . -DCAMPELLO_PHYSICS_BUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure
```

### Run Benchmarks

```bash
cmake -B build -S . -DCAMPELLO_PHYSICS_BUILD_TESTS=ON -DCAMPELLO_PHYSICS_BUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/tests/campello_physics_bench
```

### iOS

```bash
cmake -B build-ios -S . \
  -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake \
  -DPLATFORM=OS64
cmake --build build-ios
```

### Android

```bash
cmake -B build-android -S . \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-24
cmake --build build-android
```

---

## Quick Start

```cpp
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes.h>

using namespace campello::physics;

// Create world
PhysicsWorld world;
world.setGravity({0.f, -9.81f, 0.f});

// Static floor
BodyDescriptor floorDesc;
floorDesc.type      = BodyType::Static;
floorDesc.shape     = std::make_shared<BoxShape>(Vector3{50.f, 0.5f, 50.f});
floorDesc.transform.position = {0.f, -0.5f, 0.f};
Body floor = world.createBody(floorDesc);

// Dynamic sphere
BodyDescriptor sphereDesc;
sphereDesc.type     = BodyType::Dynamic;
sphereDesc.mass     = 1.f;
sphereDesc.shape    = std::make_shared<SphereShape>(1.f);
sphereDesc.transform.position = {0.f, 10.f, 0.f};
Body sphere = world.createBody(sphereDesc);

// Simulate
for (int i = 0; i < 300; ++i)
    world.step(1.f / 60.f);
```

---

## GPU Compute (experimental)

Enable with `-DCAMPELLO_PHYSICS_GPU=ON`. Requires `campello_gpu` v0.9.0 (fetched automatically).

```cpp
world.setBackend(PhysicsBackend::Gpu);   // explicit GPU
world.setBackend(PhysicsBackend::Auto);  // switch at threshold
world.setGpuBodyThreshold(500);
```

The GPU path uses **XPBD** (not SI) and targets `≤ 2 000` dynamic bodies — O(N²) broadphase, sort+sweep is deferred. Contact events are not dispatched in GPU mode. CPU path behavior is unchanged.

---

## Dependencies

| Library | Purpose | Source |
|---|---|---|
| [vector_math](https://github.com/rusoleal/vector_math) | Vec3, Quat, Mat4 (SIMD) | campello ecosystem |
| [GoogleTest](https://github.com/google/googletest) | Unit testing (optional) | FetchContent |
| [Google Benchmark](https://github.com/google/benchmark) | Benchmarks (optional) | FetchContent |
| [campello_gpu](https://github.com/rusoleal/campello_gpu) | GPU compute (optional) | FetchContent |

---

## Campello Ecosystem

`campello_physics` is one module in the campello C++20 game engine collection:

| Module | Description |
|---|---|
| [campello_core](https://github.com/rusoleal/campello) | ECS foundation |
| [campello_renderer](https://github.com/rusoleal/campello) | 3D renderer (glTF, WebGPU-modeled) |
| [campello_physics](https://github.com/rusoleal/campello_physics) | Physics engine (this repo) |
| [campello_audio](https://github.com/rusoleal/campello) | Audio system |
| [campello_input](https://github.com/rusoleal/campello) | Input handling |
| [campello_net](https://github.com/rusoleal/campello) | Networking |
| [vector_math](https://github.com/rusoleal/vector_math) | Math library |

---

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

---

## License

MIT — see [LICENSE](LICENSE).
