# CLAUDE.md — campello_physics

## Project Overview

`campello_physics` is a from-scratch C++20 3D rigid-body physics engine. It is a standalone library in the [campello](https://github.com/rusoleal/campello) ecosystem. No third-party physics engine is used as a backend.

**Namespace:** `campello::physics`
**C++ standard:** C++20
**Build system:** CMake 3.22+

---

## Directory Structure

```
campello_physics/
├── include/
│   └── campello_physics/      # Public API headers (installed with the library)
│       ├── physics_world.h
│       ├── body.h
│       ├── shapes.h
│       ├── material.h
│       ├── constraints.h
│       ├── query.h
│       └── listeners.h
├── src/                       # Implementation files (not installed)
│   ├── broadphase/
│   ├── narrowphase/
│   ├── dynamics/
│   ├── constraints/
│   └── query/
├── tests/                     # GoogleTest unit and integration tests
├── examples/                  # Standalone usage examples
├── docs/                      # Doxygen configuration and extra docs
├── cmake/                     # CMake helper modules (platform toolchains, etc.)
├── CMakeLists.txt
├── CLAUDE.md
├── agents.md
├── todo.md
└── README.md
```

---

## Math Library

Math types come from **vector_math** (`github.com/rusoleal/vector_math`), fetched via CMake FetchContent.

**Namespace:** `systems::leal::vector_math`

| Type | Use in physics |
|---|---|
| `Vector3` | positions, normals, forces, velocities |
| `Vector4` | planes, homogeneous coordinates |
| `Quaternion<float>` | body rotations |
| `Matrix3` | inertia tensors |
| `Matrix4` / `Matrix4f` | world transforms (Matrix4f is SIMD-optimized) |

Always prefer `Matrix4f` in hot paths (broadphase, integration). Import with:

```cpp
#include <vector_math/vector_math.h>
using vm = systems::leal::vector_math;
```

---

## Key Public API Types

```cpp
namespace campello::physics {

enum class BodyType { Static, Kinematic, Dynamic, Sensor };

struct BodyDescriptor { ... };  // input to PhysicsWorld::createBody()
class Body { ... };             // thin copyable handle
class Shape { ... };            // ref-counted, shareable
class Material { ... };
class Constraint { ... };
struct QueryFilter { uint32_t layer; uint32_t mask; };
struct RaycastHit { ... };
struct ShapeCastHit { ... };
struct OverlapResult { ... };

struct IContactListener { virtual void onContactAdded(...); ... };
struct ITriggerListener { virtual void onTriggerEnter(...); ... };
struct IStepListener    { virtual void onPreStep(float dt); ... };

class PhysicsWorld { ... };     // simulation root, factory for all objects

} // namespace campello::physics
```

---

## Build Commands

### Configure and build (desktop)
```bash
cmake -B build -S .
cmake --build build
```

### Build with tests
```bash
cmake -B build -S . -DCAMPELLO_PHYSICS_BUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure
```

### Build with examples
```bash
cmake -B build -S . -DCAMPELLO_PHYSICS_BUILD_EXAMPLES=ON
cmake --build build
```

### Release build
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

---

## Coding Conventions

- **C++20 features encouraged:** concepts, ranges, `std::span`, `std::optional`, `[[nodiscard]]`, designated initializers.
- **No raw owning pointers** in public API — use `std::shared_ptr` for shapes, `std::unique_ptr` for world-owned objects.
- **Body** is a thin copyable handle (wraps an integer ID internally). Never store raw pointers to body data.
- **Shapes are immutable after creation.** Create a new shape if geometry needs to change.
- **No exceptions** in the physics core — use `std::optional` / error codes.
- **Headers** in `include/campello_physics/` must be self-contained and include only what they need.
- **No STL containers** in hot-path inner loops — use fixed-size arrays or pool-allocated structures.
- **SIMD paths** use `Matrix4f` from vector_math. All SIMD code must compile cleanly on scalar fallback (ARM 32-bit).
- Naming: `camelCase` for methods and variables, `PascalCase` for types, `SCREAMING_SNAKE` for macros.

---

## Architecture Overview

```
PhysicsWorld::step(dt)
  │
  ├── BroadPhase::update()          — AABB BVH, produces collision pairs
  │
  ├── NarrowPhase::collide(pairs)   — GJK/EPA/SAT, produces contact manifolds
  │
  ├── ConstraintSolver::solve()     — Sequential Impulse, resolves constraints + contacts
  │
  └── Integrator::integrate(dt)     — Semi-implicit Euler, updates positions/velocities
```

Bodies are stored in a flat pool (`BodyPool`). Shapes are ref-counted and may be shared across bodies. Constraints hold two `Body` handles.

The query system (`RaycastClosest`, `RaycastAll`, `ShapeCast`, `Overlap`) operates independently from the simulation pipeline and can be called between steps.

---

## Testing

Tests live in `tests/`. Each phase has its own test file:

```
tests/
├── test_shapes.cpp
├── test_broadphase.cpp
├── test_narrowphase.cpp
├── test_dynamics.cpp
├── test_constraints.cpp
├── test_queries.cpp
├── test_world.cpp          # integration test (sphere + plane)
└── bench_simulation.cpp    # Google Benchmark (1000 boxes)
```

Run all tests:
```bash
cd build && ctest --output-on-failure
```

---

## Dependencies (auto-fetched via FetchContent)

| Library | Purpose | CMake option |
|---|---|---|
| vector_math | Math types and SIMD | always |
| GoogleTest | Unit tests | `CAMPELLO_PHYSICS_BUILD_TESTS` |
| Google Benchmark | Performance benchmarks | `CAMPELLO_PHYSICS_BUILD_BENCHMARKS` |
