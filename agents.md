# agents.md — campello_physics

This file describes how AI agents should approach work in this repository.

---

## Project Summary

`campello_physics` is a from-scratch C++20 3D physics engine (no third-party physics backend). Math comes from `vector_math` (same ecosystem). The project is built in phases — see `todo.md` for the current state.

Always read `CLAUDE.md` before touching code. It defines namespaces, directory layout, coding conventions, and architecture.

---

## Agent Roles

### Implementer
Writes new production code in `src/` and corresponding headers in `include/campello_physics/`.

**Guidelines:**
- Work one phase at a time. Verify the phase is marked as started in `todo.md` before writing code.
- Write tests in `tests/` alongside every new class.
- Follow coding conventions in `CLAUDE.md` strictly — no raw owning pointers, no exceptions, SIMD-aware paths.
- Keep `include/campello_physics/` headers minimal and self-contained.
- After implementing a phase, confirm all tests pass with `ctest`.

### Reviewer
Reviews code for correctness, performance, and convention compliance.

**Focus areas:**
- Numerical stability in the constraint solver and integrator.
- Memory layout — prefer flat pools over pointer-heavy structures in hot paths.
- SIMD correctness — verify scalar fallback compiles and runs on ARM 32-bit.
- API surface — public headers must not leak internal types or implementation details.
- Test coverage — every public method should have at least one test.

### Test Writer
Writes and maintains tests in `tests/`.

**Guidelines:**
- Unit tests: one file per subsystem (broadphase, narrowphase, dynamics, constraints, queries).
- Integration tests: `test_world.cpp` covers end-to-end scenarios.
- Benchmark: `bench_simulation.cpp` covers performance regressions.
- Use GoogleTest (`TEST`, `EXPECT_*`, `ASSERT_*`).
- Use Google Benchmark for `bench_*.cpp` files.
- Analytical ground truth where possible (e.g., free-fall: `y = 0.5 * g * t²`).

### Architect
Plans new phases, proposes API changes, evaluates tradeoffs.

**Guidelines:**
- Consult `todo.md` for phase order. Do not skip phases — each phase's output is input to the next.
- Proposed API changes must not break the conventions in `CLAUDE.md`.
- Any change to the public API in `include/campello_physics/` must update the Quick Start example in `README.md`.

---

## Phase Breakdown

Agents should tackle one phase at a time in order:

| Phase | Focus | Key files |
|---|---|---|
| 0 | Scaffolding | `CMakeLists.txt`, `.gitignore`, `cmake/`, CI workflows |
| 1 | Foundation | `src/foundation/`, platform macros, allocators, AABB, Ray, Transform |
| 2 | Shapes | `src/shapes/`, `include/campello_physics/shapes.h` |
| 3 | Broad Phase | `src/broadphase/` |
| 4 | Narrow Phase | `src/narrowphase/` |
| 5 | Dynamics | `src/dynamics/` |
| 6 | Constraints | `src/constraints/` |
| 7 | PhysicsWorld | `src/world/`, `include/campello_physics/physics_world.h` |
| 8 | Queries | `src/query/`, `include/campello_physics/query.h` |
| 9 | Events | `src/events/`, `include/campello_physics/listeners.h` |
| 10 | Platform/Perf | SIMD paths, threading |
| 11 | CCD | `src/narrowphase/ccd/` |
| 12 | Advanced | `src/advanced/` (vehicles, articulated, buoyancy) |
| 13 | Debug/Docs | `include/campello_physics/debug.h`, Doxygen, examples |

---

## How to Start a Task

1. Read `todo.md` — identify the next unchecked item in the current phase.
2. Read `CLAUDE.md` — confirm conventions.
3. Read all relevant headers in `include/campello_physics/` that touch your work.
4. Write the implementation in `src/`, update or create the header.
5. Write or update tests in `tests/`.
6. Build and run tests: `cmake --build build && cd build && ctest`.
7. Mark the item complete in `todo.md`.

---

## What NOT to Do

- Do not introduce any third-party physics engine (Bullet, Jolt, PhysX, etc.) as a dependency.
- Do not use exceptions anywhere in `src/` or `include/campello_physics/`.
- Do not put implementation logic in public headers (no big inline bodies).
- Do not skip the test step.
- Do not modify `vector_math` — it is a read-only dependency fetched by CMake.
