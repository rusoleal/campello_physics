#include <benchmark/benchmark.h>
#include <campello_physics/physics_world.h>
#include <campello_physics/shapes/sphere_shape.h>
#include <campello_physics/shapes/box_shape.h>
#include <campello_physics/shapes/capsule_shape.h>
#include <campello_physics/constraints/ball_socket_constraint.h>
#include <cmath>
#include <thread>

using namespace campello::physics;
using V3 = vm::Vector3<float>;

// ── Helpers ───────────────────────────────────────────────────────────────────

static void addFloor(PhysicsWorld& world) {
    BodyDescriptor d;
    d.type  = BodyType::Static;
    d.shape = std::make_shared<BoxShape>(V3(200.f, 0.5f, 200.f));
    d.transform.position = V3(0.f, -0.5f, 0.f);
    world.createBody(d);
}

// Place N spheres in an XZ grid at height y.
static void addSphereGrid(PhysicsWorld& world, int count,
                           float y = 5.f, float spacing = 1.2f) {
    BodyDescriptor d;
    d.type  = BodyType::Dynamic;
    d.mass  = 1.f;
    d.shape = std::make_shared<SphereShape>(0.4f);
    d.linearDamping  = 0.01f;
    d.angularDamping = 0.05f;

    const int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(count))));
    for (int i = 0; i < count; ++i) {
        int ix = i % cols;
        int iz = (i / cols) % cols;
        int iy = i / (cols * cols);
        d.transform.position = V3(
            (float(ix) - cols * 0.5f) * spacing,
            y + float(iy) * spacing,
            (float(iz) - cols * 0.5f) * spacing);
        world.createBody(d);
    }
}

// Warm a world for `warmSteps` before benchmarking.
static void warmWorld(PhysicsWorld& world, int warmSteps) {
    for (int i = 0; i < warmSteps; ++i)
        world.step(1.f / 60.f);
}

// Compute and report Hz from the benchmark state.
static void reportHz(benchmark::State& state, int stepsPerIter) {
    // Total steps across all iterations, divided by total wall-clock time → Hz.
    state.counters["Hz"] = benchmark::Counter(
        double(state.iterations()) * stepsPerIter,
        benchmark::Counter::kIsRate);
}

// ── BM_FreeFall ───────────────────────────────────────────────────────────────
// N bodies in free-fall with no contacts — measures pure integration throughput.

static void BM_FreeFall(benchmark::State& state) {
    const int N            = static_cast<int>(state.range(0));
    const int stepsPerIter = 60;

    for (auto _ : state) {
        PhysicsWorld world;
        world.setGravity(V3(0.f, -9.81f, 0.f));
        addSphereGrid(world, N, /*y=*/5.f, /*spacing=*/5.f);  // wide spacing, no contacts

        for (int i = 0; i < stepsPerIter; ++i)
            world.step(1.f / 60.f);

        benchmark::DoNotOptimize(world);
    }

    state.counters["bodies"] = double(N);
    reportHz(state, stepsPerIter);
}
BENCHMARK(BM_FreeFall)
    ->Arg(100)->Arg(500)->Arg(1000)->Arg(5000)
    ->Unit(benchmark::kMillisecond);

// ── BM_Pile ───────────────────────────────────────────────────────────────────
// N spheres compacted together — many contacts every step.

static void BM_Pile(benchmark::State& state) {
    const int N            = static_cast<int>(state.range(0));
    const int stepsPerIter = 60;

    for (auto _ : state) {
        PhysicsWorld world;
        world.setGravity(V3(0.f, -9.81f, 0.f));
        world.contactIterations    = 10;
        world.constraintIterations = 10;
        addFloor(world);
        addSphereGrid(world, N, /*y=*/1.f, /*spacing=*/1.0f);  // tight grid

        for (int i = 0; i < stepsPerIter; ++i)
            world.step(1.f / 60.f);

        benchmark::DoNotOptimize(world);
    }

    state.counters["bodies"] = double(N);
    reportHz(state, stepsPerIter);
}
BENCHMARK(BM_Pile)
    ->Arg(100)->Arg(500)->Arg(1000)
    ->Unit(benchmark::kMillisecond);

// ── BM_PileSteadyState ────────────────────────────────────────────────────────
// Like BM_Pile but pre-warmed so bodies have settled.  Measures cost of
// maintaining a resting pile (many sleeping islands + waking edge contacts).

static void BM_PileSteadyState(benchmark::State& state) {
    const int N            = static_cast<int>(state.range(0));
    const int warmSteps    = 120;  // 2 s at 60 Hz — enough for pile to settle
    const int stepsPerIter = 60;

    // Build once, warm, then benchmark.
    PhysicsWorld world;
    world.setGravity(V3(0.f, -9.81f, 0.f));
    world.contactIterations    = 10;
    world.constraintIterations = 10;
    addFloor(world);
    addSphereGrid(world, N, /*y=*/1.f, /*spacing=*/1.0f);
    warmWorld(world, warmSteps);

    for (auto _ : state) {
        for (int i = 0; i < stepsPerIter; ++i)
            world.step(1.f / 60.f);
        benchmark::DoNotOptimize(world);
    }

    state.counters["bodies"] = double(N);
    reportHz(state, stepsPerIter);
}
BENCHMARK(BM_PileSteadyState)
    ->Arg(100)->Arg(500)->Arg(1000)
    ->Unit(benchmark::kMillisecond);

// ── BM_ConstraintChains ───────────────────────────────────────────────────────
// N ball-socket chains of length 10 hanging from kinematic anchors.
// Measures solver throughput under dense constraint load.

static void BM_ConstraintChains(benchmark::State& state) {
    const int chainCount   = static_cast<int>(state.range(0));
    const int chainLen     = 10;
    const int stepsPerIter = 60;

    for (auto _ : state) {
        PhysicsWorld world;
        world.setGravity(V3(0.f, -9.81f, 0.f));
        world.constraintIterations = 20;

        BodyDescriptor linkDesc;
        linkDesc.type  = BodyType::Dynamic;
        linkDesc.mass  = 1.f;
        linkDesc.shape = std::make_shared<SphereShape>(0.2f);

        for (int c = 0; c < chainCount; ++c) {
            float cx = float(c % 20) * 2.f;
            float cz = float(c / 20) * 2.f;

            // Kinematic anchor
            BodyDescriptor anchorDesc;
            anchorDesc.type  = BodyType::Kinematic;
            anchorDesc.mass  = 0.f;
            anchorDesc.shape = std::make_shared<SphereShape>(0.1f);
            anchorDesc.transform.position = V3(cx, 0.f, cz);
            Body prev = world.createBody(anchorDesc);

            for (int i = 0; i < chainLen; ++i) {
                linkDesc.transform.position = V3(cx, -(float(i) + 1.f), cz);
                Body curr = world.createBody(linkDesc);
                world.addConstraint(BallSocketConstraint::create(
                    prev, V3(0.f, -0.5f, 0.f),
                    curr, V3(0.f,  0.5f, 0.f)));
                prev = curr;
            }
        }

        for (int i = 0; i < stepsPerIter; ++i)
            world.step(1.f / 60.f);

        benchmark::DoNotOptimize(world);
    }

    state.counters["chains"]      = double(chainCount);
    state.counters["totalBodies"] = double(chainCount * (chainLen + 1));
    reportHz(state, stepsPerIter);
}
BENCHMARK(BM_ConstraintChains)
    ->Arg(10)->Arg(50)->Arg(100)
    ->Unit(benchmark::kMillisecond);

// ── BM_MixedScene ─────────────────────────────────────────────────────────────
// Realistic mixed load: static floor, N dynamic spheres settling on it,
// plus N/10 ball-socket chains of length 5 above the pile.

static void BM_MixedScene(benchmark::State& state) {
    const int N            = static_cast<int>(state.range(0));
    const int stepsPerIter = 60;

    for (auto _ : state) {
        PhysicsWorld world;
        world.setGravity(V3(0.f, -9.81f, 0.f));
        world.contactIterations    = 10;
        world.constraintIterations = 10;

        addFloor(world);
        addSphereGrid(world, N, /*y=*/1.f, /*spacing=*/1.1f);

        // Add N/10 short chains
        const int chains = std::max(1, N / 10);
        BodyDescriptor linkDesc;
        linkDesc.type  = BodyType::Dynamic;
        linkDesc.mass  = 0.5f;
        linkDesc.shape = std::make_shared<CapsuleShape>(0.15f, 0.3f);

        for (int c = 0; c < chains; ++c) {
            float cx = float(c % 10) * 3.f - 15.f;
            float cz = float(c / 10) * 3.f - 15.f;

            BodyDescriptor anchorDesc;
            anchorDesc.type  = BodyType::Kinematic;
            anchorDesc.mass  = 0.f;
            anchorDesc.shape = std::make_shared<SphereShape>(0.1f);
            anchorDesc.transform.position = V3(cx, 8.f, cz);
            Body prev = world.createBody(anchorDesc);

            for (int i = 0; i < 5; ++i) {
                linkDesc.transform.position = V3(cx, 8.f - float(i + 1) * 0.8f, cz);
                Body curr = world.createBody(linkDesc);
                world.addConstraint(BallSocketConstraint::create(
                    prev, V3(0.f, -0.4f, 0.f),
                    curr, V3(0.f,  0.4f, 0.f)));
                prev = curr;
            }
        }

        for (int i = 0; i < stepsPerIter; ++i)
            world.step(1.f / 60.f);

        benchmark::DoNotOptimize(world);
    }

    state.counters["bodies"] = double(N);
    reportHz(state, stepsPerIter);
}
BENCHMARK(BM_MixedScene)
    ->Arg(100)->Arg(500)->Arg(1000)
    ->Unit(benchmark::kMillisecond);

// ── BM_PileParallel ───────────────────────────────────────────────────────────
// Fixed 1000-body pile; varies worker thread count to measure multi-thread scaling.

static void BM_PileParallel(benchmark::State& state) {
    const int workerCount  = static_cast<int>(state.range(0));
    const int N            = 1000;
    const int stepsPerIter = 60;

    for (auto _ : state) {
        PhysicsWorld world;
        world.setWorkerThreads(workerCount);
        world.setGravity(V3(0.f, -9.81f, 0.f));
        world.contactIterations    = 10;
        world.constraintIterations = 10;
        addFloor(world);
        addSphereGrid(world, N, /*y=*/1.f, /*spacing=*/1.0f);

        for (int i = 0; i < stepsPerIter; ++i)
            world.step(1.f / 60.f);

        benchmark::DoNotOptimize(world);
    }

    state.counters["threads"] = double(workerCount);
    state.counters["bodies"]  = double(N);
    reportHz(state, stepsPerIter);
}
BENCHMARK(BM_PileParallel)
    ->Arg(1)
    ->Arg(2)
    ->Arg(4)
    ->Arg(static_cast<int>(std::thread::hardware_concurrency()))
    ->Unit(benchmark::kMillisecond);
