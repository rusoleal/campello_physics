#include <benchmark/benchmark.h>
#include "campello_adapter.h"
#include "jolt_adapter.h"
#include <memory>
#include <string>

#ifdef BULLET_FOUND
#include "bullet_adapter.h"
#endif

using FactoryFn = std::unique_ptr<Adapter>(*)();

static void BM_FreeFall(benchmark::State& state, FactoryFn factory, int threads) {
    auto adapter = factory();
    adapter->init(threads);
    const int count = static_cast<int>(state.range(0));
    adapter->setupFreeFall(count);

    const float dt = 1.f / 60.f;
    // Warmup
    for (int i = 0; i < 30; ++i) adapter->step(dt);

    int steps = 0;
    for (auto _ : state) {
        adapter->step(dt);
        ++steps;
    }
    state.counters["Hz"] = benchmark::Counter(
        static_cast<double>(steps), benchmark::Counter::kIsRate);
    state.counters["bodies"] = static_cast<double>(count);
    state.counters["threads"] = static_cast<double>(threads);
}

static void BM_Pile(benchmark::State& state, FactoryFn factory, int threads) {
    auto adapter = factory();
    adapter->init(threads);
    const int count = static_cast<int>(state.range(0));
    adapter->setupPile(count);

    const float dt = 1.f / 60.f;
    for (int i = 0; i < 60; ++i) adapter->step(dt);

    int steps = 0;
    for (auto _ : state) {
        adapter->step(dt);
        ++steps;
    }
    state.counters["Hz"] = benchmark::Counter(
        static_cast<double>(steps), benchmark::Counter::kIsRate);
    state.counters["bodies"] = static_cast<double>(count);
    state.counters["threads"] = static_cast<double>(threads);
}

static void BM_ConstraintChains(benchmark::State& state, FactoryFn factory, int threads) {
    auto adapter = factory();
    adapter->init(threads);
    const int chains = static_cast<int>(state.range(0));
    const int links = 10;
    adapter->setupConstraintChains(chains, links);

    const float dt = 1.f / 60.f;
    for (int i = 0; i < 60; ++i) adapter->step(dt);

    int steps = 0;
    for (auto _ : state) {
        adapter->step(dt);
        ++steps;
    }
    state.counters["Hz"] = benchmark::Counter(
        static_cast<double>(steps), benchmark::Counter::kIsRate);
    state.counters["chains"] = static_cast<double>(chains);
    state.counters["totalBodies"] = static_cast<double>(chains * links);
    state.counters["threads"] = static_cast<double>(threads);
}

static void BM_MixedScene(benchmark::State& state, FactoryFn factory, int threads) {
    auto adapter = factory();
    adapter->init(threads);
    const int count = static_cast<int>(state.range(0));
    adapter->setupMixedScene(count);

    const float dt = 1.f / 60.f;
    for (int i = 0; i < 60; ++i) adapter->step(dt);

    int steps = 0;
    for (auto _ : state) {
        adapter->step(dt);
        ++steps;
    }
    state.counters["Hz"] = benchmark::Counter(
        static_cast<double>(steps), benchmark::Counter::kIsRate);
    state.counters["bodies"] = static_cast<double>(count);
    state.counters["threads"] = static_cast<double>(threads);
}

// ── Register benchmarks ──────────────────────────────────────────────────────

static std::unique_ptr<Adapter> makeCampello() { return std::make_unique<CampelloAdapter>(); }
static std::unique_ptr<Adapter> makeJolt()     { return std::make_unique<JoltAdapter>(); }
#ifdef BULLET_FOUND
static std::unique_ptr<Adapter> makeBullet()   { return std::make_unique<BulletAdapter>(); }
#endif

#ifdef BULLET_FOUND
#define REGISTER_SCENARIO(NAME, FUNC, RANGE, THREADS)                         \
    BENCHMARK_CAPTURE(BM_##NAME, Campello_##THREADS##t, makeCampello, THREADS)\
        ->Arg(RANGE)->MinTime(0.5)->Unit(benchmark::kMillisecond);             \
    BENCHMARK_CAPTURE(BM_##NAME, Jolt_##THREADS##t, makeJolt, THREADS)        \
        ->Arg(RANGE)->MinTime(0.5)->Unit(benchmark::kMillisecond);             \
    BENCHMARK_CAPTURE(BM_##NAME, Bullet_##THREADS##t, makeBullet, THREADS)    \
        ->Arg(RANGE)->MinTime(0.5)->Unit(benchmark::kMillisecond);
#else
#define REGISTER_SCENARIO(NAME, FUNC, RANGE, THREADS)                         \
    BENCHMARK_CAPTURE(BM_##NAME, Campello_##THREADS##t, makeCampello, THREADS)\
        ->Arg(RANGE)->MinTime(0.5)->Unit(benchmark::kMillisecond);             \
    BENCHMARK_CAPTURE(BM_##NAME, Jolt_##THREADS##t, makeJolt, THREADS)        \
        ->Arg(RANGE)->MinTime(0.5)->Unit(benchmark::kMillisecond);
#endif

// Serial benchmarks (1 thread)
REGISTER_SCENARIO(FreeFall,        BM_FreeFall,        1000, 1)
REGISTER_SCENARIO(Pile,            BM_Pile,            1000, 1)
REGISTER_SCENARIO(ConstraintChains,BM_ConstraintChains, 100, 1)
REGISTER_SCENARIO(MixedScene,      BM_MixedScene,      1000, 1)

// Parallel benchmarks (4 threads) — only where both engines support threading
REGISTER_SCENARIO(FreeFall,        BM_FreeFall,        1000, 4)
REGISTER_SCENARIO(Pile,            BM_Pile,            1000, 4)
REGISTER_SCENARIO(ConstraintChains,BM_ConstraintChains, 100, 4)
REGISTER_SCENARIO(MixedScene,      BM_MixedScene,      1000, 4)

BENCHMARK_MAIN();
