#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace campello::physics {

// Lightweight persistent thread pool for embarrassingly-parallel loops.
// Spawns N-1 worker threads on construction and reuses them across calls.
// This eliminates the cost of creating/joining std::thread every substep.
class ThreadPool {
public:
    explicit ThreadPool(int numThreads);
    ~ThreadPool();

    // Disable copy / move (threads are not movable)
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Execute fn(first,last) over [0,total) split into small chunks.
    // Blocks until all workers finish. Safe to call from any thread.
    void parallelFor(int total, const std::function<void(int first, int last)>& fn);

    [[nodiscard]] int numThreads() const noexcept { return m_numThreads; }

private:
    int m_numThreads;
    std::vector<std::thread> m_workers;

    std::mutex              m_mutex;
    std::condition_variable m_cvWork;
    std::condition_variable m_cvDone;

    bool m_stop = false;
    int  m_generation = 0;
    int  m_done = 0;

    std::function<void(int,int)> m_fn;
    int m_total = 0;
    int m_chunk = 0;
    std::atomic<int> m_next{0};

    void workerLoop();
};

} // namespace campello::physics
