#include "thread_pool.h"
#include <algorithm>

namespace campello::physics {

ThreadPool::ThreadPool(int numThreads) : m_numThreads(numThreads > 1 ? numThreads : 1) {
    for (int i = 1; i < m_numThreads; ++i) {
        m_workers.emplace_back([this] { workerLoop(); });
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock lock(m_mutex);
        m_stop = true;
        ++m_generation;
    }
    m_cvWork.notify_all();
    for (auto& t : m_workers) {
        if (t.joinable()) t.join();
    }
}

void ThreadPool::parallelFor(int total, const std::function<void(int first, int last)>& fn) {
    if (m_numThreads <= 1 || total <= 0) {
        if (total > 0) fn(0, total);
        return;
    }

    // Small chunks improve load balancing when collision-pair cost is uneven.
    const int chunk = std::max(1, total / (m_numThreads * 4));

    {
        std::unique_lock lock(m_mutex);
        ++m_generation;
        m_done = 0;
        m_fn = fn;
        m_total = total;
        m_chunk = chunk;
        m_next.store(0);
    }
    m_cvWork.notify_all();

    // Calling thread participates as a worker.
    int idx;
    while ((idx = m_next.fetch_add(1)) * chunk < total) {
        int first = idx * chunk;
        int last  = std::min(first + chunk, total);
        fn(first, last);
    }

    // Wait for all worker threads to finish.
    {
        std::unique_lock lock(m_mutex);
        m_cvDone.wait(lock, [this] { return m_done == m_numThreads - 1; });
    }
}

void ThreadPool::workerLoop() {
    int myGeneration = -1;
    while (true) {
        std::unique_lock lock(m_mutex);
        m_cvWork.wait(lock, [this, myGeneration] {
            return m_stop || m_generation > myGeneration;
        });
        if (m_stop) return;

        myGeneration = m_generation;
        lock.unlock();

        int idx;
        while ((idx = m_next.fetch_add(1)) * m_chunk < m_total) {
            int first = idx * m_chunk;
            int last  = std::min(first + m_chunk, m_total);
            m_fn(first, last);
        }

        lock.lock();
        ++m_done;
        if (m_done == m_numThreads - 1) {
            m_cvDone.notify_one();
        }
    }
}

} // namespace campello::physics
