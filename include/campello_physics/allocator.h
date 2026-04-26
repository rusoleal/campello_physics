#pragma once

#include <cstddef>
#include <cstdint>

namespace campello::physics {

// Fixed-size block pool allocator.
// Thread-safety is NOT guaranteed — external locking required for concurrent access.
class PoolAllocator {
public:
    PoolAllocator(std::size_t blockSize, std::size_t blockCount);
    ~PoolAllocator();

    PoolAllocator(const PoolAllocator&)            = delete;
    PoolAllocator& operator=(const PoolAllocator&) = delete;
    PoolAllocator(PoolAllocator&&)                 = delete;
    PoolAllocator& operator=(PoolAllocator&&)      = delete;

    // Returns a pointer to a free block, or nullptr if the pool is exhausted.
    [[nodiscard]] void* allocate() noexcept;

    // Returns a block to the pool. ptr must have been returned by allocate().
    void deallocate(void* ptr) noexcept;

    [[nodiscard]] std::size_t capacity() const noexcept { return m_blockCount; }
    [[nodiscard]] std::size_t used()     const noexcept { return m_used; }
    [[nodiscard]] bool        full()     const noexcept { return m_used == m_blockCount; }

private:
    std::uint8_t* m_memory    = nullptr;
    void*         m_freeHead  = nullptr;
    std::size_t   m_blockSize = 0;
    std::size_t   m_blockCount= 0;
    std::size_t   m_used      = 0;
};

} // namespace campello::physics
