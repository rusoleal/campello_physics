#include <campello_physics/allocator.h>

#include <cassert>
#include <cstdlib>
#include <new>

namespace campello::physics {

static constexpr std::size_t kMinBlockSize = sizeof(void*);

PoolAllocator::PoolAllocator(std::size_t blockSize, std::size_t blockCount)
    : m_blockSize(blockSize < kMinBlockSize ? kMinBlockSize : blockSize)
    , m_blockCount(blockCount)
{
    m_memory = static_cast<std::uint8_t*>(std::malloc(m_blockSize * m_blockCount));
    if (!m_memory)
        throw std::bad_alloc{};

    // Build free list — each free block stores a pointer to the next free block.
    m_freeHead = m_memory;
    for (std::size_t i = 0; i < m_blockCount - 1; ++i) {
        void* current = m_memory + i * m_blockSize;
        void* next    = m_memory + (i + 1) * m_blockSize;
        *reinterpret_cast<void**>(current) = next;
    }
    *reinterpret_cast<void**>(m_memory + (m_blockCount - 1) * m_blockSize) = nullptr;
}

PoolAllocator::~PoolAllocator() {
    std::free(m_memory);
}

void* PoolAllocator::allocate() noexcept {
    if (!m_freeHead)
        return nullptr;

    void* block   = m_freeHead;
    m_freeHead    = *reinterpret_cast<void**>(block);
    ++m_used;
    return block;
}

void PoolAllocator::deallocate(void* ptr) noexcept {
    assert(ptr != nullptr);
    assert(m_used > 0);

    *reinterpret_cast<void**>(ptr) = m_freeHead;
    m_freeHead = ptr;
    --m_used;
}

} // namespace campello::physics
