#pragma once

#include <cstdint>

namespace campello::physics {

// ── IPhysicsProfiler ──────────────────────────────────────────────────────────
//
// Optional profiling interface.  Implement and register via
// PhysicsWorld::setProfiler() to receive named scope timings.
//
// The engine calls beginScope / endScope around every major pipeline stage
// each substep.  Scopes are not nested beyond two levels.

class IPhysicsProfiler {
public:
    virtual ~IPhysicsProfiler() = default;

    // Called at the start of a named timing scope.
    // name is a compile-time string literal — lifetime is static.
    virtual void beginScope(const char* name) = 0;

    // Called at the end of the matching beginScope.
    virtual void endScope(const char* name) = 0;
};

// ── RAII scope guard (internal use) ──────────────────────────────────────────

class ProfileScope {
public:
    ProfileScope(IPhysicsProfiler* p, const char* name) noexcept
        : m_profiler(p), m_name(name) {
        if (m_profiler) m_profiler->beginScope(m_name);
    }
    ~ProfileScope() {
        if (m_profiler) m_profiler->endScope(m_name);
    }
    ProfileScope(const ProfileScope&)            = delete;
    ProfileScope& operator=(const ProfileScope&) = delete;

private:
    IPhysicsProfiler* m_profiler;
    const char*       m_name;
};

} // namespace campello::physics

// Convenience macro — zero-cost when profiler is null.
#define CAMPELLO_PROFILE_SCOPE(profiler, name) \
    ::campello::physics::ProfileScope _prof_scope_##__LINE__((profiler), (name))
