# Cross-engine benchmark: campello_physics vs Jolt Physics
# Requires CAMPELLO_PHYSICS_BUILD_BENCHMARKS=ON

include(FetchContent)
FetchContent_Declare(
    joltphysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG        v5.2.0
    SOURCE_SUBDIR  Build
)

# Jolt build options — minimal, disable what we don't need
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(TARGET_TEST_FRAMEWORK OFF CACHE BOOL "" FORCE)
set(CROSS_PLATFORM_DETERMINISTIC OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(joltphysics)

# Find Bullet (installed via brew on macOS, or system package elsewhere)
find_path(BULLET_INCLUDE_DIR btBulletDynamicsCommon.h
    PATHS /usr/local/include/bullet /opt/homebrew/include/bullet /usr/include/bullet
    PATH_SUFFIXES bullet
)
find_library(BULLET_DYNAMICS_LIB BulletDynamics
    PATHS /usr/local/lib /opt/homebrew/lib /usr/lib
)
find_library(BULLET_COLLISION_LIB BulletCollision
    PATHS /usr/local/lib /opt/homebrew/lib /usr/lib
)
find_library(BULLET_LINEAR_MATH_LIB LinearMath
    PATHS /usr/local/lib /opt/homebrew/lib /usr/lib
)
if(BULLET_INCLUDE_DIR AND BULLET_DYNAMICS_LIB AND BULLET_COLLISION_LIB AND BULLET_LINEAR_MATH_LIB)
    set(BULLET_FOUND TRUE)
    set(BULLET_INCLUDE_DIRS ${BULLET_INCLUDE_DIR})
    set(BULLET_LIBRARIES ${BULLET_DYNAMICS_LIB} ${BULLET_COLLISION_LIB} ${BULLET_LINEAR_MATH_LIB})
endif()

set(BULLET_SRC bullet_adapter.cpp)
set(BULLET_LINK)
set(BULLET_INCLUDE)
if(BULLET_FOUND)
    message(STATUS "Bullet found: ${BULLET_INCLUDE_DIRS}")
    list(APPEND BULLET_LINK ${BULLET_LIBRARIES})
    list(APPEND BULLET_INCLUDE ${BULLET_INCLUDE_DIRS})
else()
    message(WARNING "Bullet not found — Bullet benchmarks will be disabled")
    set(BULLET_SRC)
endif()

add_executable(campello_physics_cross_engine_bench
    main.cpp
    campello_adapter.cpp
    jolt_adapter.cpp
    ${BULLET_SRC}
)

if(BULLET_FOUND)
    target_compile_definitions(campello_physics_cross_engine_bench PRIVATE BULLET_FOUND)
endif()

target_link_libraries(campello_physics_cross_engine_bench
    PRIVATE
        campello_physics
        Jolt
        benchmark::benchmark
        ${BULLET_LINK}
)

target_include_directories(campello_physics_cross_engine_bench
    PRIVATE
        ${joltphysics_SOURCE_DIR}
        ${BULLET_INCLUDE}
)
