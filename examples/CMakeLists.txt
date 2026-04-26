
# ── campello_physics examples ─────────────────────────────────────────────────
# Examples require campello_renderer (fetched automatically).
# They are macOS-only unless ported to other platforms.

if(NOT APPLE OR IOS)
    message(STATUS "campello_physics examples: macOS host required — skipping")
    return()
endif()

# ── Fetch campello_renderer ───────────────────────────────────────────────────
include(FetchContent)

# campello_renderer → gltf fetches vector_math under the name "extern_vector_math",
# which creates a duplicate "vector_math" target.  Pre-populate it using the copy
# already fetched by the root CMakeLists so CMake skips re-fetching and
# add_subdirectory, and reuses the existing target.
# CMake 3.24+: FetchContent_GetProperties checks DEFINED on the global property,
# which requires define_property to have been called (set_property alone is not enough).
FetchContent_GetProperties(vector_math)
if(vector_math_POPULATED)
    foreach(_prop IN ITEMS
            _FetchContent_extern_vector_math_populated
            _FetchContent_extern_vector_math_sourceDir
            _FetchContent_extern_vector_math_binaryDir)
        define_property(GLOBAL PROPERTY ${_prop})
    endforeach()
    set_property(GLOBAL PROPERTY _FetchContent_extern_vector_math_populated TRUE)
    set_property(GLOBAL PROPERTY _FetchContent_extern_vector_math_sourceDir "${vector_math_SOURCE_DIR}")
    set_property(GLOBAL PROPERTY _FetchContent_extern_vector_math_binaryDir "${vector_math_BINARY_DIR}")
endif()

FetchContent_Declare(
    campello_renderer
    GIT_REPOSITORY https://github.com/rusoleal/campello_renderer.git
    GIT_TAG        v0.1.4
)
FetchContent_MakeAvailable(campello_renderer)

# ── Shared example utilities ──────────────────────────────────────────────────
add_subdirectory(common)

# ── Example applications ──────────────────────────────────────────────────────
add_subdirectory(falling_sphere)
add_subdirectory(character_controller)
add_subdirectory(vehicle)
add_subdirectory(ragdoll)
