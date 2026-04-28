add_executable(campello_physics_tests
    # Phase 1
    test_foundation.cpp

    # Phase 2
    test_shapes.cpp

    # Phase 3
    test_broadphase.cpp

    # Phase 4
    test_narrowphase.cpp

    # Phase 5
    test_dynamics.cpp

    # Phase 6
    test_constraints.cpp

    # Phase 7
    test_world.cpp
    test_body_interface.cpp

    # Phase 8
    test_queries.cpp

    # Phase 9
    test_events.cpp

    # Phase 11
    test_ccd.cpp

    # Phase 12
    test_buoyancy.cpp
    test_articulated_body.cpp
    test_ragdoll.cpp
    test_vehicle.cpp

    # Phase 15
    test_island_detection.cpp

    # Stress / torture
    test_stress.cpp

    # Fuzz / adversarial
    test_fuzz.cpp

    # Phase 13
    test_debug_draw.cpp
    test_serialization.cpp

    # Phase 14
    test_gpu_backend.cpp

    main.cpp
)

target_link_libraries(campello_physics_tests
    PRIVATE
        campello_physics
        GTest::gtest
)

include(GoogleTest)
if(EMSCRIPTEN)
    # Defer test discovery to test-runtime so we don't need to execute
    # the cross-compiled .js binary at configure time.
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
    # Make sure CTest knows to run wasm binaries via Node.js.
    if(NOT CMAKE_CROSSCOMPILING_EMULATOR)
        find_program(NODE_EXECUTABLE NAMES node nodejs REQUIRED)
        set(CMAKE_CROSSCOMPILING_EMULATOR ${NODE_EXECUTABLE})
    endif()
endif()
gtest_discover_tests(campello_physics_tests)

if(CAMPELLO_PHYSICS_BUILD_BENCHMARKS)
    add_executable(campello_physics_bench
        bench_simulation.cpp
        bench_main.cpp
    )
    target_link_libraries(campello_physics_bench
        PRIVATE
            campello_physics
            benchmark::benchmark
    )
endif()
