target_sources(campello_physics PRIVATE
    physics_world_stub.cpp   # removed once Phase 1 sources are added

    # Phase 1: foundation
    foundation/allocator.cpp
    foundation/thread_pool.cpp

    # Phase 2: shapes (primitives are header-only)
    shapes/convex_hull_shape.cpp
    shapes/triangle_mesh_shape.cpp
    shapes/height_field_shape.cpp

    # Phase 3: broad phase
    broadphase/dynamic_bvh.cpp
    broadphase/broad_phase.cpp

    # Phase 4: narrow phase
    narrowphase/gjk.cpp
    narrowphase/sat.cpp
    narrowphase/narrow_phase.cpp

    # Phase 5: dynamics
    dynamics/body_pool.cpp
    dynamics/integrator.cpp
    dynamics/body_interface.cpp

    # Phase 6: constraints
    constraints/constraint_solver.cpp
    constraints/ball_socket_constraint.cpp
    constraints/fixed_constraint.cpp
    constraints/hinge_constraint.cpp
    constraints/slider_constraint.cpp
    constraints/distance_constraint.cpp
    constraints/d6_constraint.cpp

    # Phase 7: world
    world/physics_world.cpp

    # Phase 12: advanced features
    world/buoyancy_system.cpp
    world/articulated_body_system.cpp
    world/ragdoll_factory.cpp
    world/vehicle_system.cpp

    # Phase 8: queries
    query/raycast.cpp
    query/shape_cast.cpp
    query/overlap.cpp

    # Phase 9: events
    # events/event_dispatcher.cpp

    # Phase 13: debug, profiling & serialization
    debug/debug_draw.cpp
    debug/physics_debug_draw.cpp
    world/physics_world_serialization.cpp
)

if(CAMPELLO_PHYSICS_GPU)
    target_sources(campello_physics PRIVATE
        # Phase 14: GPU compute backend
        gpu/gpu_physics_backend.cpp
    )
endif()
