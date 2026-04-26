
add_library(campello_physics_example_common STATIC
    GLBScene.cpp
)

target_include_directories(campello_physics_example_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${campello_renderer_SOURCE_DIR}/inc
)

target_link_libraries(campello_physics_example_common PUBLIC
    campello_physics
    campello_renderer
)

target_compile_features(campello_physics_example_common PUBLIC cxx_std_20)
