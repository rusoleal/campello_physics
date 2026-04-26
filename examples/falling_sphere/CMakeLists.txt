
add_executable(example_falling_sphere MACOSX_BUNDLE
    main.mm
    AppDelegate.h
    AppDelegate.mm
    ViewController.h
    ViewController.mm
)

set_target_properties(example_falling_sphere PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist"
    XCODE_ATTRIBUTE_PRODUCT_NAME "FallingSphere"
)

target_link_libraries(example_falling_sphere PRIVATE
    campello_physics_example_common
    "-framework Cocoa"
    "-framework Metal"
    "-framework MetalKit"
    "-framework QuartzCore"
)

target_compile_options(example_falling_sphere PRIVATE
    -Wno-deprecated-declarations  # Cocoa / Metal APIs on older SDK
)
