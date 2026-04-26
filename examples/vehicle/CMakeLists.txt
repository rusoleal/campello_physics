
add_executable(example_vehicle MACOSX_BUNDLE
    main.mm
    AppDelegate.h
    AppDelegate.mm
    ViewController.h
    ViewController.mm
)

set_target_properties(example_vehicle PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist"
    XCODE_ATTRIBUTE_PRODUCT_NAME "Vehicle"
)

target_link_libraries(example_vehicle PRIVATE
    campello_physics_example_common
    "-framework Cocoa"
    "-framework Metal"
    "-framework MetalKit"
    "-framework QuartzCore"
)

target_compile_options(example_vehicle PRIVATE
    -Wno-deprecated-declarations
)
