
add_executable(example_character_controller MACOSX_BUNDLE
    main.mm
    AppDelegate.h
    AppDelegate.mm
    ViewController.h
    ViewController.mm
)

set_target_properties(example_character_controller PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist"
    XCODE_ATTRIBUTE_PRODUCT_NAME "CharacterController"
)

target_link_libraries(example_character_controller PRIVATE
    campello_physics_example_common
    "-framework Cocoa"
    "-framework Metal"
    "-framework MetalKit"
    "-framework QuartzCore"
)

target_compile_options(example_character_controller PRIVATE
    -Wno-deprecated-declarations
)
