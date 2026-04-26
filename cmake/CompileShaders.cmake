set(CAMPELLO_SHADER_OUT_DIR "${CMAKE_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${CAMPELLO_SHADER_OUT_DIR}")

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin" OR CMAKE_SYSTEM_NAME STREQUAL "iOS")
    # Metal: .metal → .air → .metallib
    set(SHADER_SRC_DIR "${CMAKE_SOURCE_DIR}/shaders/msl")
    set(SHADER_EXT "metal")
    set(SHADER_OUT_EXT "metallib")

    foreach(KERNEL broadphase_aabb broadphase_pairs xpbd_predict xpbd_contacts xpbd_finalize)
        set(SRC "${SHADER_SRC_DIR}/${KERNEL}.${SHADER_EXT}")
        set(AIR "${CAMPELLO_SHADER_OUT_DIR}/${KERNEL}.air")
        set(OUT "${CAMPELLO_SHADER_OUT_DIR}/${KERNEL}.${SHADER_OUT_EXT}")

        add_custom_command(OUTPUT "${AIR}"
            COMMAND xcrun -sdk macosx metal -c "${SRC}" -o "${AIR}"
            DEPENDS "${SRC}"
            COMMENT "Compiling Metal shader ${KERNEL}.metal"
        )
        add_custom_command(OUTPUT "${OUT}"
            COMMAND xcrun -sdk macosx metallib "${AIR}" -o "${OUT}"
            DEPENDS "${AIR}"
            COMMENT "Linking Metal library ${KERNEL}.metallib"
        )
        list(APPEND CAMPELLO_SHADER_OUTPUTS "${OUT}")
    endforeach()

elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_SYSTEM_NAME STREQUAL "Android")
    # Vulkan: .comp → .spv via glslc
    find_program(GLSLC glslc HINTS "$ENV{VULKAN_SDK}/bin" REQUIRED)
    set(SHADER_SRC_DIR "${CMAKE_SOURCE_DIR}/shaders/glsl")

    foreach(KERNEL broadphase_aabb broadphase_pairs xpbd_predict xpbd_contacts xpbd_finalize)
        set(SRC "${SHADER_SRC_DIR}/${KERNEL}.comp")
        set(OUT "${CAMPELLO_SHADER_OUT_DIR}/${KERNEL}.spv")

        add_custom_command(OUTPUT "${OUT}"
            COMMAND "${GLSLC}" "${SRC}" -o "${OUT}"
            DEPENDS "${SRC}"
            COMMENT "Compiling GLSL shader ${KERNEL}.comp"
        )
        list(APPEND CAMPELLO_SHADER_OUTPUTS "${OUT}")
    endforeach()

elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # DirectX 12: .hlsl → .cso via fxc
    find_program(FXC fxc HINTS "C:/Program Files (x86)/Windows Kits/10/bin/${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION}/x64" REQUIRED)
    set(SHADER_SRC_DIR "${CMAKE_SOURCE_DIR}/shaders/hlsl")

    foreach(KERNEL broadphase_aabb broadphase_pairs xpbd_predict xpbd_contacts xpbd_finalize)
        set(SRC "${SHADER_SRC_DIR}/${KERNEL}.hlsl")
        set(OUT "${CAMPELLO_SHADER_OUT_DIR}/${KERNEL}.cso")

        add_custom_command(OUTPUT "${OUT}"
            COMMAND "${FXC}" /T cs_5_1 /E "${KERNEL}" /Fo "${OUT}" "${SRC}"
            DEPENDS "${SRC}"
            COMMENT "Compiling HLSL shader ${KERNEL}.hlsl"
        )
        list(APPEND CAMPELLO_SHADER_OUTPUTS "${OUT}")
    endforeach()
endif()

add_custom_target(campello_physics_shaders ALL DEPENDS ${CAMPELLO_SHADER_OUTPUTS})
add_dependencies(campello_physics campello_physics_shaders)
