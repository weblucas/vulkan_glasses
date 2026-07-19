cmake_minimum_required(VERSION 3.14) 
project(vulkan_glasses)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the build type to Release by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Set install prefix to project_folder/bin
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin" CACHE PATH "Install path prefix" FORCE)

# Keep library search paths on the installed binaries so runtime libraries in
# non-standard locations (e.g. the HDF5 serial dir) still resolve after install.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)



# Options for ROS versions
option(BUILD_WITH_ROS1 "Build with ROS 1 (catkin) extensions" OFF)
option(BUILD_WITH_ROS2 "Build with ROS 2 (ament) extensions" OFF)
option(BUILD_WITH_CSV_PROCESSOR "Build with CSV Processor application" OFF)
option(BUILD_WITH_H5_VIEWER "Build the .h5 image viewer" OFF)
option(COMPILE_SHADERS "Build with CSV Processor application" ON)

if(COMPILE_SHADERS)
    # Find the glslangValidator executable
    find_program(GLSLANG_VALIDATOR glslangValidator)
    if(NOT GLSLANG_VALIDATOR)
        message(FATAL_ERROR "glslangValidator not found. Make sure it's in your PATH or Vulkan SDK is properly installed.")
    endif()

    # Define the input shader files
set(SHADER_VERT_FILE shaders/vrglasses4robots_shader.vert)
set(SHADER_FRAG_FILE shaders/vrglasses4robots_shader.frag)

# Define the output SPIR-V files
set(SHADER_VERT_SPV ${CMAKE_BINARY_DIR}/vrglasses4robots_shader.vert.spv)
set(SHADER_FRAG_SPV ${CMAKE_BINARY_DIR}/vrglasses4robots_shader.frag.spv)



# Custom command to compile the vertex shader
add_custom_command(
    OUTPUT ${SHADER_VERT_SPV}
    COMMAND ${GLSLANG_VALIDATOR} -V ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_VERT_FILE} -o ${SHADER_VERT_SPV}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_VERT_FILE}
    COMMENT "Compiling vertex shader: ${SHADER_VERT_FILE}"
)

# Custom command to compile the fragment shader
add_custom_command(
    OUTPUT ${SHADER_FRAG_SPV}
    COMMAND ${GLSLANG_VALIDATOR} -V ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_FRAG_FILE} -o ${SHADER_FRAG_SPV}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_FRAG_FILE}
    COMMENT "Compiling fragment shader: ${SHADER_FRAG_FILE}"
)

# Create a custom target to ensure shaders are compiled before other targets
add_custom_target(shaders ALL DEPENDS ${SHADER_VERT_SPV} ${SHADER_FRAG_SPV})
install(FILES ${SHADER_VERT_SPV} ${SHADER_FRAG_SPV} DESTINATION shaders)

endif(COMPILE_SHADERS)


add_subdirectory(lib)


# HighFive (HDF5 wrapper) is shared by both HDF5-based apps; add it once.
if(BUILD_WITH_CSV_PROCESSOR OR BUILD_WITH_H5_VIEWER)
    set (HIGHFIVE_EXAMPLES OFF)
    set (HIGHFIVE_UNIT_TESTS OFF)
    set (HIGHFIVE_BUILD_DOCS OFF)
    # Disable HighFive's Boost support so it neither requires Boost at configure
    # time nor defines H5_USE_BOOST on targets that link it. We read/write HDF5
    # with plain std::vector, so Boost is not needed anywhere in the project.
    set (USE_BOOST OFF CACHE BOOL "Disable HighFive Boost support" FORCE)
    add_subdirectory("third_party/HighFive")
    # Standalone .h5 dataset I/O library shared by the renderer and the viewer.
    add_subdirectory(h5_dataset)
endif()

if(BUILD_WITH_CSV_PROCESSOR)
    add_subdirectory(apps/csv_renderer)
    # Launcher that runs the renderer with the shader folder resolved relative to
    # the install location, so the install tree is self-contained.
    install(PROGRAMS scripts/run_csv_renderer.sh DESTINATION .)
endif()

if(BUILD_WITH_H5_VIEWER)
    add_subdirectory(apps/h5_viewer)
    install(PROGRAMS scripts/run_h5_viewer.sh DESTINATION .)
endif()

option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

