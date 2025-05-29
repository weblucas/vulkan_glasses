cmake_minimum_required(VERSION 3.14) 
project(vulkan_glasses)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW) # Use the new behavior for Boost::headers alias
endif()

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



# Options for ROS versions
option(BUILD_WITH_ROS1 "Build with ROS 1 (catkin) extensions" OFF)
option(BUILD_WITH_ROS2 "Build with ROS 2 (ament) extensions" OFF)
option(BUILD_WITH_CSV_PROCESSOR "Build with CSV Processor application" OFF)
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


endif(COMPILE_SHADERS)


add_subdirectory(lib)


if(BUILD_WITH_CSV_PROCESSOR)
    set (HIGHFIVE_EXAMPLES OFF)
    set (HIGHFIVE_UNIT_TESTS OFF)
    set (HIGHFIVE_BUILD_DOCS OFF)
    add_subdirectory("third_party/HighFive")
    add_subdirectory(apps/csv_renderer)
endif()

# --- ROS 1 (Catkin) ---
if(BUILD_WITH_ROS1)
    # Attempt to find Catkin (ROS 1)
    # This block helps ensure that you don't try to build ROS1 stuff in a ROS2 workspace or vice-versa.
    # You might also check environment variables like ROS_VERSION.
    if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
        find_package(catkin QUIET COMPONENTS roscpp std_msgs) # Add other ROS1 pkgs
        if(catkin_FOUND)
            message(STATUS "ROS 1 (catkin) found. Configuring ROS 1 wrapper.")
            add_subdirectory(apps/ros1_node)
        else()
            message(WARNING "BUILD_WITH_ROS1 is ON, but Catkin (ROS 1) was not found or required components are missing. ROS 1 wrapper will NOT be built.")
            set(BUILD_WITH_ROS1 OFF CACHE BOOL "Build with ROS 1 extensions" FORCE)
        endif()
    else()
        message(WARNING "BUILD_WITH_ROS1 is ON, but ROS_VERSION environment variable is not '1' or not set. Skipping ROS 1 wrapper.")
        set(BUILD_WITH_ROS1 OFF CACHE BOOL "Build with ROS 1 extensions" FORCE)
    endif()
endif()

# --- ROS 2 (Ament) ---
if(BUILD_WITH_ROS2)
    # Attempt to find Ament (ROS 2)
    if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
        find_package(ament_cmake QUIET)
        if(ament_cmake_FOUND)
            message(STATUS "ROS 2 (ament_cmake) found. Configuring ROS 2 wrapper.")
            find_package(rclcpp QUIET) # Example, add other ROS2 pkgs
            find_package(std_msgs QUIET) # Example
            if(rclcpp_FOUND)
                 add_subdirectory(ros2_wrapper)
            else()
                message(WARNING "BUILD_WITH_ROS2 is ON, but rclcpp (ROS 2) was not found. ROS 2 wrapper will NOT be built.")
                set(BUILD_WITH_ROS2 OFF CACHE BOOL "Build with ROS 2 extensions" FORCE)
            endif()
        else()
            message(WARNING "BUILD_WITH_ROS2 is ON, but ament_cmake (ROS 2) was not found. ROS 2 wrapper will NOT be built.")
            set(BUILD_WITH_ROS2 OFF CACHE BOOL "Build with ROS 2 extensions" FORCE)
        endif()
    else()
        message(WARNING "BUILD_WITH_ROS2 is ON, but ROS_VERSION environment variable is not '2' or not set. Skipping ROS 2 wrapper.")
        set(BUILD_WITH_ROS2 OFF CACHE BOOL "Build with ROS 2 extensions" FORCE)
    endif()
endif()

# Note: You might want to prevent building both ROS1 and ROS2 wrappers simultaneously
# if they are intended to be used in mutually exclusive environments.
if(BUILD_WITH_ROS1 AND BUILD_WITH_ROS2)
    message(WARNING "Both BUILD_WITH_ROS1 and BUILD_WITH_ROS2 are ON. Ensure your environment and use case support this.")
endif()