cmake_minimum_required(VERSION 3.21)
project(vulkan_glasses_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Reuse the standalone vkg libraries from the repo. They are add_subdirectory'd
# (one-step `colcon build`): because this project is the top level, the libs see
# VKG_ENABLE_INSTALL unset / PROJECT_IS_TOP_LEVEL false and therefore only DEFINE
# their targets (no install/export rules leak into the ament install tree).
set(VKG_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_subdirectory(${VKG_REPO_ROOT}/libs/render  ${CMAKE_BINARY_DIR}/vkg_render)
add_subdirectory(${VKG_REPO_ROOT}/libs/dataset ${CMAKE_BINARY_DIR}/vkg_dataset)

add_executable(vkg_ros2_node src/vkg_ros2_node.cc)
target_link_libraries(vkg_ros2_node vkg::render vkg::dataset)
ament_target_dependencies(vkg_ros2_node
  rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge image_transport ament_index_cpp)

install(TARGETS vkg_ros2_node DESTINATION lib/${PROJECT_NAME})

# Shaders (compiled by the render lib via vkg_compile_shaders) go into this
# package's share dir; the node's default shader_folder points there.
install(FILES ${VKG_SHADER_SPV_FILES} DESTINATION share/${PROJECT_NAME}/shaders)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
