cmake_minimum_required(VERSION 3.21)  # PROJECT_IS_TOP_LEVEL
project(vulkan_glasses LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release by default.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Default the install prefix to <repo>/bin, but only for a standalone top-level
# build and only if the user didn't set one (no FORCE, so colcon/-DCMAKE_INSTALL_PREFIX win).
if(PROJECT_IS_TOP_LEVEL AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin" CACHE PATH "Install path prefix" FORCE)
endif()

# Keep library search paths on installed binaries so runtime libs in non-standard
# locations (e.g. the HDF5 serial dir) still resolve after install.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

# --- Orthogonal build options -------------------------------------------------
# The two libraries build independently (artifacts: render-lib-only / dataset-lib-only).
# Apps are off by default and pull in the libraries they need.
option(BUILD_RENDER_LIB      "Build the Vulkan render library (vkg::render)"  ON)
option(BUILD_DATASET_LIB     "Build the .h5 dataset library (vkg::dataset)"   ON)
option(BUILD_CSV_RENDERER    "Build the standalone CSV renderer app"          OFF)
option(BUILD_DATASET_VIEWER  "Build the standalone .h5 dataset viewer app"    OFF)
option(BUILD_TESTS           "Build unit tests"                               OFF)

# Apps imply their libraries.
if(BUILD_CSV_RENDERER)
    set(BUILD_RENDER_LIB ON)
    set(BUILD_DATASET_LIB ON)
endif()
if(BUILD_DATASET_VIEWER)
    set(BUILD_DATASET_LIB ON)
endif()

# The libraries emit install/export rules when built from this top-level tree.
set(VKG_ENABLE_INSTALL ON)

if(BUILD_RENDER_LIB)
    add_subdirectory(libs/render)
endif()
if(BUILD_DATASET_LIB)
    add_subdirectory(libs/dataset)
endif()

if(BUILD_CSV_RENDERER)
    add_subdirectory(apps/csv_renderer)
    install(PROGRAMS scripts/run_csv_renderer.sh DESTINATION .)
endif()
if(BUILD_DATASET_VIEWER)
    add_subdirectory(apps/vkg_dataset_viewer)
    install(PROGRAMS scripts/run_vkg_dataset_viewer.sh DESTINATION .)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
