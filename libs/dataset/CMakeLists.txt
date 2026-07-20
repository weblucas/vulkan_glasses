# vkg::dataset — read/write the renderer's .h5 dataset format.
# Depends only on OpenCV (public, its headers use cv::Mat) + HDF5/HighFive (private
# implementation) + glog. No Vulkan, no renderer lib. HighFive is header-only, so
# we use its headers directly and link the system HDF5 C library — no HighFive
# CMake subproject needed.
project(vkg_dataset LANGUAGES CXX)

find_package(OpenCV REQUIRED core)
find_package(HDF5 REQUIRED COMPONENTS C)

# HighFive headers (header-only). Overridable; defaults to the vendored copy.
if(NOT DEFINED VKG_HIGHFIVE_INCLUDE_DIR)
    set(VKG_HIGHFIVE_INCLUDE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/HighFive/include)
endif()

add_library(vkg_dataset src/h5_dataset.cc)
add_library(vkg::dataset ALIAS vkg_dataset)
# So the INSTALLED imported target is vkg::dataset (not vkg::vkg_dataset).
set_target_properties(vkg_dataset PROPERTIES EXPORT_NAME dataset)
target_compile_features(vkg_dataset PUBLIC cxx_std_17)

target_include_directories(vkg_dataset PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS})
# HighFive + HDF5 are implementation-only (h5_dataset.h exposes only cv::Mat).
target_include_directories(vkg_dataset PRIVATE
    $<BUILD_INTERFACE:${VKG_HIGHFIVE_INCLUDE_DIR}>
    ${HDF5_INCLUDE_DIRS})
target_link_libraries(vkg_dataset PUBLIC ${OpenCV_LIBRARIES})
# Private, but a static-library consumer still links these (carried as LINK_ONLY).
# Plain paths/names (not exported targets) -> safe in the installed export.
target_link_libraries(vkg_dataset PRIVATE ${HDF5_C_LIBRARIES} glog)

if(NOT DEFINED VKG_ENABLE_INSTALL)
    set(VKG_ENABLE_INSTALL ${PROJECT_IS_TOP_LEVEL})
endif()

if(VKG_ENABLE_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)
    install(TARGETS vkg_dataset EXPORT vkg_datasetTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(EXPORT vkg_datasetTargets NAMESPACE vkg::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vkg_dataset)
    configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/vkg_datasetConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/vkg_datasetConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vkg_dataset)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vkg_datasetConfig.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vkg_dataset)
endif()
