# vkg::render — headless Vulkan renderer library. ROS-agnostic.
#
# Builds three ways, all from this one file:
#   * standalone      (cmake -S libs/render)         -> PROJECT_IS_TOP_LEVEL, installs+exports
#   * from repo root  (add_subdirectory)             -> VKG_ENABLE_INSTALL set by the top level
#   * from the ROS2 ament package (add_subdirectory) -> neither set, only defines the target
project(vkg_render LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED core highgui imgproc)
find_package(Vulkan REQUIRED)

add_library(vkg_render src/vulkan_renderer.cc src/VulkanTools.cc)
add_library(vkg::render ALIAS vkg_render)
# So the INSTALLED imported target is vkg::render (not vkg::vkg_render), matching the alias.
set_target_properties(vkg_render PROPERTIES EXPORT_NAME render)
target_compile_features(vkg_render PUBLIC cxx_std_17)

target_include_directories(vkg_render PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS})
# Vulkan_LIBRARIES / OpenCV_LIBRARIES are file paths (not exported targets), so
# they are safe to carry in the installed link interface.
# glog is used by the public header (glog/logging.h); a consumer of the static
# lib must link it too. Plain name (-lglog), safe in the installed export.
target_link_libraries(vkg_render PUBLIC ${OpenCV_LIBRARIES} ${Vulkan_LIBRARIES} glog)

# Compile the shaders (runtime assets) into the build tree; consumers install /
# point --shader_folder at them (the renderer loads vkg_shader.{vert,frag}.spv).
include(${CMAKE_CURRENT_LIST_DIR}/cmake/CompileShaders.cmake)
vkg_compile_shaders(vkg_shaders
    ${PROJECT_BINARY_DIR}/shaders
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
add_dependencies(vkg_render vkg_shaders)
# Expose the compiled shader dir/files to consumers (apps install them).
set(VKG_SHADER_SPV_DIR ${PROJECT_BINARY_DIR}/shaders CACHE INTERNAL "vkg shader spv dir")
set(VKG_SHADER_SPV_FILES ${vkg_shaders_SPV_FILES} CACHE INTERNAL "vkg shader spv files")

if(NOT DEFINED VKG_ENABLE_INSTALL)
    set(VKG_ENABLE_INSTALL ${PROJECT_IS_TOP_LEVEL})
endif()

if(VKG_ENABLE_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)
    install(TARGETS vkg_render EXPORT vkg_renderTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(FILES ${VKG_SHADER_SPV_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/vkg/shaders)
    install(EXPORT vkg_renderTargets NAMESPACE vkg::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vkg_render)
    configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/vkg_renderConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/vkg_renderConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vkg_render)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vkg_renderConfig.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vkg_render)
endif()
