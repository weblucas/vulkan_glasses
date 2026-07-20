# Standalone CSV renderer. Reuses the two libraries via their namespaced targets;
# Vulkan/OpenCV/Eigen include+link flow transitively from vkg::render.
find_package(OpenCV REQUIRED core highgui imgproc)

add_executable(vkg_csv_renderer src/csv_main.cc src/csv_processor.cc)
target_include_directories(vkg_csv_renderer PRIVATE include)
target_link_libraries(vkg_csv_renderer PRIVATE
    vkg::render vkg::dataset glog gflags ${OpenCV_LIBRARIES})

# Install the app next to the compiled shaders it needs (the launcher passes
# --shader_folder=<dir>/shaders). Shader spv paths come from the render lib.
install(TARGETS vkg_csv_renderer RUNTIME DESTINATION .)
install(FILES ${VKG_SHADER_SPV_FILES} DESTINATION shaders)
