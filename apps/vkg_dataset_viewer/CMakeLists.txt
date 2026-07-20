# Standalone .h5 dataset viewer. Only needs the dataset library + OpenCV highgui.
find_package(OpenCV REQUIRED core highgui imgproc)

add_executable(vkg_dataset_viewer src/vkg_dataset_viewer_main.cc)
target_link_libraries(vkg_dataset_viewer PRIVATE
    vkg::dataset glog gflags ${OpenCV_LIBRARIES})

install(TARGETS vkg_dataset_viewer RUNTIME DESTINATION .)
