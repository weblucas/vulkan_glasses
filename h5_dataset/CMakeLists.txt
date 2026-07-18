# Standalone library for reading/writing the renderer's .h5 dataset format.
# Depends only on OpenCV + HighFive/HDF5 (+ glog) — no Vulkan, no renderer lib —
# so any application can link it to load/save this format.

find_package( OpenCV REQUIRED core )

add_library(${PROJECT_NAME}_h5_dataset "src/h5_dataset.cc")
target_include_directories(${PROJECT_NAME}_h5_dataset
    PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_h5_dataset
    PUBLIC HighFive ${OpenCV_LIBRARIES} glog)

install(TARGETS ${PROJECT_NAME}_h5_dataset
    ARCHIVE DESTINATION lib)
install(FILES include/h5_dataset.h
    DESTINATION include/vulkan_glasses)
