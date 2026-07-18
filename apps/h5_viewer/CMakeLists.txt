find_package( OpenCV REQUIRED core highgui imgproc )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME}_h5_viewer "src/h5_viewer_main.cc")
target_link_libraries(${PROJECT_NAME}_h5_viewer glog gflags ${PROJECT_NAME}_h5_dataset ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_h5_viewer
    RUNTIME DESTINATION .)
