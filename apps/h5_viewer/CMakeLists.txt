find_package( OpenCV REQUIRED core highgui imgproc )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( include )

add_executable(${PROJECT_NAME}_h5_viewer "src/h5_viewer_main.cc" "src/h5_reader.cc")
target_link_libraries(${PROJECT_NAME}_h5_viewer glog gflags HighFive ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_h5_viewer
    RUNTIME DESTINATION .)
