






find_package( Eigen3 REQUIRED )
find_package( OpenCV REQUIRED core highgui imgproc )
find_package( Vulkan )


find_package( Boost COMPONENTS filesystem REQUIRED )


include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )




include_directories( include )


add_executable(${PROJECT_NAME}_csv_renderer  "src/csv_main.cc"  "src/csv_processor.cc" "src/hdf5_utils.cc")
target_include_directories(${PROJECT_NAME}_csv_renderer PRIVATE ${CMAKE_SOURCE_DIR}/lib/include)
target_link_libraries(${PROJECT_NAME}_csv_renderer glog gflags HighFive ${OpenCV_LIBRARIES} Boost::filesystem ${Vulkan_LIBRARY} ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_csv_renderer
    RUNTIME DESTINATION .)


