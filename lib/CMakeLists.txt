

find_package( Eigen3 REQUIRED )
find_package( OpenCV REQUIRED core highgui imgproc )
find_package( Vulkan )

if( NOT Vulkan_FOUND )
    message(FATAL_ERROR "Vulkan not found. Please install Vulkan development files.")
endif()


include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( include )


add_library(${PROJECT_NAME}_lib "src/VulkanTools.cc" "src/vulkan_renderer.cc")
target_link_libraries(${PROJECT_NAME}_lib  ${OpenCV_LIBRARIES}  ${Vulkan_LIBRARY} )

if(COMPILE_SHADERS)
    add_dependencies(${PROJECT_NAME}_lib shaders)
endif(COMPILE_SHADERS)



