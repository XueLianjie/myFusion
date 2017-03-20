
#PCL
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})

# eigen
include_directories( "/usr/include/eigen3/" )

add_library(frame frame.cpp)

add_executable(generatePointCloud generatePointCloud.cpp)

target_link_libraries(generatePointCloud ${OpenCV_LIBS} 
					 ${PCL_LIBRARIES}
					 frame)


