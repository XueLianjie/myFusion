
#PCL
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})

#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )

# CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )


# eigen
include_directories( "/usr/include/eigen3/" )

add_library(frame frame.cpp)
add_library(mapcloud mapcloud.cpp)
add_library(config config.cpp)
#add_library(visualodometry visualodometry.cpp)


add_executable(generatePointCloud generatePointCloud.cpp)
#add_executable(getconstruction testsurfaceconstruction.cpp)

target_link_libraries(generatePointCloud ${OpenCV_LIBS} 
					 ${PCL_LIBRARIES}
					 frame 
					 mapcloud
					 config
					    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} )
#					 visualodometry

#target_link_libraries(getconstruction    ${OpenCV_LIBS} 
#					 ${PCL_LIBRARIES}
#
#					 frame 
#					 mapcloud
#					 config)
#
