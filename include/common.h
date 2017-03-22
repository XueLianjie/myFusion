//C++ standard library
#include <iostream>
#include <sstream>
#include <fstream>

#include <chrono>
#include <string>
#include <boost/concept_check.hpp>

//Opencv library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>