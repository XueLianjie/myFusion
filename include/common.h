//C++ standard library
#include <iostream>
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

//Eigen
#include <Eigen/Geometry>