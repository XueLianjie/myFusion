//C++ standard library
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <string>
#include <boost/concept_check.hpp>
#include <list>
#include <vector>
#include <memory>
#include <set>
#include <map>
#include <unordered_map>

//for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>

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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//my library
//#include "config.h"
//#include "frame.h"
//#include "mapcloud.h"
