#include "mapcloud.h"


void MapCloud::addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, Eigen::Isometry3d T)
{
  pcl::transformPointCloud(*mapcloud, *tmpcloud, T);
  *mapcloud = *tmpcloud + *pointcloud;
  
}

//void MapCloud::addKeyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, Eigen::Isometry3d T)
//{
  
//}