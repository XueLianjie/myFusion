#include "mapcloud.h"

void MapCloud::addTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr targetpointcloud)
{
  targetcloud = targetpointcloud;
  
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapCloud::iterativeClosestPoint()
{
  icp.setInputSource(sourcecloud);
  icp.setInputTarget(targetcloud);
  icp.align(final);//
  
  std::cout << "point cloud size = " << final.points.size() << std::endl;
  final.points.clear();
  
  trans = icp.getFinalTransformation();
  pcl::transformPointCloud(*mapcloud, *tmpcloud, trans);//对地图点云做trans变换
  *tmpcloud += *targetcloud;
  mapcloud = tmpcloud;
  sourcecloud = targetcloud;
  
  std::cout << "has converged:" << icp.hasConverged() << "\nscore: " << icp.getFitnessScore() << std::endl;
  // 滤波降采样
  voxel.setLeafSize( gridsize, gridsize, gridsize );//设置网格滤波器的分辨率
  voxel.setInputCloud( mapcloud );//加入待滤波的地图点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud ( new pcl::PointCloud<pcl::PointXYZ> );//此处必须为新的空指针，否则程序停止，智能指针不需要释放。
  voxel.filter( *map_cloud );

  mapcloud = map_cloud;      
  return mapcloud;
  
}