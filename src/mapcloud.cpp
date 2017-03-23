#include "mapcloud.h"

void MapCloud::addTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr targetpointcloud)
{
  targetcloud = targetpointcloud;
  
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapCloud::iterativeClosestPoint()
{

  targetcloud = MapCloud::gridfilter(gridsize, targetcloud);//对目标点云进行滤波降采样
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
  mapcloud = MapCloud::gridfilter(0.01, mapcloud);
  
  /*
  // 滤波降采样
  voxel.setLeafSize( gridsize, gridsize, gridsize );//设置网格滤波器的分辨率
  voxel.setInputCloud( mapcloud );//加入待滤波的地图点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud ( new pcl::PointCloud<pcl::PointXYZ> );//此处必须为新的空指针，否则程序停止，智能指针不需要释放。
  voxel.filter( *map_cloud );

  mapcloud = map_cloud;  
  */
  return mapcloud;
  
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapCloud::gridfilter(double grid_size, pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud)
{
  // 滤波降采样
  voxel.setLeafSize( grid_size, grid_size, grid_size );//设置网格滤波器的分辨率
  voxel.setInputCloud( voxel_cloud );//加入待滤波的地图点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud ( new pcl::PointCloud<pcl::PointXYZ> );//此处必须为新的空指针，否则程序停止，智能指针不需要释放。
  voxel.filter( *map_cloud );
  return map_cloud;
  
}