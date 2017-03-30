#include "mapcloud.h"

void MapCloud::addTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr targetpointcloud)
{
  targetcloud = targetpointcloud;
  
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapCloud::iterativeClosestPoint()
{

  targetcloud = MapCloud::gridfilter(gridsize, targetcloud);//对目标点云进行滤波降采样
  //滤波器移除离群点
  sor.setInputCloud(targetcloud);//设置输入点云
  sor.setMeanK(MeanK);//设置进行统计时考虑查询点临近点数
  sor.setStddevMulThresh(outlier_threshold);//设置判断是否为离群点的阈值
  sor.filter(*targetcloud);//移除离群点滤波
  
  //icp 迭代最近点进行位姿估计
  icp.setInputSource(sourcecloud);//设置源点云
  icp.setInputTarget(targetcloud);//设置目标点云
  icp.align(final);//icp 迭代
  
  std::cout << "point cloud size = " << final.points.size() << std::endl;//输出点云点数
  final.points.clear();//释放final 指向的点云
  
  trans = icp.getFinalTransformation();//icp 求得的变换矩阵
  
  //判断变换矩阵是否在合理范围内
  
  
  /*
  //拼接点云，以最新的相机坐标系为世界坐标系
  pcl::transformPointCloud(*mapcloud, *tmpcloud, trans);//对地图点云做trans变换
  *tmpcloud += *targetcloud;//拼接点云
  mapcloud = tmpcloud;//更新点云地图
  sourcecloud = targetcloud;//更新源点云
  
  std::cout << "has converged:" << icp.hasConverged() << "\nscore: " << icp.getFitnessScore() << std::endl;
  mapcloud = MapCloud::gridfilter(0.01, mapcloud);
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