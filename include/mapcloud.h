#include "common.h" 
#pragma once
//这是一个类
class MapCloud
{
public:
  MapCloud();
    
  MapCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
    sourcecloud = pointcloud;
    mapcloud = pointcloud;
    targetcloud = pointcloud;
    tmpcloud = pointcloud;
  }

  void addTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr targetpointcloud); //添加目标点云
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr iterativeClosestPoint(); //迭代最近点
  
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr mapcloud;//点云地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr sourcecloud;  //源点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetcloud;  // 目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud; // 中间变量，暂时存储点云用
  pcl::PointCloud<pcl::PointXYZ> final; //icp 算法中用来存储坐标变换后的点云
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;//icp 智能指针
  
  pcl::VoxelGrid<pcl::PointXYZ> voxel; //网格滤波器，调整点云地图分辨率

  double gridsize = 0.1; //点云分辨率
    
  Eigen::Matrix<float, 4, 4> trans; //两点云之间的变换坐标

};

