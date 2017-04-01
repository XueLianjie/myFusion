#include "common.h"
#include "config.h"
#pragma once
//这是一个类
class MapCloud
{
public:
  MapCloud()
  {
   first_depth_addr = Config::get<std::string> ("first_depth_addr");
  }
    
  MapCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) 
  {
    sourcecloud = pointcloud;
    mapcloud = pointcloud;
    targetcloud = pointcloud;
//    tmpcloud = pointcloud;
    first_depth_addr = Config::get<std::string> ("first_depth_addr");
  }

  void addTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr targetpointcloud); //添加目标点云
  void addSourceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sourcepointcloud); //添加源点云

  pcl::PointCloud<pcl::PointXYZ>::Ptr iterativeClosestPoint(); //迭代最近点
  
public:
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr mapcloud;//点云地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr sourcecloud;  //源点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetcloud;  // 目标点云
//  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud; // 中间变量，暂时存储点云用
  
  pcl::PointCloud<pcl::PointXYZ> final; //icp 算法中用来存储坐标变换后的点云
  Eigen::Matrix<float, 4, 4> trans; //两点云之间的变换坐标
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;//icp 智能指针
 
  
  std::string first_depth_addr ;
  
//  bool check_outofrange(Eigen::Isometry range);
  

};

