#pragma once
#include "common.h"
#include "config.h"
#include "frame.h"
#include "mapcloud.h"

class VisualOdometry
{
//  VisualOdometry();
public:
  VisualOdometry()
  {
    
    start_indext = Config::get<int> ("start_indext");//读取的图像开始
    end_index = Config::get<int> ("end_index");//读取图像的结束
    new_camera.cx = Config::get<float> ("camera.cx");// 325.5;
    new_camera.cy = Config::get<float> ("camera.cy");//253.5;
    new_camera.fx = Config::get<float> ("camera.fx");//518.0;
    new_camera.fy = Config::get<float> ("camera.fy");//519.0;
    new_camera.depthScale = Config::get<float> ("camera.scale");//1000.0;//尺度变换，将mm -> m
    //map_cloud 
    //frame = Frame(new_camera,depth);
//    map_cloud.gridsize = Config::get<double>("grid_size");
    map_cloud = MapCloud(frame.toPointCloud());
    /*pointcloud = MapCloud::gridfilter(map_cloud.gridsize, frame.toPointCloud());
    map_cloud.sourcecloud = pointcloud;
    map_cloud.mapcloud = pointcloud;
    map_cloud.targetcloud = pointcloud;
    map_cloud.tmpcloud = pointcloud;
    */
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr VO(pcl::PointCloud<pcl::PointXYZ>::Ptr out_put);
  
public:
  int iterative_interval = Config::get<int>("iterative_interval");//icp 迭代间隔帧数
  int start_indext, end_index;
  Camera new_camera; //相机 参数
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
  cv::Mat depth = cv::imread("./depth_png/1.png", -1);  
  Frame frame;//(new_camera,depth);
  MapCloud map_cloud;//(frame.toPointCloud());
  int show_cloud = Config::get<int> ("show_cloud");
//  pcl::visualization::CloudViewer viewer("viewer");
  
};