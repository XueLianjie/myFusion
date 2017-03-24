#pragma once
#include "common.h"

class VisualOdometry
{
  VisualOdometry();
  VisualOdometry()
  {
    
    start_indext = Config::get<int> ("start_indext");
    end_index = Config::get<int> ("end_index");
    new_camera.cx = Config::get<float> ("camera.cx");// 325.5;
    new_camera.cy = Config::get<float> ("camera.cy");//253.5;
    new_camera.fx = Config::get<float> ("camera.fx");//518.0;
    new_camera.fy = Config::get<float> ("camera.fy");//519.0;
    new_camera.depthScale = Config::get<float> ("camera.scale");//1000.0;//尺度变换，将mm -> m
  }
  
public:
  int iterative_interval = Config::get<int>("iterative_interval");//icp 迭代间隔帧数
  int start_indext, end_index;
  Camera new_camera; //相机 参数
  MapCloud map_cloud;
    
};