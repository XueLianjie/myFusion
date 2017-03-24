#pragma once
#include "common.h"
//namespace Fusion{
  
  struct Camera
  {
    double cx, cy, fx, fy, depthScale;
            
  };
    
  class Frame
  {
  public:
    Frame();
    Frame(Camera camera_, cv::Mat frame_);
//    ~Frame();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud();
       
  private:
    Camera camera;
    cv::Mat frame;
    pcl::PassThrough<pcl::PointXYZ> pass; //直通滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//统计离群点滤波器
    
    
    
  };

