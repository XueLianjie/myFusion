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
    
  };

