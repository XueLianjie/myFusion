#include "common.h"
//namespace Fusion{
#pragma once  
  struct Camera
  {
    double cx, cy, fx, fy, depthScale;
            
  };
    
  class Frame
  {
  public:
    Frame();
    Frame(Camera camera_, cv::Mat frame_);
    ~Frame(){}
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud();
       
  private:
    Camera camera;
    cv::Mat frame;
    pcl::PassThrough<pcl::PointXYZ> pass; //直通滤波器,截取点云区域
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//统计离群点滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel; //网格滤波器，调整点云地图分辨率
    int MeanK;//统计滤波范围点
    double zmin,zmax,voxel_grid;//直通滤波器z方向区域,网格滤波分辨率
    double outlier_threshold;//离群点阈值
  };

