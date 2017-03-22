#include "frame.h"
#include "common.h"
#include "mapcloud.h"
//namespace Fusion{

int main(int argc, char** argv)
{
  Camera new_camera; //相机 参数
  new_camera.cx = 325.5;
  new_camera.cy = 253.5;
  new_camera.fx = 518.0;
  new_camera.fy = 519.0;
  new_camera.depthScale = 1000.0;//尺度变换，将mm -> m.
  
  cv::Mat depth = cv::imread("./depth_png/1.png", -1);
  Frame frame(new_camera, depth);
  MapCloud map_cloud(frame.toPointCloud());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = frame.toPointCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr output = pointcloud; // (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::CloudViewer viewer("viewer");

  for(int i = 1; i < 70; i=i+3)
  { 
    std::stringstream ss;
    ss<<i+1;
    std::string filename;
    ss>>filename;
    cv::Mat depth = cv::imread("./depth_png/" + filename + ".png", -1);
    Frame frame(new_camera, depth);
    map_cloud.addTargetCloud(frame.toPointCloud());
    output = map_cloud.iterativeClosestPoint();
    viewer.showCloud(output);
    
  }
  pcl::io::savePCDFile( "mapcloud1.pcd", *output );
  
  
  return 0;
  
}
