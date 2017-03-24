#include "frame.h"
#include "common.h"
#include "mapcloud.h"
#include "config.h"
#include "visualodometry.h"
//namespace Fusion{

int main(int argc, char** argv)
{
  
  if ( argc != 2 )
  {
    cout<<"usage: run_vo parameter_file"<<endl;
    return 1;
  }
  Config::setParameterFile ( argv[1] );
  /*
  VisualOdometry vo;
  pcl::PointCloud<pcl::PointXYZ>::Ptr output;
  vo.VO(output);
  
  */
  
  Camera new_camera; //相机 参数
  new_camera.cx = Config::get<float> ("camera.cx");// 325.5;
  new_camera.cy = Config::get<float> ("camera.cy");//253.5;
  new_camera.fx = Config::get<float> ("camera.fx");//518.0;
  new_camera.fy = Config::get<float> ("camera.fy");//519.0;
  new_camera.depthScale = Config::get<float> ("camera.scale");//1000.0;//尺度变换，将mm -> m
  
  
  
  cv::Mat depth = cv::imread("./depth_png/1.png", -1);
  Frame frame(new_camera, depth);
  MapCloud map_cloud(frame.toPointCloud());
  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = frame.toPointCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr output;// = pointcloud; // (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::CloudViewer viewer("viewer");
  
  int iterative_interval = Config::get<int>("iterative_interval");//icp 迭代间隔帧数
  int start_indext, end_index;
  start_indext = Config::get<int> ("start_indext");
  end_index = Config::get<int> ("end_index");
  
  
  for(int i = start_indext; i <= end_index; i = i + iterative_interval)
  { 
    std::stringstream ss;
    ss << i+1;
    std::string filename;
    ss >> filename;
    cv::Mat depth = cv::imread("./depth_png/" + filename + ".png", -1);
    Frame frame(new_camera, depth);
    map_cloud.addTargetCloud(frame.toPointCloud());
    output = map_cloud.iterativeClosestPoint();
    viewer.showCloud(output);
    
  }
  
  pcl::io::savePCDFile( "mapcloud2.pcd", *output );
  
  return 0;
  
}
