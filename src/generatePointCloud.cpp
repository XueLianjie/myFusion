#include "frame.h"
#include "common.h"
//namespace Fusion{
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

int main(int argc, char** argv)
{
  Camera new_camera;
  new_camera.cx=325.5;
  new_camera.cy=253.5;
  new_camera.fx=518.0;
  new_camera.fy=519.0;
  new_camera.depthScale = 1000.0;
  
  cv::Mat depth_1,depth_2;
  
  depth_1 = cv::imread("./depth/1.pgm", -1);
  depth_2 = cv::imread("./depth/2.pgm", -1);
  
  Frame fr_1(new_camera, depth_1);
  Frame fr_2(new_camera, depth_2);
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
  
  cloud_1 = fr_1.toPointCloud();  
  cloud_2 = fr_1.toPointCloud(); 
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_1);
  icp.setInputTarget(cloud_2);
  pcl::PointCloud<pcl::PointXYZ> final;
  icp.align(final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  
  
  std::cout << "point cloud size = " << cloud_1->points.size() << std::endl;
  pcl::io::savePCDFile("./pointcloud.pcd", *cloud_1);
  cloud_1 -> points.clear();
  std::cout << "point cloud saved." << std::endl;
  
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  //std::cout << icp.getFinalTransformation() * cloud_in->points[1] << std::endl;
  return 0;
  
}
