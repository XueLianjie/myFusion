#include "frame.h"
#include "common.h"
//namespace Fusion{
//PCL

int main(int argc, char** argv)
{
  Camera new_camera;
  new_camera.cx = 325.5;
  new_camera.cy = 253.5;
  new_camera.fx = 518.0;
  new_camera.fy = 519.0;
  new_camera.depthScale = 1000.0;
  
//  cv::Mat depth_1, depth_2;
  
//  depth_1 = cv::imread("./depth/1.pgm", -1);
//  depth_2 = cv::imread("./depth/2.pgm", -1);
  
//  Frame fr_1(new_camera, depth_1);
//  Frame fr_2(new_camera, depth_2);
  
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
  
  
  cv::Mat depth = cv::imread("./depth_png/1.png", -1);
  Frame frame(new_camera, depth);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = frame.toPointCloud();
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output = pointcloud; // (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> final;
  Eigen::Matrix<float, 4, 4> trans;
  pcl::visualization::CloudViewer viewer("viewer");
  
  for(int i = 1; i < 170; i=i+5)
  { 
    std::stringstream ss;
    ss<<i+1;
    std::string filename;
    ss>>filename;
    cout << filename;
    //f.rgb = cv::imread( filename );
    cv::Mat depth = cv::imread("./depth_png/" + filename + ".png", -1);
    Frame frame(new_camera, depth);
    tmp = frame.toPointCloud();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pointcloud);
    icp.setInputTarget(tmp);
    icp.align(final);
    trans = icp.getFinalTransformation();
    pointcloud = tmp;
    pcl::transformPointCloud(*output, final, trans);
    final += *tmp;
    *output = final;


    
    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<pcl::PointXYZ> voxel;
    //static ParameterReader pd;
    double gridsize = 0.01; // atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( output );
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapcloud ( new pcl::PointCloud<pcl::PointXYZ> );
    voxel.filter( *mapcloud );
    output = mapcloud;      
    viewer.showCloud(mapcloud);
    
  }
  pcl::io::savePCDFile( "output.pcd", *output );

/*  
  
  cloud_1 = fr_1.toPointCloud();  
  cloud_2 = fr_1.toPointCloud(); 
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_1);
  icp.setInputTarget(cloud_2);
//  pcl::PointCloud<pcl::PointXYZ> final;
  icp.align(final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  final += *cloud_2;
  
*/
  
/*
  std::cout << "point cloud size = " << final.points.size() << std::endl;
  pcl::io::savePCDFile("./final.pcd", final);
  final.points.clear();
  std::cout << "point cloud saved." << std::endl;
  
*/

  //std::cout << icp.getFinalTransformation() * cloud_in->points[1] << std::endl;
  
  return 0;
  
}
