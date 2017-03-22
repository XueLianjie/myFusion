#include "common.h" 
class MapCloud
{
public:
  MapCloud() {mapcloud = new pcl::PointCloud<pcl::PointXYZ>;}
//  MapCloud();
  void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::Matrix4 T);
  //void addKeyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, Eigen::Isometry3d T);
  
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr mapcloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud;
  
  
};