#include "mapcloud.h"

void MapCloud::addTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr targetpointcloud)//添加目标点云
{
  targetcloud = targetpointcloud;
}

void MapCloud::addSourceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sourcepointcloud)//添加源点云
{
  sourcecloud = sourcepointcloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapCloud::iterativeClosestPoint()//点对点迭代，考虑换成point to line 迭代
{
  //icp 迭代最近点进行位姿估计
  icp.setInputSource(sourcecloud);//设置源点云
  icp.setInputTarget(targetcloud);//设置目标点云
  icp.align(final);//icp 迭代
  
  std::cout << "point cloud size = " << final.points.size() << std::endl;//输出点云点数
//  final.points.clear();//释放final 指向的点云
  
  trans = icp.getFinalTransformation();//icp 求得的变换矩阵
  std::cout << "transformation: " << trans.matrix();
  //判断变换矩阵是否在合理范围内
  
  
  /*
  //拼接点云，以最新的相机坐标系为世界坐标系
  pcl::transformPointCloud(*mapcloud, *tmpcloud, trans);//对地图点云做trans变换
  *tmpcloud += *targetcloud;//拼接点云
  mapcloud = tmpcloud;//更新点云地图
  sourcecloud = targetcloud;//更新源点云
  
  std::cout << "has converged:" << icp.hasConverged() << "\nscore: " << icp.getFitnessScore() << std::endl;
  mapcloud = MapCloud::gridfilter(0.01, mapcloud);
  */
  
  return mapcloud;
  
}

/*//估计运动范围
bool MapCloud::check_outofrange(Eigen::Isometry range)
{
  Eigen::matrix<double,4,4>* T = new Eigen::matrix<double,4,4>;
  *T = range.matrix();
  
  range;
  
}
*/

