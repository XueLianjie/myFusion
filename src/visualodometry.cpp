#include "visualodometry.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr VisualOdometry::VO(pcl::PointCloud<pcl::PointXYZ>::Ptr out_put)
{
  for(int i = start_indext; i <= end_index; i = i + iterative_interval)
  { 
    std::stringstream ss;
    ss << i+1;
    std::string filename;
    ss >> filename;
    cv::Mat depth = cv::imread("./depth_png/" + filename + ".png", -1);
    Frame frame(new_camera, depth);
    map_cloud.addTargetCloud(frame.toPointCloud());
    out_put = map_cloud.iterativeClosestPoint();
//    if(show_cloud)
//    viewer.showCloud(out_put);

  }
      return out_put;
}