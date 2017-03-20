#include "frame.h"

  Frame::Frame()
  {
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    camera.depthScale = 1000.0;  
    
  }
  
  Frame::Frame(Camera camera_, cv::Mat frame_)
  {
    camera.cx = camera_.cx;
    camera.cy = camera_.cy;
    camera.fx = camera_.fx;
    camera.fy = camera_.fy;
    camera.depthScale = camera_.depthScale;
    frame = frame_;
    
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::toPointCloud()
  {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p;
    
    for(int m = 0; m < frame.rows; m++)
    for(int n = 0; n < frame.cols; n++)
    {
      ushort d = frame.ptr<ushort>(m)[n];
      //cout << d << endl;
      if(d == 0)
	continue;
      
      p.z = - double(d) / camera.depthScale;
      p.x = (n - camera.cx) * p.z / camera.fx;
      p.y = (m - camera.cy) * p.z / camera.fy;
      cloud_p->points.push_back(p);
      
    }  
    cloud_p->height = 1;
    cloud_p->width = cloud_p->points.size();
    cloud_p->is_dense = false;
    cloud_p->points.resize(cloud_p->width * cloud_p->height);
    
    return cloud_p;
    
  }
 