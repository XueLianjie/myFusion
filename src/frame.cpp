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
    zmin = Config::get<double> ("z_min");
    zmax = Config::get<double> ("z_max");
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::toPointCloud()
  {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p;
    
    //根据相机模型逐点进行像素点到三维空间坐标的转换
    for(int m = 0; m < frame.rows; m++)
    for(int n = 0; n < frame.cols; n++)
    {
      ushort d = frame.ptr<ushort>(m)[n];
      //cout << d << endl;
      if(d == 0)
	continue;
      
      p.z = - double(d) / camera.depthScale;
      p.x = - (n - camera.cx) * p.z / camera.fx;
      p.y = (m - camera.cy) * p.z / camera.fy;
      cloud_p->points.push_back(p);
      
    }  
    cloud_p->height = 1;
    cloud_p->width = cloud_p->points.size();
    cloud_p->is_dense = false;
    cloud_p->points.resize(cloud_p->width * cloud_p->height);
    
    //
    
    
    //进行滤波
    //直通滤波器，去除较远处的点，设置z 方向的显示范围 
    pass.setInputCloud(cloud_p);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zmin, zmax);//由于前面为了正确显示点云，对p.z取反，此处对z坐标的范围设置为（-3.0, 0.0），此数值可根据需要更改。
    pass.filter(*cloud_p);

    
    return cloud_p;
    
  }
 