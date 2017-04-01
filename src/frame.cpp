#include "frame.h"
#include "config.h"

  Frame::Frame()//无参构造函数
  { 
    //相机参数
    camera.cx = Config::get<float> ("camera.cx");
    camera.cy = Config::get<float> ("camera.cy");
    camera.fx = Config::get<float> ("camera.fx");
    camera.fy = Config::get<float> ("camera.fy");
    camera.depthScale = Config::get<float> ("camera.scale");//尺度变换，将mm -> m
    
    frame = cv::imread("./depth_png/1.png", -1); //无参数时默认读取该位置一张图片
    
    zmin = Config::get<double> ("z_min");
    zmax = Config::get<double> ("z_max");
    voxel_grid = Config::get<double> ("grid_size");
    MeanK = Config::get<int> ("MeanK");
    outlier_threshold = Config::get<double> ("outlier_threshold");
    
  }
  
  Frame::Frame(Camera camera_, cv::Mat frame_)//有参构造函数
  {
    //相机内参
    camera.cx = camera_.cx;
    camera.cy = camera_.cy;
    camera.fx = camera_.fx;
    camera.fy = camera_.fy;
    camera.depthScale = camera_.depthScale;
    
    frame = frame_;
    
    zmin = Config::get<double> ("z_min");
    zmax = Config::get<double> ("z_max");
    voxel_grid = Config::get<double> ("grid_size");
    MeanK = Config::get<int> ("MeanK");
    outlier_threshold = Config::get<double> ("outlier_threshold");
    
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::toPointCloud()
  {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);//待转化的3D点云指针
    pcl::PointXYZ p;//像素点对应的3D点
    
    //根据相机模型逐点进行像素点到三维空间坐标的转换
    for(int m = 0; m < frame.rows; m++)
    for(int n = 0; n < frame.cols; n++)
    {
      ushort d = frame.ptr<ushort>(m)[n];
      if(d == 0)//对于不存在的点应舍去
	continue;
      //根据相机内参计算点云在相机坐标系下的空间位置
      p.z = - double(d) / camera.depthScale;//转换成视野里正对着的视角，z方向取反，同时注意直通滤波器的范围也要做相应的改变。
      p.x = - (n - camera.cx) * p.z / camera.fx;//x方向取反
      p.y = (m - camera.cy) * p.z / camera.fy;
      cloud_p->points.push_back(p);
      
    }  
    cloud_p->height = 1;
    cloud_p->width = cloud_p->points.size();
    cloud_p->is_dense = false;
    cloud_p->points.resize(cloud_p->width * cloud_p->height);
    
    //进行滤波
    //直通滤波器，去除较远处的点，设置z 方向的显示范围 
    pass.setInputCloud(cloud_p);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zmin, zmax);//由于前面为了正确显示点云，对p.z取反，此处对z坐标的范围设置为（-3.0, 0.0），此数值可根据需要更改。
    pass.filter(*cloud_p);
    
    //滤波器移除离群点
    sor.setInputCloud(cloud_p);//设置输入点云
    sor.setMeanK(MeanK);//设置进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(outlier_threshold);//设置判断是否为离群点的阈值
    sor.filter(*cloud_p);//移除离群点滤波
    
    //进行网格滤波降采样
    voxel.setLeafSize( 0.01, 0.01, 0.01 );//设置网格滤波器的分辨率
    voxel.setInputCloud(cloud_p);
    voxel.filter( *cloud_p );//滤波后的点云存储到cloud_p中
    
    
    return cloud_p;//返回转换好的点云指针
    
  }
 