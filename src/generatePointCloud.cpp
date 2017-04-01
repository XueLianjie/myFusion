#include "frame.h"
#include "common.h"
#include "mapcloud.h"
#include "config.h"
#include "visualodometry.h"
//namespace Fusion{
//using namespace std;

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
  int start_indext, end_index, currIndex, lastIndex;
  start_indext = Config::get<int> ("start_indext");
  currIndex = start_indext;
  end_index = Config::get<int> ("end_index");
  
  //加入g2o
  //g2o初始化
      // 选择优化方法
  typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
  typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
  g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>* linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  g2o::SparseOptimizer globalOptimizer; //最终用到的求解器
  globalOptimizer.setAlgorithm(solver);
  globalOptimizer.setVerbose(false);//不输出调试信息
  
  //向globalOptimizer增加第一个顶点
  g2o::VertexSE3* v = new g2o::VertexSE3();//此顶点在g2o库里已经进行了定义，
  v->setId(start_indext);
  v->setEstimate(Eigen::Isometry3d::Identity());
  v->setFixed(true);//第一个顶点固定，不做优化
  globalOptimizer.addVertex(v);
  lastIndex = currIndex;
  
  
  //BlockSolver  
  
  for(int i = start_indext; i <= end_index; i = i + iterative_interval)//每iterative_interval 个图片进行一次位姿估计
  { 
    std::stringstream ss;
    ss << i+1;
    std::string filename;
    ss >> filename;
    cv::Mat depth = cv::imread("./depth_png/" + filename + ".png", -1);
    Frame frame(new_camera, depth);
    map_cloud.addTargetCloud(frame.toPointCloud());
    output = map_cloud.iterativeClosestPoint();
   // viewer.showCloud(output);
    
    //向g2o中增加此顶点与上一帧联系的边
    //顶点部分
    //顶点设置id
    currIndex ++;
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(currIndex);
    v->setEstimate(Eigen::Isometry3d::Identity());
    globalOptimizer.addVertex(v);
    
    //边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    //连接此边的两个顶点id
    edge->vertices()[0] = globalOptimizer.vertex(lastIndex);
    edge->vertices()[1] = globalOptimizer.vertex(currIndex);
    //信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double,6,6>::Identity();
    //信息矩阵是协方差的逆，表示我们对边的精度的预先估计
    //因为pose 为 6D 的，信息矩阵是6*6的矩阵， 假设位置和角度的估计精度均为0.1 且相互独立
    //那么协方差则为对角为0.01的矩阵， 信息矩阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    //也可以将角度设大一些， 表示对角度的估计更加准确
    edge->setInformation(information);
    //边的估计即是ICP迭代的结果
    //Eigen::Matrix<float, 4, 4> trans
    //edge->setMeasurement(Eigen::Isometry3d T);
    Eigen::Matrix<double,4,4> translate;//trans的double形式
    //Eigen::Vector3d rotate_matrix = translate(3,3);
    //Eigen::Vector3d pretranslate_vec = translate();
    
    translate = map_cloud.trans.cast<double>();
    
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.matrix() = translate;
    //T.rotate(map_cloud.);
    //T.pretranslate();
    
    edge->setMeasurement(T);
    //将此边加入图中
    globalOptimizer.addEdge(edge);
    
    //lastFrame = currFrame;
    lastIndex = currIndex;
    
    
  }
  
      
    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./data/result_after.g2o" );
    cout<<"Optimization done."<<endl;
    
    //拼接优化后的点云
    cout << "saving pointcloud map..." << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
    
        currIndex = 1;
	  pcl::VoxelGrid<pcl::PointXYZ> voxel; //网格滤波器，调整点云地图分辨率
    //从g2o中取出点云
   for(int i = start_indext; i <= end_index; i = i + iterative_interval)//每iterative_interval 个图片进行一次位姿估计
  { 

    
    std::stringstream ss;
    ss << i;//第一张图
    std::string filename;
    ss >> filename;
    cv::Mat depth = cv::imread("./depth_png/" + filename + ".png", -1);
    Frame frame(new_camera, depth);
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud = frame.toPointCloud();
    //map_cloud.addTargetCloud(frame.toPointCloud());
    //output = map_cloud.iterativeClosestPoint();
    g2o::VertexSE3* vertexp = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(currIndex));
    currIndex ++;
    Eigen::Isometry3d pose = vertexp->estimate();//该帧优化后的位姿
    cout << "saving pointcloud" << endl;
  voxel.setLeafSize( 0.01, 0.01, 0.01 );//设置网格滤波器的分辨率
  voxel.setInputCloud( newCloud );//加入待滤波的地图点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud ( new pcl::PointCloud<pcl::PointXYZ> );//此处必须为新的空指针，否则程序停止，智能指针不需要释放。
  voxel.filter( *map_cloud );//滤波后的点云存储到map_cloud中
 // newCloud = map_cloud;
    
    //拼接点云，以最新的相机坐标系为世界坐标系
    pcl::transformPointCloud(*map_cloud, *tmp, pose.matrix());//对地图点云做trans变换
    *output += *tmp;//拼接点云
    //output = tmp;
    
    tmp->clear();
    newCloud->clear();    
      
    }
        cout << "saving pointcloud" << endl;

    globalOptimizer.clear();
  
  pcl::io::savePCDFile( "g2omap.pcd", *output );
  
  return 0;
  
}
