#pragma once
#include "common.h"
//using namespace std;
class Config
{
private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
  Config(){}; //私有构造函数无法初始化，可以形成单例模式singleton
public:
  ~Config();
  
  //set a new config file
  static void setParameterFile(const std::string &filename);//静态函数
  
  //access the parameter values
  template <typename T>
  static T get(const std::string &key)//静态成员函数， 不能通过对象调用静态成员函数；  由于静态成员函数不与特定的对象关联， 因此只能使用静态数据成员。 
  {
    return T(Config::config_->file_[key]);
  }
  
};