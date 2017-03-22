#pragma once
#include "common.h"
class Config
{
private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
  Config(){}; //私有构造函数无法初始化，可以形成单例模式
public:
  ~Config();
  
  //set a new config file
  static void setParameterFile();
  
  //access the parameter values
  template <typename T>
  static T get()
  {
    return T(Config::config_->file_[key]);
  }
  
};