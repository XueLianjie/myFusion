#include "config.h"

void Config::setParameterFile()
{
  if(config_ == nullptr)
  {
    config_ = shared_ptr<Config>(new Config);
  }
  config_ -> file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if(config_ -> file_.isOpened() == false)
  {
    std::cerr << "parametmete file " << filename << "does not exist." << std::endl;
    config_->file_.release();
    return;
  }
  
}

Config::~Config()
{
  if(file_.isOpened())
  file_.release();
  
  
}

shared_ptr<Config> Config::config_ = nullptr;
