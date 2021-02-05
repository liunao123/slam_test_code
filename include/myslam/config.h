#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "common_include.h"
namespace myslam
{
    /**
     *配置类，使用SetParameterFile 确定配置文件
     * 使用Get得到对应的参数，
     * 单例模式
     * */
    class Config
    {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {}  // private constructor makes singleton

    public:
        ~Config(); //close the file when deconstructing

        // set a new config file
        static bool SetParameterFile(const std::string &filename);

        //access the parameterfile to get value
        template <typename T>
        static T Get(const std::string &key)
        {
            return T(Config::config_->file_[key]);
        }
    };
    
} //namespace

#endif //MYSLAM_CONFIG_H