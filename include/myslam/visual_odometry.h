#pragma once

#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "backend.h"
#include "common_include.h"
#include "dataset.h"
#include "frontend.h"
#include "viewer.h"


namespace myslam
{
    /**
     *  VO 对外的接口
     **/
    class VisualOdometry
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<VisualOdometry> Ptr;

            ///constructor with config file
            VisualOdometry(std::string &config_path);

            /**
             * do initialization thing before run
             * @return true if ok            
             **/
            bool Init();

            bool Run();

            /**
             * start VO in the dataset
             **/
            bool Step();

            ///获取前端
            FrontendStatus GetFrontendStatus() const 
            { return frontend_->GetStatus(); }

            private:
                bool inited_ = false;

                std::string config_file_path_ = nullptr;

                Frontend::Ptr frontend_ = nullptr;
                Backend::Ptr backend_ = nullptr;
                Map::Ptr map_ = nullptr;
                Viewer::Ptr viewer_ = nullptr;

            // dataset
            Dataset::Ptr dataset_ = nullptr;
    };
    
} // namespace myslam

#endif // MYSLAM_VISUAL_ODOMETRY_H