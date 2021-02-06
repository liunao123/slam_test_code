#include <opencv2/opencv.hpp>


#include "../include/myslam/algorithm.h"
#include "../include/myslam/backend.h"
#include "../include/myslam/common_include.h"
#include "../include/myslam/config.h"
#include "../include/myslam/feature.h"
#include "../include/myslam/frontend.h"
#include "../include/myslam/g2o_types.h"

#include "../include/myslam/map.h"
#include "../include/myslam/viewer.h"

namespace myslam
{
    Frontend::Frontend()
    {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }

    bool Frontend::AddFrame(myslam::Frame::Ptr frame)
    {
        current_frame_ = frame;
        switch (status_)
        {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
        }
        last_frame_ = current_frame_;
        return true;
    }
    
    bool Frnotend::Track()
    {
        if (last_frame_)
        {
            current_frame_ -> SetPose(relative_motion_ * last_frame_->Pose());
        }
        int num_track_last = TrackLastFrame();
        track_inliers_ = EstimateCurrentPose();

        if (tracking_inliers_ > num_features_tracking_)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
        } else if (tracking_inliers_ < num_features_tracking_)
        {
            status_  = FrontendStatus::TRACKING_BAD;
        } else 
        {
            status_ = FrontendStatus::LOST;
        }
        
        InsertKeyframe();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        if (viewer_)
        {
            viewer_ -> AddCurrentFrame(current_frame_);
        }
        return true;

        
        
        
    }


} // namespace myslam





