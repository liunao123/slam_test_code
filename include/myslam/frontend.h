#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace myslam
{
    class Backend;
    class Viewer;

    enum class FrontendStatus
    {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

    /**
     * 前端
     * 估计前端帧的pose，在满足条件时，向地图加入关键帧并进行优化
     **/

    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef std::shared_ptr<Frontend> Ptr;
        Frontend();

        //外部接口
        bool AddFrame(Frame::Ptr frame);

        //  Set函数
        void SetMap(Map::Ptr map) { map_ = map; }

        void SetBackend(std::shared_ptr<Backend> backend)
        {
            backend_ = backend;
        }

        void SetViewer(std::shared_ptr<Viewer> viewer)
        {
            viewer_ = viewer;
        }

        FrontendStatus GetStatus() const { return status_; }

        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        /**
             * track in normal mode
             * @return true if success
             **/
        bool Track();

        /**
             * Reset when lost
             * @return true if success
             **/
        bool Reset();
        /**
             * track with last frame
             * @return num of tracked points
             **/
        int TrackLastFrame();

        /**
             * estimate current frame's pose
             * @return num of inliers
             **/
        int EstimateCurrentPose();

        /**
             * set current frame as a keyframe 
             * and insert it into backend
             * @return true if success
             **/
        bool InsertKeyframe();

        /**
             * try init the frontend with stereo img save in current_frame_
             * @return true if success
             **/
        bool StereoInit();

        /**
             * Detect features in left img in current_frame_
             * keypoint will be saved in current_frame_
             * @return 
             **/
        int DectectFeatures();

        /**
             * Detect features in left img in current_frame_
             * @return num of features found
             **/
        int FindFeaturesInRight();

        /**
             * Build the initial map with single image
             * @return true if ok
             **/
        bool BuildInitMap();

        /**
             * Triangulate the 2D points in current frame
             * @return num  of triangulated points
             **/
        int TriangulateNewPoints();

        /**
             * set the features in kf as new observation of map points
             **/
        void SetObservationsForKeyFrame();

        //data
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr; //当前帧
        Frame::Ptr last_frame_ = nullptr;    //上一帧

        std::shared_ptr<Camera> camera_left_ = nullptr;
        std::shared_ptr<Camera> camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        SE3 relation_metion_; //当前帧与上一帧的相对运动，用于估计当前帧pose初值

        //inliers , used for testing new keyframes
        int tracking_inliers_ = 0;

        //params
        int num_features_ = 200;
        int num_features_init = 100;
        int num_features_tracking = 50;
        int num_features_tracking_bad = 20;
        int num_features_needed_for_keyframe = 80;

        //utilities
        //features detector in opencv
        cv::Ptr<cv::GFTTDetector> gftt_;
        
    };

} // namespace myslam

#endif //MYSLAM_FRONTEND_H