#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "./camera.h"
#include "./common_include.h"

namespace myslam
{
    // forward declare
    struct MapPoint;
    struct Feature;

    /*
    每一帧都有一个ID
    关键帧，有关键帧的ID
    */
   struct Frame
   {
       public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
       typedef std::shared_ptr<Frame> Ptr;

       unsigned long id_  = 0;
       unsigned  long keyframe_id_ = 0;
       bool is_keyframe_ = false;
       double time_stamp_; 
       SE3 pose_;
       std::mutex pose_mutex_;              //Pose数据锁
       cv::Mat left_img_, right_img_;

       //extracted features in left image
       std::vector<std::shared_ptr<Feature>> feature_left_;

       //corresponding feature in right image, 
       //set to unllptr , if no corresponding
       std::vector<std::shared_ptr<Feature>> feature_right_;

        // data members
       public:
           Frame() {}

           Frame(long id, double time_stamp, const SE3 &pose,
                 const Mat &left, const Mat &right);
           
           //set and get pose , threah safe
           SE3 Pose()
           {
               std::unique_lock<std::mutex> lck(pose_mutex_);
               return pose_;
           }

           void SetPose(const SE3 &pose)
           {
               std::unique_lock<std::mutex> lck(pose_mutex_);
               pose_ = pose;
           }

           ///设置关键帧，并分配关键帧的ID
           void SetKeyFrame();

           ///工厂构建模式，分配ID
           static std::shared_ptr<Frame> CreateFrame();
           
   };
   
} // namespace myslam

#endif //MYSLAM_FRAME_H