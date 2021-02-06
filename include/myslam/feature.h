#pragma once
#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "./common_include.h"

namespace myslam
{
    struct Frame;
    struct MapPoint;
    
    /*
    2D特征点，在三角化之后，被关联到一个地图点
    */
   struct Feature
   {
       public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

       typedef std::shared_ptr<Feature> Ptr;

       std::weak_ptr<Frame> frame_;             //持有该feature的frame
       cv::KeyPoint position_;                   //2D 提取的位置
       std::weak_ptr<MapPoint> map_point_;      //关联地图点

       bool is_outlier_ = false;                //
       bool is_on_left_image_ = true;           //表示是否在 left image， false 代表在右图

       public:
       Feature() {}

       Feature(std::shared_ptr<Frame> frame,const cv::KeyPoint &kp)
        :frame_(frame) , position_(kp) {}       
       
   };
   
    
} // namespace myslam



#endif

