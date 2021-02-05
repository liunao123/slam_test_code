#pragma once

#ifndef MAP_H
#define MAP_H

#include "./common_include.h"
#include "frame.h"
#include "mappoint.h"

namespace myslam
{
    /**
     * 和地图进行交互，前段调用InsertKeyframe和InsertmapPoint插入新帧和地图点
     * 后端维护地图的结构，判定outlier等，进行删除操作
     * */
    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframeType;

        Map() {}

        //增加一个关键帧
        void InsertKeyFrame(Frame::Ptr frame);
        //增加一个地图顶点
        void InsertMapPoint(MapPoint::Ptr map_point);

        //获取所有的地图点
        LandmarksType GetAllMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        //获取激活地图点
        LandmarksType GetActiveMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }
        //获取所有的关键帧
        KeyframeType GetAllKeyframes()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }
        //获取激活地图点
        KeyframeType GetActivekeyframes()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        //清理map中，观测数量为0 的点
        void Cleanmap();

        private:
        //将旧的 关键帧，设置为不活跃状态
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;           //所有的路标
        LandmarksType active_landmarks_;   //活跃的

        KeyframeType keyframes_; //关键帧
        KeyframeType active_keyframes_; //活跃的关键帧

        Frame::Ptr current_frame_ = nullptr;

        //setting
        //激活关键帧的数量
        int num_active_keyframe_ = 7; 

    };

} // namespace myslam

#endif //MAP_H
