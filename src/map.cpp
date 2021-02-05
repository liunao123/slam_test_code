//cpp file for Map.h

#include "../include/myslam/map.h"
#include "../include/myslam/feature.h"

namespace myslam
{
    //增加一个关键帧
    void Map::InsertKeyFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;

        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end())
        {
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        } 
        else 
        {
            keyframes_[frame->keyframe_id_] = frame; // ????相当于什么都没做，这一步是多余的吧
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        if (active_keyframes_.size() > num_active_keyframe_)
        {
            RemoveOldKeyframe;
        }        
    }

    //增加一个地图--顶点
    void Map::InsertMapPoint(MapPoint::Ptr map_point)
    {
        if (landmarks_.find(map_point->id_) == landmarks_.end())
        {
            landmarks_.insert(make_pair(map_point->id_, map_point));
            active_landmarks_.insert(make_pair(map_point->id_, map_point));
        }
        else
        {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] =  map_point;
        }                
    }

    void Map::RemoveOldKeyframe()
    {
        if (current_frame_ == nullptr ) return;        
        //寻找与当前帧最近、与、 最远的两个关键帧
        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->pose_.inverse();
        
        for (auto & kf: active_keyframes_)
        {
            if (kf.second == current_frame_ ) continue;

            auto dis = (kf.second->Pose() * Twc).log().norm();
            if (dis >  max_dis)
            {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis <  min_dis)
            {
                min_dis = dis;
                min_kf_id = kf.first;
            }            
        }

        // 最近的阈值
        const double min_dis_th = 0.2; 
        Frame::Ptr frame_to_remove = nullptr;
        if (min_dis < min_dis_th)
        {// 删掉最近的帧
            frame_to_remove = keyframes_.at(min_kf_id);
        }
        else
        {   // 删掉最远的帧
            frame_to_remove = keyframes_.at(max_kf_id);
        }
        LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
        
        //remove keyframe and landmarks observation
        active_keyframes_.erase(frame_to_remove->keyframe_id_);

        for (auto feat : frame_to_remove->feature_left_)
        {
            auto mp = feat -> map_point_.lock();
            if (mp)
            {
                mp -> RemoveObservation(feat);
            }            
        }
        for (auto feat : frame_to_remove->feature_right_)
        {
            if (feat == nullptr) continue;
            auto mp = feat -> map_point_.lock();
            if (mp)
            {
                mp->RemoveObservation(feat);
            }            
        }
        Cleanmap();
    }

    void Map::Cleanmap()
    {
        int cnt_landmark_remove = 0;
        //-----------------------------------------x写法独特------------------
        for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end(); )
        {
            if (iter->second->observed_times_ == 0)
            {
                iter = active_landmarks_.erase(iter);
                cnt_landmark_remove++;
            }
            else
            {
                ++iter;
            }
        }
        LOG(INFO) << "Remove --- " << cnt_landmark_remove << " active landmarks" ;
        
    }
}

//namespace myslam