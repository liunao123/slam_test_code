#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace myslam
{
    /**
     * 可视化部分
     **/
    class Viewer
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Viewer> Ptr;

            Viewer();

            void SetMap(Map::Ptr map) {map_ =  map;}
            void Close();

            ///增加一个当前帧
            void AddCurrentFrame(Frame::Ptr current_frame);

            ///更新地图
            void UpdateMap();

            

        private:
            void ThreadLoop();
            void DrawFrame(Frame::Ptr frame, const float* color);
            void DrawMapPoint();
            void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

            ///plot the features in the current frame into an image
            cv::Mat PlotFrameImage();
            
            Frame::Ptr current_frame_ = nullptr;
            Map::Ptr map_;

            std::thread viewer_thread_;
            bool viewer_running_  = true;

            std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
            std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
            bool map_updated_ = true;

            std::mutex viewer_data_mutex_;
    };

} // namespace myslam

#endif //MYSLAM_VIEWER_H