#ifndef MYSLMA_BACKEND_H
#define MYSLMA_BACKEND_H

#include "./common_include.h"
#include "frame.h"
#include "map.h"

namespace myslam
{
    /**
     * 后端
     * 单独的优化线程，在MAp更新启动优化
     * Map更新，由前端触发
     **/
    class Backend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;
        ///构造函数中，启动优化线程，并且挂起
        Backend();

        //设置左右目的相机，用于获取内外参数
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            cam_left = left;
            cam_right = right;
        }

        ///设置地图
        void SetMap(std::shared_ptr<Map> map)
        {
            map_ = map;
        }

        ///触发地图更新，启动优化
        void updateMap();

        ///关闭后盾线程
        void Stop();

    private:
        ///后端线程
        void BackendLoop();

        ///对给定关键帧和路标点进行优化
        void Optimize(Map::KeyframeType &keyframe, Map::LandmarksType &landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;
        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;
        Camera::Ptr cam_left = nullptr, cam_right = nullptr;
    };

} // namespace myslam

#endif //MYSLMA_BACKEND_H