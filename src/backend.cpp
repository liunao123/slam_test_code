
#include "../include/myslam/backend.h"
#include "../include/myslam/algorithm.h"
#include "../include/myslam/feature.h"
#include "../include/myslam/g2o_types.h"
#include "../include/myslam/map.h"
#include "../include/myslam/mappoint.h"

namespace myslam
{
    Backend::Backend()
    {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }
    
    void Backend::updateMap()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void Backend::Stop()
    {
        backend_running_.store(false);
        map_update_.notify_one();

        backend_thread_.join();

    }

    void Backend::BackendLoop()
    {
        while (backend_running_.load())
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            ///后端仅仅优化，激活的frame和landmarks
            Map::KeyframeType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            Optimize(active_kfs, active_landmarks);
        }
    }

    void Backend::Optimize(Map::KeyframeType &keyframes,
                           Map::LandmarksType &landmarks)
    {
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // pose顶点 使用Keyfram id
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;
            // camera vertex pose
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);
            if (kf->keyframe_id_ > max_kf_id_ )
            {
                max_kf_id_  = kf->keyframe_id_;
            }
            
            vertices.insert(kf->keyframe_id_, vertex_pose );
        }

        //路标顶点，使用路标id索引
        

    }








} // namespace myslam
