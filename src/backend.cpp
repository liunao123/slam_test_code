
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
            if (kf->keyframe_id_ > max_kf_id_)
            {
                max_kf_id_ = kf->keyframe_id_;
            }

            vertices.insert(kf->keyframe_id_, vertex_pose);
        }

        //路标顶点，使用路标id索引
        std::map<unsignedv long, VertexXYZ *> vertices_landmarks;

        //  K and 左右外参
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        //edges
        int index = 1;
        double chi2_th = 5.991; //robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_end_features;

        for (auto &landmark : landmarks)
        {
            if (landmark.second->is_outlier_)
                continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations)
            {
                if (obs.lock() == nullptr)
                    continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                    continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_)
                    edge = new EdgeProjection(K, left_ext);
                else
                    edge = new EdgeProjection(K, right_ext);

                //如果landmark还没有被加入优化，则新加一个顶点
                if (vertics_landmarks.find(landmark_id) == vertics_landmarks.end())
                {
                    //********************************//
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }
                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));
                edge->setVertex(1, vertices_landmarks.at(landmark_id));
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;
            }
        }

        // do optimize and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5)
        {
            cnt_inlier = 0;
            cnt_outlier = 0;
            //deter if we want to adjust the outlier threshold
            for (auto &ef : edges_end_features)
            {
                if (ef.first->chi2() > chi2_th)
                    cnt_outlier++;
                else
                    cnt_inlier++;
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5)
                break;
            else
            {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_end_features)
        {
            if (ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                //remove this observation
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            }
            else
                ef.second->is_outlier_ = false;
        }
        
        LOG(INFO) << "outlier/inlier in optimization: " << cnt_outlier 
                << " / " << cnt_inlier;
        //Set pose and landmark postion 
        for (auto &v : vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        
        for (auto &v : vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }

} // namespace myslam
