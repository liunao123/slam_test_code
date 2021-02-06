#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "common_include.h"
#include "camera.h"
#include "frame.h"

namespace myslam
{
    /**
     * 数据集读取
     * 构造时传入配置文件的路径，
     * 配置文件的dataset_dir，为路径
     * Init之后可获得相机和下一幅图像
     **/
    class Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef std::shared_ptr<Dataset> Ptr;
        Dataset(const std::string &dataset_dir);

        ///初始化，返回是否成功
        bool Init();

        ///creat and return the next frame containing the stereo images
        /// this is a function, 返回值是Frame的指针
        Frame::Ptr Nextframe();

        ///get camera bi id
        Camera::Ptr GetCamera(int camera_id) const
        {
            return camera_.at(camera_id);
        }

        private:
            std::string dataset_path_;
            int current_image_index_ = 0;
            std::vector<Camera::Ptr> camera_;

    };
    
}





#endif //MYSLAM_DATASET_H