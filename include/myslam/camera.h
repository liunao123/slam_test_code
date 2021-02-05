#pragma once

#ifndef CAMERA_H
#define CAMERA_H

#include "./common_include.h"

namespace myslam
{
    //Pinhole RGB-D camera model
    class Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;
        // Camera intrinsics
        float fx_, fy_, cx, cy, depth_scale_;    
        Camera();
    };
    
    
    
}




#endif