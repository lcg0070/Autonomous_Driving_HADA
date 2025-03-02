//
// Created by User on 2024-08-05.
//

#include <windows.h>
#include "Velodyne.h"
#include "3D_Lidarfunction.hpp"
#include "SetLidarParam.h"
#include "LiDAR_coordchange.h"
#include <iostream>




static cv::Mat cam_rotate;

static PointCloud points[VELODYNE_DATA_SIZE][VELODYNE_CHANNELS];


//#define DATA_SIZE       (size_t)(360.0 / VELODYNE_RESOL)

int init_velodyne(){
    // 3x4 matrix for camera coeff and lidar position
    // xyz is m scale
    cv::Mat cameraMatrix, distCoeffs;
    cam_rotate = lidarCalibration(cameraMatrix, distCoeffs);

    std::cout<<cam_rotate<<std::endl;
    init_Velodyne();
    Sleep(3000);
    return 0;
}

cv::Mat velodyne_processing(int* interested_source, int tol, cv::Mat& uv_mat) {

    get_point_cloud(points);

    cv::Mat xyzmat, uvmat;

    lidar2uvcoord(uvmat, xyzmat, points, cam_rotate);

    int idx = uv2realcoord(uvmat, interested_source, tol);

    if(idx == -1) return cv::Mat::zeros(1,1,CV_64F);;

    uv_mat = uvmat.col(idx);
    return xyzmat.col(idx);
}

int velodyne_visual_processing(cv::Mat& frame) {

    get_point_cloud(points);

    cv::Mat xyzmat, uvmat;

    lidar2uvcoord(uvmat, xyzmat, points, cam_rotate);

    if (uv2real_visualize(frame, uvmat)) return 1;

    return 0;
}


int clear_velodyne(){

    clear_Velodyne();
    return 0;
}