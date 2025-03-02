//
// Created by User on 2024-08-05.
//

#include "SetLidarParam.h"
#include "3D_Lidarfunction.hpp"

// Camera Intrinsic Matrix
void cameraCalibration(cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    // K Matrix
    double camera_source[9] = {
            FX, 0., CX,
            0., FY, CY,
            0., 0., 1.
    };
    cameraMatrix = cv::Mat(3, 3, CV_64F, camera_source).clone();

    double dist_source[4] = {
            K1, K2, P1, P2
    };
    distCoeffs = cv::Mat(4, 1, CV_64F, dist_source).clone();
}

//// rotation_Translation Matrix ( R | T )
cv::Mat rotation_translation(cv::Mat &rotation) {
    auto *ptr = rotation.ptr<double>(0);
    double data[12];
    for (int i = 0; i < 9; i++) {
        if (i < 3)
            data[i] = ptr[i];
        else if (i < 6)
            data[i + 1] = ptr[i];
        else
            data[i + 2] = ptr[i];
    }
    data[3] = VELODYNE_X;
    data[7] = VELODYNE_Y;
    data[11] = VELODYNE_Z;
    return cv::Mat{3, 4, CV_64F, data}.clone();
}

// lidarcalibration
cv::Mat lidarCalibration(cv::Mat &cameraMatrix, cv::Mat &distcoeffs) {

    cv::Mat rotate = eul2rotmx() * eul2rotmy() * eul2rotmz();
    cv::Mat rotrans = rotation_translation(rotate);

    cameraCalibration(cameraMatrix, distcoeffs);

    return cameraMatrix * rotrans;
}

// Rotation Martix
cv::Mat eul2rotmx() {
    double radian = DEG2RAD * VELODYNE_ROLL;
    double mat_source[9] = {
            1., 0., 0.,
            0., cos(radian), -sin(radian),
            0., sin(radian), cos(radian)
    };
    return cv::Mat(3, 3, CV_64F, mat_source).clone();
}

cv::Mat eul2rotmy() {
    double radian = DEG2RAD * VELODYNE_PITCH;
    double mat_source[9] = {
            cos(radian), 0., -sin(radian),
            0., 1, 0,
            sin(radian), 0, cos(radian)
    };
    return cv::Mat(3, 3, CV_64F, mat_source).clone();
}

cv::Mat eul2rotmz() {
    double radian = DEG2RAD * VELODYNE_YAW;
    double mat_source[9] = {
            cos(radian), -sin(radian), 0.,
            sin(radian), cos(radian), 0.,
            0., 0., 1.
    };
    return cv::Mat(3, 3, CV_64F, mat_source).clone();
}