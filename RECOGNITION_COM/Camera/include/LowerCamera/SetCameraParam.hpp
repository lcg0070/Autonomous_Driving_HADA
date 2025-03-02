//
// Created by jarry_goon on 24. 8. 1.
//

#ifndef SETCMAERAPARAM_H
#define SETCMAERAPARAM_H

#include <opencv2/opencv.hpp>
#include "CameraLower.hpp"
#include "LaneProcessing.hpp"

inline void set_camera_parameter(cv::Mat& camera_matrix, cv::Mat& dist_coeffs){

    float matrix_ele[9]  = {fx,  0.f, cx,
                            0.f, fy,  cy,
                            0.f, 0.f, 1.f
    };

    float matrix_ele2[4] = {k1,k2,p1,p2};

    cv::Mat(3, 3, CV_32F, matrix_ele).copyTo(camera_matrix);
    cv::Mat(4, 1, CV_32F, matrix_ele2).copyTo(dist_coeffs);
}

inline void set_camera_intrinsic_maxtrix(const cv::Mat& camera_matrix,
                                         const cv::Mat& dist_coeffs,
                                         cv::Mat& map1,
                                         cv::Mat& map2) {
    cv::Size imageSize(FRAME_WIDTH, FRAME_HEIGHT);

    cv::Mat intrinsic_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, imageSize, 1);
    // std::cout << "intrinsic_matrix" << intrinsic_matrix << std::endl;
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), intrinsic_matrix, imageSize, CV_32FC1, map1, map2);
}

inline void set_camera_extrinsic_matrix(cv::Mat& tvec, cv::Mat& rvec){
//
    float matrix_ele[3]  = {X,
                            Y,
                            Z };

    float matrix_ele2[3] = {ROLL,
                            PITCH,
                            YAW };

    cv::Mat(3, 1, CV_32F, matrix_ele).copyTo(tvec);
    cv::Mat(3, 1, CV_32F, matrix_ele2).copyTo(rvec);
}

inline void set_3D_matrix(cv::Mat& mat3D) {

    float matrix_ele[12]  = {-WIDTH * 0.5 , LENGTH ,0.f,
                             WIDTH * 0.5 , LENGTH ,0.f,
                             -WIDTH * 0.5 , YMAX   ,0.f,
                             WIDTH * 0.5 , YMAX   ,0.f};
    cv::Mat(4, 3, CV_32F, matrix_ele).copyTo(mat3D);


    // float matrix_ele[27]  =  {
    //                             -0.0*0.6+0.055, 2.0*0.6+0.066, 0.f,
    //
    //                             -3.0*0.6+0.055, 4.0*0.6+0.066, 0.f,
    //                             -0.0*0.6+0.055, 4.0*0.6+0.066, 0.f,
    //                              3.0*0.6+0.055, 4.0*0.6+0.066, 0.f,
    //                             -5.0*0.6+0.055, 6.0*0.6+0.066, 0.f,
    //                             -3.0*0.6+0.055, 6.0*0.6+0.066, 0.f,
    //                             -0.0*0.6+0.055, 6.0*0.6+0.066, 0.f,
    //                              3.0*0.6+0.055, 6.0*0.6+0.066, 0.f,
    //                              5.0*0.6+0.055, 6.0*0.6+0.066, 0.f};
    // cv::Mat(9, 3, CV_32F, matrix_ele).copyTo(mat3D);
}

void set_BEV_parameter(cv::Mat& M, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, const cv::Mat& tvec, const cv::Mat& rvec, const cv::Mat& mat3D);

#endif //SETCMAERAPARAM_H
