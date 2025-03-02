//
// Created by 이찬근 on 2024. 8. 1..
//

#ifndef LIDARCALIBRATION_H
#define LIDARCALIBRATION_H

#include "3D_Lidarfunction.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>

#define LIDAR_CONSTANT 20000


////change lidar data to camera cordinate
void lidar2uvcoord(cv::Mat& uvmat, cv::Mat& xyzmat, PointCloud data[][VELODYNE_CHANNELS], const cv::Mat& cam_rotate);

//// return x,y,z when input u,v coordinate
int uv2realcoord(cv::Mat &uvmat, int interest_points[], double tol);

//// visualize code
int uv2real_visualize(cv::Mat& frame, cv::Mat &uvmat);




#endif //LIDARCALIBRATION_H
