//
// Created by User on 2024-08-05.
//

#ifndef INC_3D_LIDAR_3D_LIDARFUNCTION_HPP
#define INC_3D_LIDAR_3D_LIDARFUNCTION_HPP

#include <opencv2/opencv.hpp>
#include <cmath>

#define RAD2DEG (double) (180. / CV_PI)
#define DEG2RAD (double) (CV_PI / 180.)


int init_velodyne();
cv::Mat velodyne_processing(int* interested_source, int tol, cv::Mat& uv_mat);
int velodyne_visual_processing(cv::Mat& frame);
int clear_velodyne();


#endif //INC_3D_LIDAR_3D_LIDARFUNCTION_HPP
