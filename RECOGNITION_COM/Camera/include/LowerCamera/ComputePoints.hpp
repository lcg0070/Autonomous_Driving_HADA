//
// Created by jarry_goon on 24. 8. 1.
//

#ifndef COMPUTEPOINTS_HPP
#define COMPUTEPOINTS_HPP

#include <opencv2/opencv.hpp>

typedef struct {
    float steer;
    float left_lane_x;
    float right_lane_x;
} Output;

//
void compute_point4BEV(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                       const cv::Mat &tvec, const cv::Mat &         rvec, const cv::Mat &mat3D,
                       cv::Point2f    points[4]
);

void init_changed_point4BEV(cv::Point2f points[4]);

Output compute_steer_angle(const cv::Mat &bev_with_seg1,cv::Mat &output_frame);
#endif //COMPUTEPOINTS_HPP
