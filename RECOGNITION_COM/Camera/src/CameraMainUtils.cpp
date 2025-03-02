//
// Created by 2024 hada camera team on 2024-03-01.
//
#include "LowerCamera/CameraLower.hpp"
#include "UpperCamera/CameraUpper.hpp"
#include "CameraMainUtils.hpp"
#include "YOLO.cuh"

#include "VideoWriter.hpp"

cv::VideoCapture front_lower_cam(1, cv::CAP_DSHOW);
cv::VideoCapture front_upper_cam(0, cv::CAP_DSHOW);


void init_camera() {
    init_lower_cam(front_lower_cam);
    init_upper_cam(front_upper_cam);
}

Flag upper_camera_processing() {
    YOLO::Object target_info = {0};
    cv::Mat front_upper_frame;

    front_upper_cam >> front_upper_frame;
    if (front_upper_frame.empty()) fprintf(stderr, "front_upper_frame empty");

    return upper_flag(front_upper_frame, target_info);
}

Output lower_camera_processing() {
    cv::Mat front_lower_frame;

    front_lower_cam >> front_lower_frame;
    if (front_lower_frame.empty()) fprintf(stderr, "front_lower_frame empty");

    return lane_tracking(front_lower_frame);
}

