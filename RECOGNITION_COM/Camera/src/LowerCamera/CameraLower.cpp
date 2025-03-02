//
// Created by 2024 hada camera team on 2024-03-01.
//
#define _CRT_SECURE_NO_WARNINGS

#include <numeric>

#include "LowerCamera/CameraLower.hpp"
#include "LowerCamera/ComputePoints.hpp"
#include "LowerCamera/LaneProcessing.hpp"
#include "LowerCamera/SetCameraParam.hpp"
#include "VideoWriter.hpp"
#include "YOLO.cuh"

static cv::Mat map1;
static cv::Mat map2;
static cv::Mat M;

static cv::Mat bev_frame(OUT_MAX_ROWS, OUT_MAX_COLS, CV_8UC3);
static YOLO    LANE("etc/lane_nano_4th.engine");

static FrameCapture origin_capture;
static FrameCapture bev_capture;
static FrameCapture output_capture;

int init_lower_cam(cv::VideoCapture &front_lower_cam)
{
    cv::Mat dist_coeffs, camera_matrix;
    cv::Mat tvec,        rvec, mat3D;

    if(!front_lower_cam.isOpened())
    {
        std::cerr << "Error: Could not open front_lower_camera" << std::endl;

        return -1;
    }

    front_lower_cam.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    front_lower_cam.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    // cv::namedWindow("front_lower_cam", cv::WINDOW_NORMAL);
    // cv::namedWindow("calibrate", cv::WINDOW_NORMAL);

    set_camera_parameter(camera_matrix, dist_coeffs);
    set_camera_intrinsic_maxtrix(camera_matrix, dist_coeffs, map1, map2);
    set_camera_extrinsic_matrix(tvec, rvec);
    set_3D_matrix(mat3D);
    set_BEV_parameter(M, camera_matrix, dist_coeffs, tvec, rvec, mat3D);

    origin_capture = FrameCapture("origin", cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
    bev_capture    = FrameCapture("BEV", cv::Size(OUT_MAX_COLS, OUT_MAX_ROWS));
    output_capture = FrameCapture("output", cv::Size(OUT_MAX_COLS, OUT_MAX_ROWS));

    return 0;
}

Output lane_tracking(const cv::Mat &front_lower_frame)  // front_lower_frame
{
    Output   output;
    cv::Mat output_frame;

    cv::Mat calibration_frame;
    cv::Mat bev_with_seg = cv::Mat::zeros(bev_frame.size(), bev_frame.type());

    // 1. Calibration & Convert BEV
    cv::remap(front_lower_frame, calibration_frame, map1, map2, cv::INTER_NEAREST);
    cv::warpPerspective(calibration_frame, bev_frame, M, bev_frame.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT);

    // 2. Predict
    if(!LANE.predict(bev_frame)) return output;
    auto objects = LANE.get_objects();

    int mid= bev_frame.cols / 2;
    int left_lane_idx =0;
    int right_lane_idx=-1;



    std::sort(objects.begin(), objects.end(), [](const YOLO::Object& a, const YOLO::Object& b) {
        return a.bbox.x < b.bbox.x;
    });

    for(int obj_idx = 0; obj_idx < objects.size(); obj_idx++) {
        if((objects[obj_idx].bbox.x >= objects[left_lane_idx].bbox.x) && (objects[obj_idx].bbox.x<mid))
            left_lane_idx = obj_idx;
        if((objects[obj_idx].bbox.x > mid + 50)) {
            right_lane_idx = obj_idx;
            if(right_lane_idx != -1) break;
        }
    }

    if (right_lane_idx >= 0 && right_lane_idx < objects.size())
        for (int i = objects.size() - 1; i > right_lane_idx; i--)
            objects.erase(objects.begin() + i);

    if (left_lane_idx >= 0 && left_lane_idx < objects.size())
        for(int i = left_lane_idx - 1; i >= 0; i--)
            objects.erase(objects.begin() + i);

    for(const YOLO::Object &object: objects)
    {
        if(object.label == 0)
        {
            // cv::rectangle(bev_frame, object.bbox, cv::Scalar(0, 0, 255), 2);
            // YOLO::draw_segment(bev_frame, bev_frame, object, cv::Scalar(0, 0, 255));
        }
        else
        {
            // cv::rectangle(bev_frame, object.bbox, cv::Scalar(0, 255, 0), 2);
            // printf("label: %d\n", object.label);
            // YOLO::draw_segment(bev_frame, bev_frame, object, cv::Scalar(0, 255, 0.));

            cv::Mat segmentMask = cv::Mat::zeros(bev_frame.size(), CV_8UC1);
            object.seg.copyTo(segmentMask(object.bbox));
            bev_with_seg.setTo(255, segmentMask);
        }
    }

    output = compute_steer_angle(bev_with_seg, output_frame);

    // imshow("front_lower_cam",front_lower_frame);
    // imshow("calibration_frame",calibration_frame);
    imshow("bev_frame", bev_frame);
    // imshow("bev_frame_with_segmentation",bev_with_seg);
    cv::imshow("combined_img", output_frame);

    origin_capture.write(front_lower_frame);
    bev_capture.write(bev_frame);
    output_capture.write(output_frame);

    cv::waitKey(1);

    /***************************************************************************************/
    return output;
}
