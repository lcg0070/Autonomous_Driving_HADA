//
// Created by user on 2024-03-01.
//

#ifndef BIRDEYEVIEW_CAMERA_HPP
#define BIRDEYEVIEW_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <Windows.h>

#include "ComputePoints.hpp"

#define   USE_CAMERA

#define fx double  ( 745.5741557364419f) //745.5741557364419f
#define fy double  ( 746.0437250295756f) //746.0437250295756f
#define cx double  ( 645.3469963983930f) //645.3469963983930f
#define cy double  ( 359.2217191875898f) //359.2217191875898f
#define k1 double  (-0.310168356441994f) //0.310168356441994f
#define k2 double  ( 0.067011411982780f) //0.067011411982780f
#define p1 double  ( 0.0f)
#define p2 double  ( 0.0f)

#define transY float (0.0f               )
#define X      float (0.0f               )
#define Y      float (0.3f               )
#define Z      float (0.5f               )
#define ROLL   float (1.750756134537095f )
#define PITCH  float (-0.002f            )
#define YAW    float (0.000f             )

#define MAX_FRAME_PREVIOUS   int (1)
#define FRAME_WIDTH  int (1280)
#define FRAME_HEIGHT int (720)
#define ROI          int (2)

#define WHEEL_BASE    float (1.175151f)
#define BEV_STEER_MAX float (30.0f)
#define WHEEL2CAM     float (1.420127f)

#define RAD2DEG (double) (180.  / CV_PI)
#define DEG2RAD (double) (CV_PI / 180. )

inline double get_time()
{
    LARGE_INTEGER end_counter, frequency;

    QueryPerformanceCounter(&end_counter);
    QueryPerformanceFrequency(&frequency);

    return (double)end_counter.QuadPart / (double)frequency.QuadPart;
}

int init_lower_cam(cv::VideoCapture &front_lower_cam);

Output lane_tracking(const cv::Mat &front_lower_frame);

#endif //BIRDEYEVIEW_CAMERA_HPP
