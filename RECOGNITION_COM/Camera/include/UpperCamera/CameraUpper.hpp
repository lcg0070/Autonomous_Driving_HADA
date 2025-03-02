/*****************************************************************************************/
/* Author: KHW1960                                                                       */
/* Date  : 2024-09-15                                                                    */
/*****************************************************************************************/

#ifndef CAMERAFRONTUPPERFUNCTION_HPP
#define CAMERAFRONTUPPERFUNCTION_HPP

#define USE_FRONT_UPPER_CAMERA

#define DELIVERY_STOP_THRESHOLD 2.0 //[m]

#include <opencv2/opencv.hpp>
#include <YOLO.cuh>

typedef struct {
    int delivery;
    int traffic_light;
} Flag;

void init_upper_cam(cv::VideoCapture &front_upper_cam);
int delivery_mission(cv::Mat& front_upper_frame, YOLO::Object& target_info);
int traffic_light_recognition(cv::Mat &front_upper_frame);


Flag upper_flag(cv::Mat &front_upper_frame, YOLO::Object &target_info);
#endif //CAMERAFRONTUPPERFUNCTION_HPP
