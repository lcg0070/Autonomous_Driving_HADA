//
// Created by dadan on 24. 10. 20.
//

#ifndef TRAFFICLIGHTPROCESSING_HPP
#define TRAFFICLIGHTPROCESSING_HPP

#include <YOLO.cuh>
#include <opencv2/opencv.hpp>

typedef enum LIGHT_Labels
{
    G4,
    GL4,
    R4,
    RL3,
    RL4,
    Y4,
} LIGHT_Labels;

void draw_light_boxes(const YOLO::Object &sign_object, cv::Mat &frame);
void light_state(int light_label, cv::Mat &frame);

#endif //TRAFFICLIGHTPROCESSING_HPP
