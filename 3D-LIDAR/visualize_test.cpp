//
// Created by User on 2024-10-06.
//

#include "3D_Lidarfunction.hpp"
#include <opencv2/opencv.hpp>

int main() {

    cv::VideoCapture cap(0, cv::CAP_DSHOW);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 60);

    if (!cap.isOpened()) {
        return -1;
    }
    init_velodyne();

    cv::Mat frame;

    while (1) {
        cap>>frame;

        if(velodyne_visual_processing(frame)) break;

    }

    clear_velodyne();

    return 0;
}