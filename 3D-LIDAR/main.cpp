//
// Created by User on 2024-08-05.
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
        cv::circle(frame, cv::Point(640, 500), 10, cv::Scalar(0, 255, 0), -1);
        int interested_source[2] = {640, 500};
        int tol =15;
        cv::Mat uvmat;

        velodyne_processing(interested_source,tol, uvmat);

        cv::imshow("Frame", frame);

        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    clear_velodyne();

    return 0;
}