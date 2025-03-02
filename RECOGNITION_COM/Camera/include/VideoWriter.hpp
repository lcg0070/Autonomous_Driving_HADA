//
// Created by shg08 on 2024-09-18.
//

#ifndef VIDEOWRITE_HPP
#define VIDEOWRITE_HPP

#include <opencv2/opencv.hpp>

class FrameCapture
{
public:
    FrameCapture() = default;
    FrameCapture(const FrameCapture& other);
    explicit FrameCapture(const std::string &save_file_name, const cv::Size& frame_size, int fps = 30);

    ~FrameCapture();

    bool write(const cv::Mat &frame);

private:
    cv::VideoWriter video_writer;

    static std::string get_current_time();
};

#endif //VIDEOWRITE_HPP
