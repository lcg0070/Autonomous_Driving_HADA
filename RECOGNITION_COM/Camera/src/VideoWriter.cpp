//
// Created by 2024 hada camera team on 2024-03-01.
//
#include "VideoWriter.hpp"

FrameCapture::FrameCapture(const FrameCapture &other)
{
    this->video_writer = other.video_writer;
}

FrameCapture::FrameCapture(const std::string &save_file_name, const cv::Size &frame_size, int fps)
{
    std::string file_name = save_file_name + "_" + get_current_time() + ".mp4";

    this->video_writer.open(file_name, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, frame_size, true);
}

FrameCapture::~FrameCapture()
{
    this->video_writer.release();
}

bool FrameCapture::write(const cv::Mat &frame)
{
    if(!this-> video_writer.isOpened())
    {
        std::cerr << "Error: Could not open the BEV output video file!" << std::endl;

        return false;
    }

    if(frame.channels() == 1) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

    this->video_writer.write(frame);

    return true;
}


std::string FrameCapture::get_current_time()
{
    std::time_t        now       = std::time(nullptr);
    std::tm*           localTime = std::localtime(&now);
    std::ostringstream oss;

    oss << std::put_time(localTime, "%Y%m%d_%H%M%S");

    return oss.str();
}
