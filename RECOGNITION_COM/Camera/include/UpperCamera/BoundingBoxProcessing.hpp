//
// Created by jarry_goon on 24. 10. 18.
//

#ifndef BOUNDINGBOXPROCESSING_HPP
#define BOUNDINGBOXPROCESSING_HPP

#include <opencv2/opencv.hpp>

inline bool bound_inner_check(const cv::Rect &present, const cv::Rect &target)
{
    // int diff_width  = (target.x + target.width >> 1) - present.x;
    // int diff_height = (target.y + target.height >> 1) - present.y;
    int diff_width  = target.x - present.x;
    int diff_height = target.y - present.y;

    bool inner_width  = (0 < diff_width) && (diff_width < present.width);
    bool inner_height = (0 < diff_height) && (diff_height < present.height);

    return inner_width && inner_height;
}

inline int bbox_norm1(const cv::Rect &bbox1, const cv::Rect &bbox2)
{
    return abs(bbox1.x - bbox2.x) + abs(bbox1.y - bbox2.y);
}

#endif //BOUNDINGBOXPROCESSING_HPP
