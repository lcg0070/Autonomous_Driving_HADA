//
// Created by jarry_goon on 24. 10. 18.
//

#include "UpperCamera/BoundingBoxProcessing.hpp"

extern inline bool bound_inner_check(const cv::Rect &present, const cv::Rect &target);

extern inline int bbox_norm1(const cv::Rect &bbox1, const cv::Rect &bbox2);
