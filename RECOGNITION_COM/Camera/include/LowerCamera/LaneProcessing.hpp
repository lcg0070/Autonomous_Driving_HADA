//
// Created by jarry_goon on 24. 10. 19.
//

#ifndef LANEPROCESSING_HPP
#define LANEPROCESSING_HPP

#include <opencv2/opencv.hpp>

#define   WIDTH            float (4.f )
#define   LENGTH           float (6.f)
#define   YMAX             float (1.f )

#define   METER_PER_COL    float ( 0.01f)
#define   METER_PER_ROW    float ( 0.01f)
#define   OUT_MAX_COLS     int (2.4f / METER_PER_COL)
#define   OUT_MAX_ROWS     int ((4.4f)/METER_PER_ROW)


#define LOAD_WIDTH    float (1.4f)
#define SLOPE_THRESHOLD float (0.13f)

cv::Mat polyFit(const cv::Mat &X_, const cv::Mat &Y_, int n);

void sliding_window(const cv::Mat& bev_roi, cv::Point2f& steer_point, cv::Point2f& left_point,
                                            cv::Point2f& right_point, cv::Mat& out_img,
                                            cv::Mat& prev_center_fit_x, int roi_num);


#endif //LANEPROCESSING_HPP
