//
// Created by jarry_goon on 24. 10. 19.
//

#include <numeric>

#include "LowerCamera/LaneProcessing.hpp"

cv::Mat polyFit(const cv::Mat &X_, const cv::Mat &Y_, int n)
{
    if(X_.rows != Y_.rows || X_.cols != 1 || Y_.cols != 1)
    {
        std::cerr << "X and Y must be column vectors with the same number of rows." << std::endl;
        return cv::Mat();
    }


    cv::Mat X2, Y2;
    X_.convertTo(X2, CV_64F);
    Y_.convertTo(Y2, CV_64F);

    cv::Mat A(X2.rows, n + 1, CV_64F);
    for(int i = 0; i < X2.rows; ++i)
    {
        double x_val = X2.at<double>(i, 0);
        for(int j = 0; j < n + 1; ++j)
        {
            A.at<double>(i, j) = std::pow(x_val, n - j);
        }
    }

    cv::Mat At = A.t();
    cv::Mat AtA, AtY, c;

    AtA = At * A;
    AtY = At * Y2;
    cv::solve(AtA, AtY, c, cv::DECOMP_SVD);

    return c;
}


void sliding_window(const cv::Mat& bev_roi, cv::Point2f& steer_point, cv::Point2f& left_point,
                                            cv::Point2f& right_point, cv::Mat& out_img,
                                            cv::Mat& prev_center_fit_x, int roi_num) {

    int left_x = out_img.cols/4;
    int right_x = 3*out_img.cols/4;

    // 슬라이딩 윈도우창 크기 설정
    int n_windows = 1;
    int window_height = out_img.rows / n_windows;
    int margin = out_img.cols / 6;
    int minpix = window_height >> 1;
    int threshold_value = 200;


    // bev_roi 에서 nonzero pixel을 찾는다.
    std::vector<cv::Point> nonzero;
    std::vector<int> left_lane_idx, right_lane_idx;
    std::vector<cv::Rect> rectangles;

    // bev_roi는 이진화된 흑백 이미지여야 한다.
    cv::Mat gray_bev_img;
    if (bev_roi.channels() == 3) cvtColor(bev_roi, gray_bev_img, cv::COLOR_BGR2GRAY);
    else gray_bev_img = bev_roi;

    cv::findNonZero(gray_bev_img, nonzero);

    for (int window = 0; window < n_windows; ++window) {
        int window_y_start = out_img.rows - (window + 1) * window_height;
        int window_y_end = out_img.rows - window * window_height;

        int left_window_x_start = left_x - margin;
        int left_window_x_end = left_x + margin -20;
        int right_window_x_start = right_x - margin +20;
        int right_window_x_end = right_x + margin;

        rectangles.emplace_back(left_window_x_start, window_y_start, 2 * margin, window_height);
        rectangles.emplace_back(right_window_x_start, window_y_start, 2 * margin, window_height);

        std::vector<int> left_window_idx, right_window_idx;
        for (size_t i = 0; i < nonzero.size(); ++i) {
            int x = nonzero[i].x;
            int y = nonzero[i].y;

            // 픽셀 값이 threshold_value 이상인 경우에만 저장
            if (gray_bev_img.at<uchar>(y, x) >= threshold_value) {
                if (y >= window_y_start && y < window_y_end) {
                    if (x >= left_window_x_start && x < left_window_x_end)
                        left_window_idx.push_back(i);
                    if (x >= right_window_x_start && x < right_window_x_end)
                        right_window_idx.push_back(i);
                }
            }
        }

        left_lane_idx.insert(left_lane_idx.end(), left_window_idx.begin(), left_window_idx.end());
        right_lane_idx.insert(right_lane_idx.end(), right_window_idx.begin(), right_window_idx.end());

        if (left_window_idx.size() > minpix) {
            left_x = std::accumulate(left_window_idx.begin(), left_window_idx.end(), 0,
                                     [&](int sum, int idx) { return sum + nonzero[idx].x; }) / left_window_idx.size();
        }
        if (right_window_idx.size() > minpix) {
            right_x = std::accumulate(right_window_idx.begin(), right_window_idx.end(), 0,
                                      [&](int sum, int idx) { return sum + nonzero[idx].x; }) / right_window_idx.size();
        }
    }

    /**************************************************************************************************/
    // Step 5: Prepare data for polynomial fitting
    cv::Mat left_lane_x= cv::Mat::zeros(left_lane_idx.size(), 1, CV_32F);
    cv::Mat left_lane_y= cv::Mat::zeros(left_lane_idx.size(), 1, CV_32F);
    cv::Mat right_lane_x= cv::Mat::zeros(right_lane_idx.size(), 1, CV_32F);
    cv::Mat right_lane_y= cv::Mat::zeros(right_lane_idx.size(), 1, CV_32F);

    // nonzero fixel들을 이용하여 polyFit 진행 -> 차선의 진행 방향 파악
    for (size_t i = 0; i < left_lane_idx.size(); ++i) {
        left_lane_x.at<float>(i, 0) = static_cast<float>(nonzero[left_lane_idx[i]].x);
        left_lane_y.at<float>(i, 0) = static_cast<float>(nonzero[left_lane_idx[i]].y);
    }
    for (size_t i = 0; i < right_lane_idx.size(); ++i) {
        right_lane_x.at<float>(i, 0) = static_cast<float>(nonzero[right_lane_idx[i]].x);
        right_lane_y.at<float>(i, 0) = static_cast<float>(nonzero[right_lane_idx[i]].y);
    }

    cv::Mat left_coeff = cv::Mat::zeros(2, 1, CV_32F);
    cv::Mat right_coeff = cv::Mat::zeros(2, 1, CV_32F);
    if (left_coeff.type() != CV_32F) left_coeff.convertTo(left_coeff, CV_32F);
    if (right_coeff.type() != CV_32F) right_coeff.convertTo(right_coeff, CV_32F);

    // Step 6: Fit polynomial to the lanes
    if (!left_lane_x.empty() && !left_lane_y.empty())
        left_coeff = polyFit(left_lane_y, left_lane_x, 1);
    if (!right_lane_x.empty() && !right_lane_y.empty())
        right_coeff = polyFit(right_lane_y, right_lane_x, 1);
    /**************************************************************************************************/

    /**************************************************************************************************/
    // Step 7: Visualize
    if (left_coeff.empty() || right_coeff.empty() || left_coeff.rows != 2 || left_coeff.cols != 1 || right_coeff.rows != 2 || right_coeff.cols != 1) std::cerr << "Polynomial fit matrix dimensions are incorrect!" << std::endl;
    if (left_coeff.empty()) left_coeff = cv::Mat::zeros(2, 1, CV_32F);
    if (right_coeff.empty()) right_coeff = cv::Mat::zeros(2, 1, CV_32F);
    if (left_coeff.type() != CV_32F) left_coeff.convertTo(left_coeff, CV_32F);
    if (right_coeff.type() != CV_32F) right_coeff.convertTo(right_coeff, CV_32F);

    // 슬라이딩 윈도우창 Visualize
    for (const auto& rect : rectangles) cv::rectangle(out_img, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), cv::Scalar(0, 255, 0), 2);

    // Step 8: Generate x and y values for plotting
    std::vector<cv::Point> left_points, right_points, center_points;
    std::vector<double> plot_y(out_img.rows), left_fit_x(out_img.rows), right_fit_x(out_img.rows),
                         center_fit_x(out_img.rows);


    float left_slope = left_coeff.at<float>(0);
    float right_slope = right_coeff.at<float>(0);
    for (int i = 0; i < out_img.rows; ++i) {
        plot_y[i] = i;

        // 왼쪽과 오른쪽 차선의 x 좌표 계산
        left_fit_x[i]  = left_coeff.at<float>(0) * i + left_coeff.at<float>(1);
        right_fit_x[i] = right_coeff.at<float>(0) * i + right_coeff.at<float>(1);

        // 왼쪽 차선, 오른쪽 차선 가운데 값
        // 한쪽 차선만 검출될 시, 차선 기준으로 도로폭만큼
        if(left_coeff.at<float>(0) != 0 || left_coeff.at<float>(1) != 0) {
            if(right_coeff.at<float>(0) != 0 || right_coeff.at<float>(1) != 0)
                center_fit_x[i] = (left_fit_x[i] + right_fit_x[i]) / 2.0f;
            else
                center_fit_x[i] = left_fit_x[i] +  LOAD_WIDTH / METER_PER_COL;
        }else {
            if(right_coeff.at<float>(0) != 0 || right_coeff.at<float>(1) != 0)
                center_fit_x[i] = right_fit_x[i] - LOAD_WIDTH / METER_PER_COL;
            else
                center_fit_x[i] = prev_center_fit_x.at<double>(roi_num, i);
        }

        // 기울기 예외처리 추가
        if(left_slope < -SLOPE_THRESHOLD) center_fit_x[i] = left_fit_x[i] +  LOAD_WIDTH / METER_PER_COL;
        if(right_slope > SLOPE_THRESHOLD) center_fit_x[i] = right_fit_x[i] -  LOAD_WIDTH / METER_PER_COL;

        // poly fit lines x 좌표 저장
        left_points.push_back(cv::Point(static_cast<int>(left_fit_x[i]), i));
        right_points.push_back(cv::Point(static_cast<int>(right_fit_x[i]), i));
        center_points.push_back(cv::Point(static_cast<int>(center_fit_x[i]), i));
    }

    // 차선 검출 안됐을시 이전 center_fit_x 값 이용
    for (int i = 0; i < out_img.rows; ++i)
        prev_center_fit_x.at<double>(roi_num, i) = center_fit_x[i];


    // 조향명령을 위한 점 3개 찍기
    steer_point = cv::Point(static_cast<int>(center_fit_x[0]), 0);
    left_point = cv::Point(static_cast<int>(left_fit_x[0]), 0);
    right_point = cv::Point(static_cast<int>(right_fit_x[0]), 0);
    cv::circle(out_img, steer_point, 10, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);


    // nonzero pixel 색 변환
    for (int idx : left_lane_idx)
        if (idx >= 0 && idx < nonzero.size()) out_img.at<cv::Vec3b>(nonzero[idx]) = cv::Vec3b(0, 0, 255);
    for (int idx : right_lane_idx)
        if (idx >= 0 && idx < nonzero.size()) out_img.at<cv::Vec3b>(nonzero[idx]) = cv::Vec3b(255, 0, 0);


    // Draw the left and right polynomial fit lines
    if (!left_coeff.empty() || !right_coeff.empty()) {
        cv::polylines(out_img, left_points, false, cv::Scalar(0, 0, 255), 2);
        cv::polylines(out_img, right_points, false, cv::Scalar(255, 0, 0), 2);
        cv::polylines(out_img, center_points, false, cv::Scalar(255, 255, 255), 2);
    }
}
