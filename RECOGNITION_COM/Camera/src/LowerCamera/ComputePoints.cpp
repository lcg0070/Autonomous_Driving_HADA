#include "LowerCamera/ComputePoints.hpp"
#include "LowerCamera/CameraLower.hpp"
#include "LowerCamera/LaneProcessing.hpp"

void compute_point4BEV(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                       const cv::Mat &tvec, const cv::Mat &         rvec,
                       const cv::Mat &mat3D, cv::Point2f            points[4]
)
{
    cv::Mat projected_points;
    // 2D 좌표 구하기
    cv::projectPoints(mat3D, rvec, tvec, camera_matrix, dist_coeffs, projected_points);


    // Bird Eye View를 위한 4개의 point 지정 return 될 값
    points[0] = cv::Point2f(projected_points.ptr<float>(0)[0], projected_points.ptr<float>(0)[1]);
    points[1] = cv::Point2f(projected_points.ptr<float>(1)[0], projected_points.ptr<float>(1)[1]);
    points[2] = cv::Point2f(projected_points.ptr<float>(3)[0], projected_points.ptr<float>(3)[1]);
    points[3] = cv::Point2f(projected_points.ptr<float>(2)[0], projected_points.ptr<float>(2)[1]);
}

void init_changed_point4BEV(cv::Point2f points[4])
{
    points[0] = cv::Point2f(0.f, 0.f);
    points[1] = cv::Point2f(OUT_MAX_COLS, 0.f);
    points[2] = cv::Point2f(OUT_MAX_COLS, OUT_MAX_ROWS);
    points[3] = cv::Point2f(0.f, OUT_MAX_ROWS);
}

inline cv::Point2f compute_world_point(float pixel_u, float pixel_v)
{
    float world_x = ((pixel_u - (float)(OUT_MAX_COLS >> 1)) * METER_PER_COL);
    float world_y = (4.866) - pixel_v * METER_PER_ROW;

    return {world_x, world_y};
}

cv::Mat prev_center_fit_x = cv::Mat::ones(3, OUT_MAX_COLS, CV_64F) * (OUT_MAX_COLS / 2.0);

Output compute_steer_angle(const cv::Mat &bev_with_seg, cv::Mat &output_frame)
{
    cv::Point2f world_center,left_center, right_center, steer_point, left_point, right_point;
    cv::Point2f center_point{0.f, 0.f};
    cv::Point2f left_x{0.f, 0.f};
    cv::Point2f right_x{0.f, 0.f};
    static float weight[3] = {0.4f, 0.3f, 0.3f};

    Output output;
    bev_with_seg.copyTo(output_frame);

    for (int i = 0; i < 3; i++) {
        cv::Rect roi(0, i * (bev_with_seg.rows / 3), bev_with_seg.cols, (bev_with_seg.rows / 3));
        cv::Mat bev_roi = bev_with_seg(roi);

        cv::Mat out_img = cv::Mat::zeros(bev_roi.size(), bev_roi.type());

        steer_point = cv::Point2f(0.f, 0.f);
        sliding_window(bev_roi, steer_point, left_point, right_point, out_img, prev_center_fit_x,i);

        out_img.copyTo(output_frame(roi));

        world_center = compute_world_point(steer_point.x, steer_point.y);
        left_center = compute_world_point(left_point.x, left_point.y);
        right_center = compute_world_point(right_point.x, right_point.y);

        center_point += weight[i] * world_center;
        left_x += weight[i] * left_center;
        right_x += weight[i] * right_center;

    }

    float l_d = std::hypot(world_center.x, center_point.y + WHEEL2CAM);
    float alpha = std::atan2(world_center.x, world_center.y + WHEEL2CAM);

    output.steer = 6.f*std::atan2(2.0 * WHEEL_BASE * std::sin(alpha), l_d) * RAD2DEG; // Steering calculation
    if (output.steer > BEV_STEER_MAX) output.steer = BEV_STEER_MAX;
    else if (output.steer < -BEV_STEER_MAX) output.steer = -BEV_STEER_MAX;


    output.left_lane_x = left_x.x;
    output.right_lane_x = right_x.x;

    return output;
}