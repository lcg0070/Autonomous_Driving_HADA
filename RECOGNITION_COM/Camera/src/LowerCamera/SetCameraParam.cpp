#include "LowerCamera/SetCameraParam.hpp"
#include "LowerCamera/ComputePoints.hpp"

extern inline void set_camera_parameter(cv::Mat &camera_matrix, cv::Mat &dist_coeffs);

extern inline void set_camera_intrinsic_maxtrix(const cv::Mat &camera_matrix,
                                                const cv::Mat &dist_coeffs,
                                                cv::Mat &      map1,
                                                cv::Mat &      map2
);

extern inline void set_3D_matrix(cv::Mat &mat3D);

extern inline void set_camera_extrinsic_matrix(cv::Mat& tvec, cv::Mat& rvec);

void set_BEV_parameter(cv::Mat &      M, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const cv::Mat &tvec,
                       const cv::Mat &rvec, const cv::Mat &mat3D
)
{
    cv::Point2f dst_vertices[4];
    cv::Point2f src_vertices[4];

    compute_point4BEV(camera_matrix, dist_coeffs, tvec, rvec, mat3D, src_vertices);

    init_changed_point4BEV(dst_vertices);           // BEV Image SIZE
    M = getPerspectiveTransform(src_vertices, dst_vertices);
}//
