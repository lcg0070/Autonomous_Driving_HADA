//
// Created by 이찬근 on 2024. 8. 1..
//
#include "Velodyne.h"
#include "LiDAR_coordchange.h"
#include "SetLidarParam.h"

inline double SQR(double value) {return value * value;}

////change lidar data to camera cordinate
void lidar2uvcoord(cv::Mat& uvmat, cv::Mat& xyzmat, PointCloud data[][VELODYNE_CHANNELS], const cv::Mat& cam_rotate){
    static cv::Mat coord_mat(4, (int)VELODYNE_DATA_SIZE*VELODYNE_CHANNELS, CV_64F);

    int num = 0;
    for(int i=0; i<VELODYNE_DATA_SIZE; i++){
        for(int j = 0; j<VELODYNE_CHANNELS; j++){
            if(data[i][j].intensity == 0){
                continue;
            }
            coord_mat.at<double>(0,num) = data[i][j].x;
            coord_mat.at<double>(1,num) = data[i][j].y;
            coord_mat.at<double>(2,num) = data[i][j].z;
            coord_mat.at<double>(3,num) = 1.0;
            num+=1;
        }
    }

    xyzmat = coord_mat.colRange(0, num);
    uvmat = cam_rotate * xyzmat;

}


//// return x,y,z when input u,v coordinate
int uv2realcoord(cv::Mat &uvmat, int interest_points[], double tol) {
    double* uvptrx = uvmat.ptr<double>(0);
    double* uvptry = uvmat.ptr<double>(1);
    double* uvptrz = uvmat.ptr<double>(2);

    for (int i = 0; i < uvmat.cols; i++) {
        double s = uvptrz[i];
        uvptrx[i] = static_cast<int>(uvptrx[i] / s);
        uvptry[i] = static_cast<int>(uvptry[i] / s);

        // checking tol
        if (SQR(uvptrx[i] - interest_points[0]) + SQR(uvptry[i] - interest_points[1]) < SQR(tol))
            return i;
    }

    return -1;
}



//// return x,y,z when input u,v coordinate
int uv2real_visualize(cv::Mat& frame, cv::Mat &uvmat) {
    double* uvptrx = uvmat.ptr<double>(0);
    double* uvptry = uvmat.ptr<double>(1);
    double* uvptrz = uvmat.ptr<double>(2);

    for (int i = 0; i < uvmat.cols; i++) {
        double s = uvptrz[i];
        uvptrx[i] = static_cast<int>(uvptrx[i] / s);
        uvptry[i] = static_cast<int>(uvptry[i] / s);
        cv::circle(frame, cv::Point(uvptrx[i], uvptry[i]), 3,cv::Scalar(0,255,0), -1);

    }
    cv::imshow("visual_test",frame);
    if(cv::waitKey(1) == 27) return 1;

    return 0;
}