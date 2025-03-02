//
// Created by User on 2024-08-05.
//

#ifndef INC_3D_LIDAR_SETLIDARPARAM_H
#define INC_3D_LIDAR_SETLIDARPARAM_H

#include <opencv2/opencv.hpp>

//image size
#define IMG_WIDTH   1280
#define IMG_HEIGHT  720

//camera intrinsic matrix
#define FX double  ( 779.5889829009982f)//1126.681718615794f//763.3973399061011f
#define FY double  ( 774.6647059663499f)//1130.745607582599f//763.2758041343890f
#define CX double  ( 649.4586501858179f)//977.7917300810489f//637.1092521893464f
#define CY double  ( 386.4713599380453f)//571.0884879555691f//377.2388697515128f
#define K1 double  (-0.322795936286537f)//0.294112136895409f//0.320145097753203f
#define K2 double  ( 0.075461621612223f)//0.058500560142525f//0.073117587168286f
#define P1 double  ( 0.0f)
#define P2 double  ( 0.0f)

//camera extrinsic matrix
// X,Y,Z [m], ROLL, PITCH, YAW [DEGREE]
#define VELODYNE_X      float   (0.0f)
#define VELODYNE_Y      float   (0.108253175473f)
#define VELODYNE_Z      float   (0.0625f)
#define VELODYNE_ROLL   float   (-90.0f - 0.057295779513082f)
#define VELODYNE_PITCH  float   (0.0f + 15.253757602364123f)
#define VELODYNE_YAW    float   (90.f + 0.947162778791340f)


//// Camera calibration
void cameraCalibration(cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

//// rotation_Translation Matrix ( R | T )
cv::Mat rotation_translation(cv::Mat &rotation);

//// camera matrix(3x3) * rotation and translation matrix(3x4)
cv::Mat lidarCalibration(cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

cv::Mat eul2rotmx();
cv::Mat eul2rotmy();
cv::Mat eul2rotmz();

#endif //INC_3D_LIDAR_SETLIDARPARAM_H
