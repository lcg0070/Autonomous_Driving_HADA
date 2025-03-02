//
// Created by User on 2024-10-03.
//

#ifndef OFF_CIRCLE_APPROX_HPP
#define OFF_CIRCLE_APPROX_HPP


//  Circle_approx Translation
#define LIDAR2CAMERA_TRANSLATION    0.292       // [m]
#define CAMERA2GPS_TRANSLATION      0.261       // [m]
#define LIDAR2GPS_TRANSLATION       0.553       // [m]
#define OBSTALCE_THRESHOLD          0.15         // [m]

// Circle Approx
#define MIN_OBJECT_SIZE 15
#define IS_OBJECT(X)            (((X) >> 3) & 1)
#define IS_NOT_CONTINUOUS(X)    (((X) >> 2) & 1)
#define IS_MAX_RANGE(X)         (((X) >> 1) & 1)
#define IS_LAST_IDX(X)          ((X) & 1)

//    INTERESTED_INDEX/7
#define OBSTACLE_INDEX          (int)((float)ORIGINAL_INDEX/(float)MIN_OBJECT_SIZE)
#define MEDIAN_SIZE             (int)15

void median_filter(USTPolarData pol_data, float* median_array);

int circle_approx(USTPolarData pol_data,float* median_array, float* out_array);

int visualize_off(float* median_array, float* out_array);

int x2UV(double x);
int y2UV(double y);

#endif //OFF_CIRCLE_APPROX_HPP
