//
// Created by User on 2024-09-04.
//

#ifndef OFF_PROCESSING_HPP
#define OFF_PROCESSING_HPP

#ifdef __cplusplus
extern "C" {
#endif


// Index
typedef enum {
    DEGREE_m90 = 1800,
    DEGREE_m80 = 1720,
    DEGREE_m60 = 1560,
    DEGREE_m40 = 1400,
    DEGREE_m30 = 1320,
    DEGREE_0 = 1080,
    DEGREE_30 = 840,
    DEGREE_40 = 760,
    DEGREE_60 = 600,
    DEGREE_80 = 440,
    DEGREE_90 = 360
} Angle;


#define ORIGINAL_INDEX      ((int)1441)                     // ORIGINAL     index 90~-90 degree
//#define INTERESTED_INDEX    ((int)1281)                     // interested   index 80~-80 degree
#define INTERESTED_INDEX    ((int)DEGREE_m40 - DEGREE_40 + 1)                     // interested   index 80~-80 degree

#define DEGREE_OFFSET       ((int)DEGREE_90)                // DEGREE
#define INTERESTED_OFFSET   ((int)(DEGREE_40-DEGREE_90))    // DEGREE

#define OBSERVE_DEGREE      (90.f)
#define INTERESTED_DEGREE   (40.f)

// LiDAR SPEC
#define MAX_RANGE           (7.f)               // 15 [m]
#define RESOLUTION          (0.125f)            // 0.125 [degree]


// function
#define	PI		3.14159265358979323846264338327950288419716939937510582
#define DEG2RAD                 (PI / 180.0)
#define RAD2DEG                 (180.0 / PI)
#define SQR(x)                  ((x) * (x))

//visualize
#define OFF_WIDTH               (int)701
#define OFF_HEIGHT              (int)351
#define GRID_SIZE               0.02             // [m]


// STEERING
#define RS                      (double)0.7
#define SIGMA                   (double)1.5
#define WEIGHT                  (double)0.5


int init_OFF();

int OFF_processing();

double OFF_processing_cluster(int visual_flag, int record_video_flag);

int clear_OFF();

void receive_data();

#ifdef __cplusplus
}
#endif

#endif //OFF_PROCESSING_HPP
