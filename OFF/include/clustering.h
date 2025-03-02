//
// Created by LeeChanKeun on 24. 8. 5.
//

#ifndef CLUSTERING_H
#define CLUSTERING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "UST.h"

// DBSCAN PARAM
#define EPS_CLUSTER             (float)0.10f                //0.10f
#define MINIMUM_DATAPOINTS      (size_t)15                   //15
#define NUM_LSH_TABLE           (size_t)1                   //1

// OFFSET PARAM
#define GPS2LIDAR               (double)0.95f                       //0.94
#define GPS2SIDE                (double)0.52f                       //0.52
#define GPS2DIAGONAL            ((double)hypot(GPS2LIDAR, GPS2SIDE))//1.07
#define PEAK_DEGREE             (double)28.9510f
#define GRADIENT_0_2_PEAK       ((double)((GPS2DIAGONAL-GPS2LIDAR)/PEAK_DEGREE))
#define GRADIENT_PEAK_2_90      ((double)(GPS2SIDE-GPS2DIAGONAL)/(90.-PEAK_DEGREE))





typedef struct{
    double x;
    double y;
}point_cord;

int DBSCAN(const float* x_data,
           const float* y_data,
           int*         labels,
           size_t       data_size,
           float        eps,
           size_t       min_num_points,
           size_t       num_lsh_hyperplanes,
           size_t       num_lsh_table
);


void DBSCAN_processing(USTRectData rec_data, int* labels, int num_label, double* out_array);

void circle_approximation(double* obstacle_xy, int obstacle_num, double* out_array);

double steer_command(double* off, double sff_degree);

void visual_cluster(USTRectData rec_data, int* labels, double* out_array);

#ifdef __cplusplus
}
#endif

#endif //CLUSTERING_H
