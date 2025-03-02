//
// Created by jarry_goon on 24. 8. 5.
//

#include "clustering.h"
#include "LSH.hpp"
#include "circle_approx.hpp"
#include "OFF_processing.hpp"
#include "UST.h"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <vector>

extern "C" {




const int CLUSTER_NUM = (size_t)(ORIGINAL_INDEX/MINIMUM_DATAPOINTS);
const int POINT_NUM   = (int)(OBSERVE_DEGREE/RESOLUTION);

int DBSCAN(const float* x_data,
           const float* y_data,
           int*         labels,
           size_t       data_size,
           float        eps,
           size_t       min_num_points,
           size_t       num_lsh_hyperplanes,
           size_t       num_lsh_table
){
    std::vector<size_t> candidate_idx_vec; // Nearest neighbor candidates

    int global_groups = 0;              // Number of groups in global
    int group;                          // Group number of a current point

    int pre_vec_size   = 0;             // Vector size of previous point
    int last_point_idx = 0;             // Index number of last point that have group number

    // Create LSH(Local Sensitive Hashing) object
    LSH hash_map(x_data, y_data, data_size, num_lsh_hyperplanes, num_lsh_table);

    for(int data_idx = 0; data_idx < data_size; data_idx++)
    {
        // 1. Find the nearest neighbor
        candidate_idx_vec = hash_map.nearest_neighbor(data_idx, eps);

        // 2. Determine noise or not
        if(candidate_idx_vec.size() < min_num_points)
        {
            labels[data_idx] = -1;

            // If length to nearest point is smaller than eps, this point is not noise.
            // However, the number of nearest neighbors of the nearest point needs to be bigger minimum neighbor number.
            if(pre_vec_size > min_num_points &&
               hypotf(x_data[data_idx] - x_data[last_point_idx], y_data[data_idx] - y_data[last_point_idx]) <= eps)
                labels[data_idx] = labels[last_point_idx];

            continue;
        }

        // 3. Determine the same group or not the previous point
        if(labels[last_point_idx] &&
           hypotf(x_data[data_idx] - x_data[last_point_idx], y_data[data_idx] - y_data[last_point_idx]) <= eps)
        {
            group            = labels[data_idx - 1];
            labels[data_idx] = group;
        }
        else if(!labels[data_idx])
        {
            group            = ++global_groups;
            labels[data_idx] = group;
        }
        else
            group = labels[data_idx];

        // 4. Labeling nearest neighbors
        for(int candidate_idx: candidate_idx_vec)
        {
            if(labels[candidate_idx]) continue;

            labels[candidate_idx] = group;
        }

        pre_vec_size   = candidate_idx_vec.size();
        last_point_idx = data_idx;
    }

    return global_groups;
}




// calculate the margin according to the degree of teh obstacle
double calculate_margin(double degree){

    double out_margin = 0;

    if (fabs(degree) > 90) return out_margin;
    if (fabs(degree) < 0)  return out_margin;

    // gradient of the 1st order func
    // if (fabs(degree) <= PEAK_DEGREE)
    //     out_margin = GRADIENT_0_2_PEAK * fabs(degree) + GPS2LIDAR; // [m]
    // else
    //     out_margin = GRADIENT_PEAK_2_90 * fabs(degree) + GPS2SIDE + GRADIENT_PEAK_2_90*(-90.); // [m]
    out_margin = GRADIENT_PEAK_2_90 * fabs(degree) + GPS2SIDE + GRADIENT_PEAK_2_90*(-90.); // [m]

    return out_margin;
}


double       cluster_xy[3*CLUSTER_NUM];
point_cord  each_data[CLUSTER_NUM][POINT_NUM];
int         each_count[CLUSTER_NUM];

void DBSCAN_processing(USTRectData rec_data, int* labels, int num_label, double* out_array){

    if(num_label == 0){return;}

    // initialize count, data
    for(int i = 0; i<num_label; i++){
        each_count[i]         = 0;
        cluster_xy[3 * i]     = 0;
        cluster_xy[3 * i + 1] = 0;
        cluster_xy[3 * i + 2] = 0;
    }

    int temp_flag = 1;  // obstacle start point -> for calculating radius
    for(int i = 0; i<ORIGINAL_INDEX; i++){
        int label = labels[i];
        if(label == -1){ continue;}
        cluster_xy[3 * (label - 1)]      += rec_data.x[i + DEGREE_OFFSET];
        cluster_xy[3 * (label - 1) + 1]  += rec_data.y[i + DEGREE_OFFSET];

        each_data[label - 1][each_count[label - 1]].x = rec_data.x[i + DEGREE_OFFSET];
        each_data[label - 1][each_count[label - 1]].y = rec_data.y[i + DEGREE_OFFSET];
        each_count[label - 1]++;
    }

    // apply offset to the lidar2COG
    for(int i = 0; i < num_label; i++){
        double cluster_center_x = cluster_xy[3 * i]     / (double)each_count[i] ;
        double cluster_center_y = cluster_xy[3 * i + 1] / (double)each_count[i] ;

        cluster_xy[3 * i]       = cluster_center_x + GPS2LIDAR;
        cluster_xy[3 * i + 1]   = cluster_center_y;


        // apply margin according to the angle of the object
        double max_rng           = 0;
        double temp_rng          = 0;
        double obstacle_degree   = 0;
        double temp_margin       = 0;
        for(int j = 0; j < each_count[i]; j++){

            temp_rng = hypot(cluster_center_x - each_data[i][j].x, cluster_center_y - each_data[i][j].y);
            obstacle_degree = RAD2DEG*atan2(cluster_xy[3 * i + 1], cluster_xy[3 * i]);

            temp_margin = calculate_margin(obstacle_degree);
            temp_rng += temp_margin;
            if(max_rng <= temp_rng) max_rng = temp_rng;
        }
        cluster_xy[3 * i + 2]  = temp_rng;
    }

//    visualize obstacle_point
//    cv::Mat obstacle_data =      cv::Mat::zeros(OFF_HEIGHT,OFF_WIDTH, CV_8UC1);
//    for(int i=0; i < num_label; i++){
//        int u = x2UV(cluster_xy[3*i+1]);
//        int v = y2UV(cluster_xy[3*i]);
//        float r = cluster_xy[3*i+2];
//        if(u >= 0 && u < OFF_WIDTH && v >= 0 && v < OFF_HEIGHT) {
//            cv::circle(obstacle_data, cv::Point(u,v), (int)(r/GRID_SIZE), cv::Scalar(255));
//        }
//    }
//    cv::imshow("cluster approx", obstacle_data);


    circle_approximation(cluster_xy, num_label, out_array);

    return;
}

void circle_approximation(double* obstacle_xy,int obstacle_num, double* out_array)
{
    for(int i = 0; i < ORIGINAL_INDEX ;i++){
        out_array[i] = MAX_RANGE;
    }
    //    initialize output
    for(int i = 0; i < obstacle_num ; i++)
    {
        double obs_centerrange = hypot(obstacle_xy[3 * i], obstacle_xy[3 * i + 1]);

        double obs_minrange    = obs_centerrange - obstacle_xy[3 * i + 2];

        // wrong obstacle
        if(obs_centerrange <= 0){continue;}

        //        over_range
        if(obs_minrange >= MAX_RANGE){continue;}

        //        car_crashes to obstacle
        if(obs_minrange <= 0){continue;}

        //        radian
        double obs_angle       = asin(obstacle_xy[3 * i + 2] / obs_centerrange);
        double obs_centerangle = atan2(obstacle_xy[3 * i + 1], obstacle_xy[3 * i]);

        double obs_minangle = obs_centerangle - obs_angle;
        double obs_maxangle = obs_centerangle + obs_angle;

        for(int j = 0; j < ORIGINAL_INDEX; j++)
        {
            double degree = DEG2RAD*(OBSERVE_DEGREE - RESOLUTION * (float)j);
            if(degree >= obs_maxangle)
            {
                continue;
            }
            if(degree < obs_minangle)
            {
                break;
            }
            double temp_x = obstacle_xy[3 * i];
            double temp_y = obstacle_xy[3 * i + 1];
            double temp_r = obstacle_xy[3 * i + 2];

            double Tan       = tan(degree);
            double sqrt_term =
                    sqrt(SQR(temp_x + temp_y * Tan) - (SQR(Tan) + 1) * (
                            SQR(temp_x) + SQR(temp_y) - SQR(temp_r)));
            double x1 = (temp_x + temp_y * Tan + sqrt_term) / (SQR(Tan) + 1);
            double y1 = Tan * x1;
            double r1 = hypot(x1, y1);          // pythagoras
            double x2 = (temp_x + temp_y * Tan - sqrt_term) / (SQR(Tan) + 1);
            double y2 = Tan * x2;
            double r2 = hypot(x2, y2);

            if(out_array[j] > r1)
            {
                out_array[j] = r1;
            }
            if(out_array[j] > r2)
            {
                out_array[j] = r2;
            }
        }
    }
}


cv::Scalar Colors[10] ={
        cv::Scalar(255, 0, 0),    // Blue
        cv::Scalar(0, 255, 0),    // Green
        cv::Scalar(0, 0, 255),    // Red
        cv::Scalar(255, 255, 0),  // Cyan
        cv::Scalar(255, 0, 255),  // Magenta
        cv::Scalar(0, 255, 255),  // Yellow
        cv::Scalar(128, 0, 128),  // Purple
        cv::Scalar(255, 165, 0),  // Orange
        cv::Scalar(128, 128, 128),// Gray
        cv::Scalar(255, 255, 255) // white
};


void visual_cluster(USTRectData rec_data, int* labels, double* out_array){

    cv::Mat original_visual_data =      cv::Mat::zeros(OFF_HEIGHT,OFF_WIDTH, CV_8UC1);
    uchar* original_ptr = original_visual_data.ptr<uchar>(0);
    for(int i=0; i<ORIGINAL_INDEX; i++){
        int u = x2UV(rec_data.y[i + DEGREE_OFFSET]);
        int v = y2UV(rec_data.x[i + DEGREE_OFFSET]);

        if(u >= 0 && u < OFF_WIDTH && v >= 0 && v < OFF_HEIGHT) {
            original_ptr[v * OFF_WIDTH + u] = 255;
        }
    }

    cv::Mat DBSCAN_visual_data =        cv::Mat::zeros(OFF_HEIGHT,OFF_WIDTH, CV_8UC3);
    for(int i=0; i<ORIGINAL_INDEX; i++){
        int u = x2UV(rec_data.y[i + DEGREE_OFFSET]);
        int v = y2UV(rec_data.x[i + DEGREE_OFFSET]);

        if(u >= 0 && u < OFF_WIDTH && v >= 0 && v < OFF_HEIGHT) {
            cv::Scalar color;
            if(labels[i] == -1) continue;
            color = Colors[labels[i]%10];
            DBSCAN_visual_data.at<cv::Vec3b>(v, u) = cv::Vec3b(color[0], color[1], color[2]);
        }
    }

    cv::Mat final_data =                cv::Mat::zeros(OFF_HEIGHT,OFF_WIDTH, CV_8UC1);

    uchar* final_ptr = final_data.ptr<uchar>(0);
    for(int i=0; i<INTERESTED_INDEX; i++){
        double radian = ((double)OBSERVE_DEGREE - RESOLUTION * ((double)i + INTERESTED_OFFSET)) * DEG2RAD;
        double x = out_array[i + INTERESTED_OFFSET] * sin(radian);
        double y = out_array[i + INTERESTED_OFFSET] * cos(radian);
        int u = x2UV(x);
        int v = y2UV(y);

        if(u >= 0 && u < OFF_WIDTH && v >= 0 && v < OFF_HEIGHT) {
            final_ptr[v * OFF_WIDTH + u] = 255;
        }
    }

    cv::imshow("ORIGINAL_data", original_visual_data);
    cv::imshow("DBSCAN_visual_data", DBSCAN_visual_data);
    cv::imshow("final_data", final_data);
}



double steer_command(double* off, double sff_degree){
    static double steer_angle;

    double max_iff = 0.0;

    for(int i = 0; i < INTERESTED_INDEX; i++)
    {
        double angle;
        double sff;
        double iff;

        angle = (double)(INTERESTED_DEGREE - RESOLUTION * (double)i);
        sff   = RS * exp(-SQR(angle - sff_degree) / (2.0 * SIGMA*SIGMA));

        iff = WEIGHT * sff + (1.0 - WEIGHT) * off[i+INTERESTED_OFFSET];

        if(iff > max_iff)
        {
            max_iff     = iff;
            steer_angle = angle;
        }
    }

    return steer_angle;
}


}
