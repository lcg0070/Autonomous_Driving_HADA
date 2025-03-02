//
// Created by User on 2024-10-03.
//


#include <stdio.h>
#include <Windows.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include "UST.h"
#include "circle_approx.hpp"
#include "OFF_processing.hpp"


// circle approximation;
static float obstacle_xy[OBSTACLE_INDEX * 3];
static int obstacle_num_visual;

static float starting_angle[OBSTACLE_INDEX];
static int  starting_idx[OBSTACLE_INDEX];
static float ending_angle[OBSTACLE_INDEX];
static int  ending_idx[OBSTACLE_INDEX];


/*-----------------------------------------------------------------------------------*/
/* Median filter                                                                     */
/*-----------------------------------------------------------------------------------*/
static int compare(const void* a, const void* b) {
    float float_a = *(const float*)a;
    float float_b = *(const float*)b;

    if (float_a < float_b) return -1;
    if (float_a > float_b) return 1;
    return 0;
}


void median_filter(USTPolarData pol_data, float* median_array) {
    float temp[MEDIAN_SIZE];

    int ancker_point;

    ancker_point = MEDIAN_SIZE >> 1;

    for (int i = 0; i < ORIGINAL_INDEX; i++)
    {
        if(pol_data.distance[i + DEGREE_90] == 0 || pol_data.distance[i + DEGREE_90] > MAX_RANGE){
            pol_data.distance[i + DEGREE_90] = MAX_RANGE;
        }
    }
    for (int i = 0; i < ancker_point; i++) {
        median_array[i]                        = pol_data.distance[i + DEGREE_90];
        median_array[ORIGINAL_INDEX - i - 1]    = pol_data.distance[pol_data.size - i - DEGREE_90 - 1];
    }

    for (int i = 0; i <= ORIGINAL_INDEX - MEDIAN_SIZE; i++) {
        memcpy(temp, pol_data.distance + i + DEGREE_90, sizeof(float) * MEDIAN_SIZE);
        qsort(temp, MEDIAN_SIZE, sizeof(float), compare);
        median_array[i + ancker_point] = temp[ancker_point];
    }
}




/*-----------------------------------------------------------------------------------*/
/* Circular Approximation                                                            */
/*-----------------------------------------------------------------------------------*/
static char check_obstacle(const float range,  const float before_range,
                           const int    idx,    const int    start_idx
)
{
    char is_object;
    char is_last_idx;
    char is_max_range;
    char is_not_continuous;

    is_object         = idx - start_idx >= MIN_OBJECT_SIZE;
    is_last_idx       = idx == ORIGINAL_INDEX - 1;
    is_max_range      = range == MAX_RANGE;
    is_not_continuous = fabs(range - before_range) > OBSTALCE_THRESHOLD;

    return (is_object << 3) | (is_not_continuous << 2) | (is_max_range << 1) | is_last_idx;
}

static int obstacle_index(USTPolarData pol_data, float* out_array)
{
    char obstacle_criteria;

    int    obstacle_num  = 0;
    int    obstacle_flag = 0;
    float before_data   = out_array[0];

    // Loop through all the LIDAR data points
    for(int i = 0; i < ORIGINAL_INDEX; i++)
    {
        obstacle_criteria = check_obstacle(out_array[i], before_data,
                                           i, starting_idx[obstacle_num]);
        if((obstacle_criteria > 8) && obstacle_flag)
        {
            ending_angle[obstacle_num] = pol_data.angle[i - 1 + DEGREE_OFFSET];
            ending_idx[obstacle_num]   = i - 1;
            obstacle_num++;
        }

        obstacle_flag = obstacle_flag ^ IS_MAX_RANGE(obstacle_criteria);

        // If no obstacle is being tracked, start a new one
        if(!obstacle_flag || IS_NOT_CONTINUOUS(obstacle_criteria))
        {
            starting_angle[obstacle_num] = pol_data.angle[i + DEGREE_OFFSET];
            starting_idx[obstacle_num]   = i;
            obstacle_flag                = 1;  // Mark that an obstacle is being tracked
        }
        before_data = out_array[i];  // Update the previous data point
    }
    return obstacle_num;
}

static void obstacle_coordinates(USTPolarData pol_data, const int obstacle_num, float* median_array)
{

    for(int i = 0; i < obstacle_num; i++)
    {
        //        3 points or 2points
        int case_type = 0;

        int   start_idx = starting_idx[i];
        float x1        = cos(starting_angle[i]) * median_array[start_idx];
        float y1        = sin(starting_angle[i]) * median_array[start_idx];

        int   end_idx = ending_idx[i];
        float x2      = cos(ending_angle[i]) * median_array[end_idx];
        float y2      = sin(ending_angle[i]) * median_array[end_idx];

        int    min_index = start_idx;
        float min_data  = MAX_RANGE;

        for(int temp = start_idx; temp <= end_idx; ++temp)
        {
            if(min_data > median_array[temp])
            {
                min_index = temp;
                min_data  = median_array[temp];
            }
        }

        float x3 = 0;
        float y3 = 0;

        if(min_index == start_idx || min_index == end_idx)
        {
            case_type = 1;
        }
        else
        {
            x3 = cos(pol_data.angle[min_index + DEGREE_OFFSET - 1]) * min_data;
            y3 = sin(pol_data.angle[min_index + DEGREE_OFFSET - 1]) * min_data;
        }
        float circle_x    = 0;
        float circle_y    = 0;
        float circle_r    = 0;
        float temp_margin = 0;


        //        circle_case 0,1
        switch(case_type)
        {
            case 0:
            {
                float den = 2. * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2));
                float num1 = (y3 - y2) * (SQR(x2) - SQR(x1) + SQR(y2) - SQR(y1)) + (y1 - y2) * (
                        SQR(x3) - SQR(x2) + SQR(y3) - SQR(y2));
                float num2 = (x2 - x3) * (SQR(x2) - SQR(x1) + SQR(y2) - SQR(y1)) + (x2 - x1) * (
                        SQR(x3) - SQR(x2) + SQR(y3) - SQR(y2));

                circle_x = num1 / den;
                circle_y = num2 / den;

                break;
            }
            case 1:
            {
                circle_x = (x1 + x2) * 0.5;
                circle_y = (y1 + y2) * 0.5;

                break;
            }
            default:
            {
                break;
            }
        }

        obstacle_xy[3 * i]     = circle_x + LIDAR2GPS_TRANSLATION;
        obstacle_xy[3 * i + 1] = circle_y;
        // (-max + min)* |deg|/35 + max
//        temp_margin = (DEFAULT_MARGIN - MAX_MARGIN) * fabs(RAD2DEG * atan2(circle_y, obstacle_xy[3 * i]) )  / 90.0 +
//                      MAX_MARGIN;
//        circle_r               = hypotf(circle_x - x1, circle_y - y1) + temp_margin;
        circle_r               = hypotf(circle_x - x1, circle_y - y1);

        obstacle_xy[3 * i + 2] = circle_r;
    }
}

void lidardata(int obstacle_num, float* out_array)
{
    for(int i = 0; i<ORIGINAL_INDEX ;i++){
        out_array[i] = MAX_RANGE;
    }
    //    initialize output
    for(int i = 0; i < obstacle_num; i++)
    {
        float obs_centerrange = hypotf(obstacle_xy[3 * i], obstacle_xy[3 * i + 1]);

        float obs_minrange    = obs_centerrange - obstacle_xy[3 * i + 2];

        //        over_range
        if(obs_minrange > MAX_RANGE)
        {
            continue;
        }
        //        car_crashes to obstacle
        if(obs_minrange < 0)
        {
            continue;
        }

        //        radian
        float obs_angle       = asin(obstacle_xy[3 * i + 2] / obs_centerrange);
        float obs_centerangle = atan2(obstacle_xy[3 * i + 1], obstacle_xy[3 * i]);

        float obs_minangle = obs_centerangle - obs_angle;
        float obs_maxangle = obs_centerangle + obs_angle;

        for(int j = 0; j < ORIGINAL_INDEX; j++)
        {
            float degree = DEG2RAD*(OBSERVE_DEGREE - RESOLUTION * (float)j);
            if(degree >= obs_maxangle)
            {
                continue;
            }
            if(degree < obs_minangle)
            {
                break;
            }
            float temp_x = obstacle_xy[3 * i];
            float temp_y = obstacle_xy[3 * i + 1];
            float temp_r = obstacle_xy[3 * i + 2];

            float Tan       = tan(degree);
            float sqrt_term =
                    sqrt(SQR(temp_x + temp_y * Tan) - (SQR(Tan) + 1) * (
                            SQR(temp_x) + SQR(temp_y) - SQR(temp_r)));
            float x1 = (temp_x + temp_y * Tan + sqrt_term) / (SQR(Tan) + 1);
            float y1 = Tan * x1;
            float r1 = hypotf(x1, y1);          // pythagoras
            float x2 = (temp_x + temp_y * Tan - sqrt_term) / (SQR(Tan) + 1);
            float y2 = Tan * x2;
            float r2 = hypotf(x2, y2);

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


int circle_approx(USTPolarData pol_data,float* median_array, float* out_array){
    int obstacle_num;
    obstacle_num = obstacle_index(pol_data, median_array);
    obstacle_coordinates(pol_data, obstacle_num, median_array);
    lidardata(obstacle_num, out_array);

    // visualize
    obstacle_num_visual = obstacle_num;

    return 0;
}




/*-----------------------------------------------------------------------------------*/
/* OFF Visualization                                                                 */
/*-----------------------------------------------------------------------------------*/

int x2UV(double x){return    (int)(OFF_WIDTH/2. + x/GRID_SIZE);}
int y2UV(double y){return    (int)(OFF_HEIGHT - y/GRID_SIZE);}




int visualize_off(float* median_array, float* out_array){
    cv::Mat median_visual_data =        cv::Mat::zeros(OFF_HEIGHT,OFF_WIDTH, CV_8UC1);
    cv::Mat out_visual_data =           cv::Mat::zeros(OFF_HEIGHT,OFF_WIDTH, CV_8UC1);
    cv::Mat visual_obstacle =           cv::Mat::zeros(OFF_HEIGHT,OFF_WIDTH, CV_8UC1);

    uchar* median_ptr = median_visual_data.ptr<uchar>(0);
    uchar* out_ptr = out_visual_data.ptr<uchar>(0);

    for(int i = 0; i<ORIGINAL_INDEX; i++){
        float degree = ((float)OBSERVE_DEGREE - RESOLUTION * (float)i) * DEG2RAD;
        float x = median_array[i] * sin(degree);
        float y= median_array[i] * cos(degree);
        int u = x2UV(x);
        int v = y2UV(y);

        if(u >= 0 && u < OFF_WIDTH && v >= 0 && v < OFF_HEIGHT) {
            median_ptr[v * OFF_WIDTH + u] = 255;
        }

        x = out_array[i] * std::sin(degree);
        y = out_array[i] * std::cos(degree);
        u = x2UV(x);
        v = y2UV(y);

        if(u >= 0 && u < OFF_WIDTH && v >= 0 && v < OFF_HEIGHT) {
            out_ptr[v * OFF_WIDTH + u] = 255;
        }
    }

    for(int i =0; i< obstacle_num_visual; i++){
        int u = x2UV(obstacle_xy[3*i+1]);
        int v = y2UV(obstacle_xy[3*i]);
        float r = obstacle_xy[3*i+2]/GRID_SIZE;

        if(u < 0 || u > OFF_WIDTH)  continue;
        if(v < 0 || v > OFF_HEIGHT) continue;

        cv::circle(visual_obstacle, cv::Point(u,v), r,cv::Scalar(255));
    }

    cv::imshow("median_lidar_data", median_visual_data);
    cv::imshow("lidar_data", out_visual_data);
    cv::imshow("obstacle_data", visual_obstacle);

    if(cv::waitKey(1) == 27) return 1;

    return 0;
}