//
// Created by jarry_goon on 2024-02-26.
//

#ifndef YOLOV8_CUDA_KERNEL_FUNTION_CUH
#define YOLOV8_CUDA_KERNEL_FUNTION_CUH

#define MAX_THREAD 1024
#define BOX_IDX (int)4

#include <opencv2/opencv.hpp>
#include <cuda_fp16.h>

__global__ void post_process_detect(const void* src, float* score, int* class_idxes, cv::Rect* box,
                                    int num_class, int channel, int type_size,
                                    int img_width, int img_height, float aspect_ratio);

__global__ void post_process_segment(const void* src, float* score, int* class_idxes,
                                     cv::Rect* box, float* mask_configures,
                                     int num_class, int channel, int segment_channel, int type_size,
                                     int img_width, int img_height, float aspect_ratio);

__global__ void half2float(const half* src, float* dst, int size);

__global__ void float2half(const float* src, half* dst, int size);

#endif //YOLOV8_CUDA_KERNEL_FUNTION_CUH
