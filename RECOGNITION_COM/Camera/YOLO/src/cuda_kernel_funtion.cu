//
// Created by jarry_goon on 2024-02-26.
//

#include "cuda_kernel_funtion.cuh"

#define CLAMP(X, MIN, MAX) ((X) < (MIN)?(MIN):(((X) > (MAX))?(MAX):(X)))

__global__ void post_process_detect(const void* src, float*          score, int*             class_idxes, cv::Rect* box,
                                    const int   num_class, const int channel, const int      type_size,
                                    const int   img_width, const int img_height, const float aspect_ratio
)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // 인덱스 범위가 실제 행렬 범위를 벗어나는 경우 함수 실행 X
    if(idx >= channel) return;

    float max_score = 0.f;
    float val;

    float x;
    float y;
    float width;
    float height;

    float x0;
    float x1;
    float y0;
    float y1;

    int class_idx = 0;

    if(type_size == 4)
    {
        const float* src_float = static_cast<const float*>(src);

        x      = src_float[idx];
        y      = src_float[channel + idx];
        width  = src_float[channel * 2 + idx];
        height = src_float[channel * 3 + idx];

        for(int i = 0; i < num_class; i++)
        {
            val = src_float[channel * (i + BOX_IDX) + idx];

            if(max_score < val)
            {
                max_score = val;
                class_idx = i;
            }
        }
    }
    else
    {
        const half* src_half = static_cast<const half*>(src);

        x      = __half2float(src_half[idx]);
        y      = __half2float(src_half[channel + idx]);
        width  = __half2float(src_half[channel * 2 + idx]);
        height = __half2float(src_half[channel * 3 + idx]);

        for(int i = 0; i < num_class; i++)
        {
            val = __half2float(src_half[channel * (i + BOX_IDX) + idx]);

            if(max_score < val)
            {
                max_score = val;
                class_idx = i;
            }
        }
    }

    x0 = CLAMP((x - width * 0.5f) * aspect_ratio, 0.f, img_width);
    x1 = CLAMP((x + width * 0.5f) * aspect_ratio, 0.f, img_width);
    y0 = CLAMP((y - height * 0.5f) * aspect_ratio, 0.f, img_height);
    y1 = CLAMP((y + height * 0.5f) * aspect_ratio, 0.f, img_height);

    box[idx].x      = x0;
    box[idx].y      = y0;
    box[idx].width  = x1 - x0;
    box[idx].height = y1 - y0;

    if(box[idx].width == 0) box[idx].width = 1;
    if(box[idx].height == 0) box[idx].height = 1;

    score[idx]       = max_score;
    class_idxes[idx] = class_idx;
}

__global__ void post_process_segment(const void* src, float*          score, int* class_idxes,
                                     cv::Rect*   box, float*          mask_configures,
                                     const int   num_class, const int channel, const int   segment_channel,
                                     const int   type_size, const int img_width, const int img_height,
                                     const float aspect_ratio
)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // 인덱스 범위가 실제 행렬 범위를 벗어나는 경우 함수 실행 X
    if(idx >= channel) return;

    float max_score = 0.f;
    float val;

    float x;
    float y;
    float width;
    float height;

    float x0;
    float x1;
    float y0;
    float y1;

    int class_idx = 0;

    if(type_size == 4)
    {
        const float* src_float = static_cast<const float*>(src);

        x      = src_float[idx];
        y      = src_float[channel + idx];
        width  = src_float[channel * 2 + idx];
        height = src_float[channel * 3 + idx];

        for(int i = 0; i < num_class; i++)
        {
            val = src_float[channel * (i + BOX_IDX) + idx];

            if(max_score < val)
            {
                max_score = val;
                class_idx = i;
            }
        }

        for(int i                                      = 0; i < segment_channel; i++)
            mask_configures[segment_channel * idx + i] = src_float[channel * (BOX_IDX + num_class + i) + idx];
    }
    else
    {
        const half* src_half = static_cast<const half*>(src);

        x      = __half2float(src_half[idx]);
        y      = __half2float(src_half[channel + idx]);
        width  = __half2float(src_half[channel * 2 + idx]);
        height = __half2float(src_half[channel * 3 + idx]);

        for(int i = 0; i < num_class; i++)
        {
            val = __half2float(src_half[channel * (i + BOX_IDX) + idx]);

            if(max_score < val)
            {
                max_score = val;
                class_idx = i;
            }
        }

        for(int i                                      = 0; i < segment_channel; i++)
            mask_configures[segment_channel * idx + i] =
                    __half2float(src_half[channel * (BOX_IDX + num_class + i) + idx]);
    }

    x0 = CLAMP((x - width * 0.5f) * aspect_ratio, 0.f, img_width);
    x1 = CLAMP((x + width * 0.5f) * aspect_ratio, 0.f, img_width);
    y0 = CLAMP((y - height * 0.5f) * aspect_ratio, 0.f, img_height);
    y1 = CLAMP((y + height * 0.5f) * aspect_ratio, 0.f, img_height);

    box[idx].x      = x0;
    box[idx].y      = y0;
    box[idx].width  = x1 - x0;
    box[idx].height = y1 - y0;

    if(box[idx].width == 0) box[idx].width = 1;
    if(box[idx].height == 0) box[idx].height = 1;

    score[idx]       = max_score;
    class_idxes[idx] = class_idx;
}

__global__ void half2float(const half* src, float* dst, const int size)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx >= size) return;

    dst[idx] = __half2float(src[idx]);
}

__global__ void float2half(const float* src, half* dst, const int size)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx >= size) return;

    dst[idx] = __float2half(src[idx]);
}
