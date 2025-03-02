//
// Created by jarry_goon on 2024-02-16.
//

#ifndef YOLO_YOLOV8_CUH
#define YOLO_YOLOV8_CUH

#include <Engine.cuh>
#include <opencv2/opencv.hpp>

class YOLO
{
public:
    struct Object
    {
        int      label;
        float    score;
        cv::Rect bbox;
        cv::Mat  seg;
    };

    explicit YOLO(const std::string& model_path);

    bool predict(const cv::Mat& img, float threshold = 0.5f, uint32_t max_object = 10);

    std::vector<Object> get_objects();

    static void draw_segment(const cv::Mat& img, cv::Mat& dst, const Object& object, const cv::Scalar& color);

private:
    void resize_img(const cv::Mat& img, cv::cuda::GpuMat& dst) const;

    void blob(const cv::cuda::GpuMat& img, cv::cuda::GpuMat& dst) const;

    bool detection(void* input_img_data, float threshold, uint32_t max_object);

    bool segmentation(void* input_img_data, float threshold, uint32_t max_object);

    enum Mode
    {
        NONE,
        DETECTION,
        SEGMENTATION
    };

    Engine model;
    Mode   mode;

    // Model info
    cv::Size           input_layer_size;
    nvinfer1::DataType img_datatype;
    int                input_bind_idx;
    nvinfer1::Dims     output0_dims;
    int                output0_bind_idx;
    nvinfer1::Dims     output1_dims;
    int                output1_bind_idx;

    cv::Size img_size;
    float    aspect_ratio;

    // Output
    std::vector<Object> objects;
};

#endif //YOLOV8_YOLOV8_CUH
