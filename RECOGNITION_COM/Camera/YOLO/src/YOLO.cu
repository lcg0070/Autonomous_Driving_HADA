//
// Created by jarry_goon on 2024-02-16.
//

#include "YOLO.cuh"
#include "cuda_kernel_funtion.cuh"

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>

#include <cuda_fp16.h>

#include <iostream>

#define NMS_THRESHOLD 0.7f

#define SIZE_FLOAT  (int) 4 // = sizeof(float)
#define SIZE_HALF   (int) 2 // = sizeof(half)

YOLO::YOLO(const std::string& model_path)
    : model(model_path),
      mode(NONE),
      input_bind_idx(-1),
      output0_bind_idx(-1),
      output1_bind_idx(-1),
      aspect_ratio(0.f)
{
    std::vector<LayerInfo> layers;

    int num_layers;

    // AI 모델 입출력 레이어 정보 추출
    layers     = model.get_IO_layers();
    num_layers = layers.size();

    if(num_layers > 3)
    {
        fprintf(stderr, "ERROR: It is not YOLOv8 model. Check the model.\n");

        return;
    }

    // 입출력 레이어 분류
    for(int i = 0; i < num_layers; i++)
    {
        // YOLO에서 입력 레이어는 모델 관계없이 img 1개
        if(auto [layer_name, io_mode, data_type, dims] = layers[i];
            io_mode == nvinfer1::TensorIOMode::kINPUT)
        {
            input_layer_size.width  = static_cast<int>(dims.d[3]);
            input_layer_size.height = static_cast<int>(dims.d[2]);
            img_datatype            = data_type;
            input_bind_idx          = i;
        }
        else if(layer_name == "output0" && io_mode == nvinfer1::TensorIOMode::kOUTPUT)
        {
            output0_dims     = dims;
            output0_bind_idx = i;
        }
        else if(layer_name == "output1" && io_mode == nvinfer1::TensorIOMode::kOUTPUT)
        {
            output1_dims     = dims;
            output1_bind_idx = i;
        }
    }

    // 입력 레이어가 1개 초과일 경우 YOLO 모델이 아니기 때문에 종료
    if(input_bind_idx != 0)
    {
        fprintf(stderr, "ERROR: It is not YOLOv8 model. Check the model.\n");

        return;
    }

    // 입출력 레이어 특성 기준으로 모드 선정
    if(layers.size() < 3)
        mode = DETECTION;
    else
        mode = SEGMENTATION;
}

bool YOLO::predict(const cv::Mat& img, const float threshold, const uint32_t max_object)
{
    cv::cuda::GpuMat resized_img;
    cv::cuda::GpuMat blob_img;

    /*------------------------------------------------ 전처리 ---------------------------------------------------------*/

    // 1. 이미지 크기 변환
    // 모델 입력 기준과 크기가 맞지 않는 경우 크기와 비율이 모두 일치하는 이미지로 변환
    if(img_size.height != input_layer_size.height || img_size.width != input_layer_size.width)
        resize_img(img, resized_img);
        // 사이즈가 같을 경우 이미지 크기 유지
    else
        resized_img.upload(img);

    // 2. 이미지 BGR에서 RGB로 변경
    cv::cuda::cvtColor(resized_img, resized_img, cv::COLOR_BGR2RGB);

    // 3. HWC에서 CHW로 변환
    blob(resized_img, blob_img);

    // 4. 이미지 데이터를 부동소숫점 형태로 바꾸고 Normalize
    blob_img.convertTo(blob_img, CV_32FC3, 1.f / 255.f);

    // 5. 후처리에 사용할 변수 저장
    img_size     = img.size();    // 원본 이미지 크기 저장
    aspect_ratio = 1.f / std::min(static_cast<float>(input_layer_size.width) / static_cast<float>(img.cols),
                                  static_cast<float>(input_layer_size.height) / static_cast<float>(img.rows));

    // 6. 모델의 역할에 맞는 추정 프로세스 실행
    switch(mode)
    {
        case DETECTION:
            return detection(blob_img.data, threshold, max_object);

        case SEGMENTATION:
            return segmentation(blob_img.data, threshold, max_object);

        default:
        case NONE:
            fprintf(stderr, "ERROR: Can not predict.\n");

            return false;
    }
}

void YOLO::resize_img(const cv::Mat& img, cv::cuda::GpuMat& dst) const
{
    cv::cuda::GpuMat img_device;
    cv::cuda::GpuMat unpadding_resized_img;

    cv::Rect roi;

    int unpadding_width;
    int unpadding_height;

    float ratio;

    // 사이즈를 줄이고 비율을 맞췄을 때 남는 부분을 채우기 위해서 검은색을 기본으로 설정
    dst = cv::cuda::GpuMat(input_layer_size, img.type(), cv::Scalar::all(0));

    // 이미지를 GPU에 업로드
    img_device.upload(img);

    // 가로 세로 비율 계산, 가장 긴 값에 맞춤
    ratio = std::min(static_cast<float>(input_layer_size.width) / static_cast<float>(img.cols),
                     static_cast<float>(input_layer_size.height) / static_cast<float>(img.rows));

    // 스케일링된 이미지 가로 세로 길이
    unpadding_width  = static_cast<int>(ratio * static_cast<float>(img.cols));
    unpadding_height = static_cast<int>(ratio * static_cast<float>(img.rows));

    // 패딩 되지 않은 스케일링된 이미지 생성
    unpadding_resized_img = cv::cuda::GpuMat(unpadding_height, unpadding_width, CV_8UC3);
    cv::cuda::resize(img_device, unpadding_resized_img, unpadding_resized_img.size());

    // 스케일링된 이미지를 입력 레이어 특성에 맞게 패딩(Padding)
    roi = cv::Rect(0, 0, unpadding_width, unpadding_height);
    unpadding_resized_img.copyTo(dst(roi));

    img_device.release();
}

void YOLO::blob(const cv::cuda::GpuMat& img, cv::cuda::GpuMat& dst) const
{
    const int width = input_layer_size.height * input_layer_size.width;

    dst = cv::cuda::GpuMat(1, width, CV_8UC3);

    std::vector<cv::cuda::GpuMat> input_channels{
            cv::cuda::GpuMat(input_layer_size.height, input_layer_size.width, CV_8U,
                             &(dst.ptr()[0])),
            cv::cuda::GpuMat(input_layer_size.height, input_layer_size.width, CV_8U,
                             &(dst.ptr()[width])),
            cv::cuda::GpuMat(input_layer_size.height, input_layer_size.width, CV_8U,
                             &(dst.ptr()[width * 2]))
            };

    cv::cuda::split(img, input_channels);
}

bool YOLO::detection(void* input_img_data, const float threshold, const uint32_t max_object)
{
    void* input;
    void* output;
    void* binding[2];
    bool  predict_succese;

    int output_size;
    int num_class;
    int channel;

    int num_block;
    int data_size;

    float*    scores_device;
    int*      class_idxes_device;
    cv::Rect* box_device;

    float*    scores_host;
    int*      class_idxes_host;
    cv::Rect* box_host;

    std::vector<float>    score_vector;
    std::vector<cv::Rect> boxes_vector;
    std::vector<int>      labels_vector;
    std::vector<int>      indices;

    int    output_vector_size;
    int    idx;
    Object obj;

    /*------------------------------------- 입력 데이터 메모리 할당 ------------------------------------------------------*/

    switch(img_datatype)
    {
        case nvinfer1::DataType::kFLOAT:
            data_size = SIZE_FLOAT;
            input = input_img_data;
            break;

        case nvinfer1::DataType::kHALF:
            data_size = SIZE_HALF * input_layer_size.area() * 3;
            cudaMalloc(&input, data_size);
            num_block = std::ceil(data_size / MAX_THREAD);
            float2half<<<num_block, MAX_THREAD>>>(static_cast<float*>(input),
                                                  static_cast<half*>(input_img_data),
                                                  data_size);
            break;

        default:
            fprintf(stderr, "YOLO only supports half or float.");
            return false;
    }

    /*------------------------------------- 출력 데이터 메모리 할당 ------------------------------------------------------*/

    // 출력 데이터 크기
    output_size = output0_dims.d[1] * output0_dims.d[2];  // 출력 데이터 크기
    num_class   = output0_dims.d[1] - BOX_IDX;           // 데이터 내 클래스 개수 저장
    channel     = output0_dims.d[2];                         // 데이터 채널 저장

    // 출력 데이터 메모리 할당
    cudaMalloc(&output, data_size * output_size);

    /*--------------------------------------------- 추론 --------------------------------------------------------------*/

    // 입출력 바인딩
    binding[input_bind_idx]   = input;
    binding[output0_bind_idx] = output;

    predict_succese = model.run(binding);

    if(img_datatype == nvinfer1::DataType::kHALF) cudaFree(input);

    if(!predict_succese)
    {
        fprintf(stderr, "ERROR: Prediction Fail.\n");
        cudaFree(output);

        return false;
    }

    /*------------------------------------------- 후처리 --------------------------------------------------------------*/

    // 결과값 변수 선언
    cudaMalloc(&scores_device, channel * SIZE_FLOAT);
    cudaMalloc(&class_idxes_device, channel * sizeof(int));
    cudaMalloc(&box_device, channel * sizeof(cv::Rect));

    num_block = std::ceil(static_cast<float>(channel) / static_cast<float>(MAX_THREAD));

    post_process_detect<<<num_block, MAX_THREAD>>>(output,
                                                   scores_device,
                                                   class_idxes_device,
                                                   box_device,
                                                   num_class, channel, data_size,
                                                   img_size.width, img_size.height, aspect_ratio);
    cudaDeviceSynchronize();

    // 출력 배열 초기화
    cudaFree(output);

    // GPU 데이터(VRAM)를 HOST(DRAM)로 이동
    scores_host      = static_cast<float*>(malloc(channel * SIZE_FLOAT));
    class_idxes_host = static_cast<int*>(malloc(channel * sizeof(int)));
    box_host         = static_cast<cv::Rect*>(malloc(channel * sizeof(cv::Rect)));

    cudaMemcpy(scores_host, scores_device, channel * SIZE_FLOAT, cudaMemcpyDeviceToHost);
    cudaMemcpy(class_idxes_host, class_idxes_device, channel * sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(box_host, box_device, channel * sizeof(cv::Rect), cudaMemcpyDeviceToHost);

    // 복사 후 GPU 메모리 해제
    cudaFree(scores_device);
    cudaFree(class_idxes_device);
    cudaFree(box_device);

    // 벡터 데이터로 변경
    score_vector  = std::vector<float>(scores_host, scores_host + channel);
    boxes_vector  = std::vector<cv::Rect>(box_host, box_host + channel);
    labels_vector = std::vector<int>(class_idxes_host, class_idxes_host + channel);

    // 데이터 메모리 해제
    free(scores_host);
    free(class_idxes_host);
    free(box_host);

    // 중첩 영역 감지후 제거
    cv::dnn::NMSBoxesBatched(boxes_vector,
                             score_vector,
                             labels_vector,
                             threshold,
                             NMS_THRESHOLD,
                             indices
                            );

    // 출력 벡터 크기 변경
    output_vector_size = indices.size();

    if(max_object < output_vector_size)
        objects.resize(max_object);
    else
        objects.resize(output_vector_size);

    // 벡터에 데이터 저장
    for(int i = 0; i < objects.size(); i++)
    {
        idx = indices[i];

        obj.bbox  = boxes_vector[idx];
        obj.score = score_vector[idx];
        obj.label = labels_vector[idx];

        objects[i] = obj;
    }

    return true;
}

bool YOLO::segmentation(void* input_img_data, float threshold, uint32_t max_object)
{
    void* input;
    void* output0;
    void* output1;
    void* binding[3];
    bool  predict_succese;

    int output0_size;
    int output1_size;

    int channel;
    int segment_channel;
    int segment_width;
    int segment_height;

    int num_block;
    int data_size;
    int num_class;

    float*    scores_device;
    int*      class_idxes_device;
    cv::Rect* box_device;
    float*    mask_configures;

    float*    scores_host;
    int*      class_idxes_host;
    cv::Rect* box_host;

    std::vector<float>    score_vector;
    std::vector<cv::Rect> boxes_vector;
    std::vector<int>      labels_vector;
    std::vector<int>      indices;

    int output_vector_size;

    int              idx;
    Object           obj{};
    cv::cuda::GpuMat mask_configure;
    cv::cuda::GpuMat mask_device;
    cv::Mat          mask_host;
    cv::Rect         roi;
    cv::cuda::GpuMat prototype;
    float*           output_float;

    /*------------------------------------- 입력 데이터 메모리 할당 ------------------------------------------------------*/

    switch(img_datatype)
    {
        case nvinfer1::DataType::kFLOAT:
            data_size = SIZE_FLOAT;
            input = input_img_data;
            break;

        case nvinfer1::DataType::kHALF:
            data_size = SIZE_HALF;
            cudaMalloc(&input, data_size * input_layer_size.area() * 3);
            num_block = std::ceil(static_cast<float>(data_size * input_layer_size.area() * 3) /
                                  static_cast<float>(MAX_THREAD));
            float2half<<<num_block, MAX_THREAD>>>(static_cast<float*>(input_img_data),
                                                  static_cast<half*>(input),
                                                  input_layer_size.area() * 3);
            break;

        default:
            std::cerr << "YOLO only supports half or float." << std::endl;
            return false;
    }

    /*------------------------------------- 출력 데이터 메모리 할당 ------------------------------------------------------*/

    // 출력 데이터 크기
    output0_size = output0_dims.d[1] * output0_dims.d[2];
    output1_size = output1_dims.d[1] * output1_dims.d[2] * output1_dims.d[3];

    channel         = output0_dims.d[2];    // 데이터 채널 저장
    segment_channel = output1_dims.d[1];    // segment 채널 저장
    segment_width   = output1_dims.d[3];    // segment width
    segment_height  = output1_dims.d[2];    // segment height

    num_class = output0_dims.d[1] - segment_channel - BOX_IDX;    // 데이터 내 클래스 개수 저장

    // 출력 데이터 메모리 할당
    cudaMalloc(&output0, data_size * output0_size);
    cudaMalloc(&output1, data_size * output1_size);

    /*--------------------------------------------- 추론 --------------------------------------------------------------*/

    // 입출력 바인딩
    binding[input_bind_idx]   = input;
    binding[output0_bind_idx] = output0;
    binding[output1_bind_idx] = output1;

    predict_succese = model.run(binding);

    if(img_datatype == nvinfer1::DataType::kHALF) cudaFree(input);

    if(!predict_succese)
    {
        fprintf(stderr, "ERROR: Prediction Fail.\n");
        cudaFree(output0);
        cudaFree(output1);

        return false;
    }

    /*------------------------------------------- 후처리 --------------------------------------------------------------*/

    // 결과값 변수 선언
    cudaMalloc(&scores_device, channel * SIZE_FLOAT);
    cudaMalloc(&class_idxes_device, channel * sizeof(int));
    cudaMalloc(&box_device, channel * sizeof(cv::Rect));
    cudaMalloc(&mask_configures, channel * segment_channel * SIZE_FLOAT);

    // 연산에 사용될 블럭 개수 계산
    num_block = std::ceil(static_cast<float>(channel) / static_cast<float>(MAX_THREAD));

    post_process_segment<<<num_block, MAX_THREAD>>>(output0,
                                                    scores_device,
                                                    class_idxes_device,
                                                    box_device,
                                                    mask_configures,
                                                    num_class, channel, segment_channel, data_size,
                                                    img_size.width, img_size.height, aspect_ratio);
    cudaDeviceSynchronize();

    cudaFree(output0);

    // GPU 데이터(VRAM)를 HOST(DRAM)로 이동
    scores_host      = static_cast<float*>(malloc(channel * SIZE_FLOAT));
    class_idxes_host = static_cast<int*>(malloc(channel * sizeof(int)));
    box_host         = static_cast<cv::Rect*>(malloc(channel * sizeof(cv::Rect)));

    cudaMemcpy(scores_host, scores_device, channel * SIZE_FLOAT, cudaMemcpyDeviceToHost);
    cudaMemcpy(class_idxes_host, class_idxes_device, channel * sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(box_host, box_device, channel * sizeof(cv::Rect), cudaMemcpyDeviceToHost);

    // 복사 후 GPU 메모리 해제
    cudaFree(scores_device);
    cudaFree(class_idxes_device);
    cudaFree(box_device);

    // 벡터 데이터로 변경
    score_vector  = std::vector<float>(scores_host, scores_host + channel);
    boxes_vector  = std::vector<cv::Rect>(box_host, box_host + channel);
    labels_vector = std::vector<int>(class_idxes_host, class_idxes_host + channel);

    // 데이터 메모리 해제
    free(scores_host);
    free(class_idxes_host);
    free(box_host);

    // 중첩 영역 감지후 제거
    cv::dnn::NMSBoxesBatched(boxes_vector,
                             score_vector,
                             labels_vector,
                             threshold,
                             NMS_THRESHOLD,
                             indices
                            );

    // 출력 벡터 크기 변경
    output_vector_size = indices.size();

    if(max_object < output_vector_size)
        objects.resize(max_object);
    else
        objects.resize(output_vector_size);

    // mask ROI 설정
    if(img_size.height > img_size.width)
        roi = cv::Rect(0, 0, segment_width * img_size.width / img_size.height, segment_height);
    else
        roi = cv::Rect(0, 0, segment_width, segment_height * img_size.height / img_size.width);

    // 프로토타입 저장
    switch(img_datatype)
    {
        case nvinfer1::DataType::kFLOAT:
            prototype = cv::cuda::GpuMat(segment_channel, segment_width * segment_height, CV_32F, output1);
            break;

        case nvinfer1::DataType::kHALF:
            cudaMalloc(&output_float, SIZE_FLOAT * output1_size);
            num_block = std::ceil(static_cast<float>(output1_size) / static_cast<float>(MAX_THREAD));
            half2float<<<num_block, MAX_THREAD>>>(static_cast<half*>(output1), output_float, output1_size);
            prototype = cv::cuda::GpuMat(segment_channel, segment_width * segment_height, CV_32F, output_float);

        default:
            break;
    }

    // 벡터에 출력 저장
    for(int i = 0; i < objects.size(); i++)
    {
        idx = indices[i];

        obj.bbox  = boxes_vector[idx];
        obj.score = score_vector[idx];
        obj.label = labels_vector[idx];

        // 세그멘트 마스크 계산
        // 세그멘트 설정값을 행렬에 저장
        mask_configure = cv::cuda::GpuMat(1, segment_channel, CV_32F,
                                          mask_configures + (segment_channel * idx));

        // 설정값과 프로토타입 행렬곱 및 전치
        cv::cuda::gemm(mask_configure, prototype, 1., cv::cuda::GpuMat(), 0.,
                       mask_device, 0.);
        mask_device = mask_device.reshape(1, segment_height);
        mask_device = mask_device(roi);

        cv::cuda::multiply(-1., mask_device, mask_device);
        cv::cuda::exp(mask_device, mask_device);
        cv::cuda::add(1., mask_device, mask_device);
        cv::cuda::divide(1., mask_device, mask_device);

        cv::cuda::resize(mask_device, mask_device, img_size, cv::INTER_LINEAR);
        cv::cuda::compare(mask_device(obj.bbox), threshold, mask_device, cv::CMP_GT);

        mask_device.download(mask_host);

        obj.seg = mask_host;

        objects[i] = obj;
    }

    cudaFree(mask_configures);
    cudaFree(output1);
    if(img_datatype == nvinfer1::DataType::kHALF) cudaFree(output_float);

    return true;
}

void YOLO::draw_segment(const cv::Mat& img, cv::Mat& dst, const Object& object, const cv::Scalar& color)
{
    dst = img.clone();

    cv::Mat color_mat = cv::Mat(img.size(), img.type(), color);
    color_mat(object.bbox).copyTo(color_mat, object.seg);
    cv::addWeighted(img(object.bbox), 1, color_mat, 0.5, 0, dst(object.bbox));
}

std::vector<YOLO::Object> YOLO::get_objects()
{
    return objects;
}
