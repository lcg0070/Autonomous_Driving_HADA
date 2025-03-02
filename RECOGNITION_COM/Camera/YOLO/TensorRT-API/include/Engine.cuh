//
// Created by jarry_goon on 2024-02-16.
//

#ifndef TENSORRT_API_ENGINE_CPP
#define TENSORRT_API_ENGINE_CPP

#include <string>
#include <vector>

#include <NvInfer.h>

class Logger final: public nvinfer1::ILogger
{
    void log(Severity severity, const char* msg) noexcept override;
};

struct LayerInfo
{
    std::string            layer_name;
    nvinfer1::TensorIOMode io_mode;
    nvinfer1::DataType     data_type;
    nvinfer1::Dims         dims;
};

class Engine
{
public:
    explicit Engine(const std::string& model_path);

    Engine(const Engine& engine) = delete;

    Engine& operator=(const Engine& engine) = delete;

    ~Engine();

    std::vector<LayerInfo> get_IO_layers() const;

    bool run(void* const* bindings) const;

private:
    nvinfer1::IRuntime*          runtime;
    nvinfer1::ICudaEngine*       engine;
    nvinfer1::IExecutionContext* context;

    static int onnx2engine(Logger logger, std::vector<char>& model_data);
};

#endif //TENSORRT_API_ENGINE_CPP
