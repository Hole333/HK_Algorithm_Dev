#include "Y11SegPredictor.h"

static Ort::Env& getGlobalEnv() {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "GlobalVMEnv");
    return env;
}

void YOLOPredictor::getParam(float confThreshold, float iouThreshold, float maskThreshold)
{
    this->confThreshold = confThreshold;
    this->iouThreshold = iouThreshold;
    this->maskThreshold = maskThreshold;
}

void YOLOPredictor::YOLOSegInit(const std::string& modelPath,
    const bool& isGPU,
    float confThreshold,
    float iouThreshold,
    float maskThreshold)
{
    this->confThreshold = confThreshold;
    this->iouThreshold = iouThreshold;
    this->maskThreshold = maskThreshold;
    env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "YOLOV11");
    sessionOptions = Ort::SessionOptions();
    //sessionOptions.SetLogSeverityLevel(0);
    // 获取可用的执行提供程序
    std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
    for (uint8_t i = 0; i < availableProviders.size(); i++)
    {
        std::cout << availableProviders[i] << std::endl;
    }

    // 检查CUDA是否可用
    auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
    OrtCUDAProviderOptions cudaOptions;
    cudaOptions.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchHeuristic;
    cudaOptions.tunable_op_tuning_enable = true;
    cudaOptions.arena_extend_strategy = 1;
    
   
    // 追加CPU执行提供程序
    sessionOptions.SetIntraOpNumThreads(8);
    std::cout << "CPU Execution Provider added." << std::endl;

    if (isGPU && (cudaAvailable != availableProviders.end()))
    {
        std::cout << "Inference device: GPU" << std::endl;
        // 1.19版本使用新的 API 设置 CUDA 提供程序
        
       sessionOptions.AppendExecutionProvider_CUDA(cudaOptions);
    }
    else
    {
        std::cout << "Inference device: CPU" << std::endl;
    }

#ifdef _WIN32
    std::wstring w_modelPath;
    try
    {
        try
        {
            w_modelPath = YOLOutils::charToWstring(modelPath.c_str());
        }
        catch (const std::exception)
        {
            return;
        }
        
        session = Ort::Session(env, w_modelPath.c_str(), sessionOptions);
    }
    catch (Ort::Exception e)
    {
        return;
    }

#else
    session = Ort::Session(env, modelPath.c_str(), sessionOptions);
#endif

    // 获取输入输出节点信息
    const size_t num_input_nodes = session.GetInputCount();
    const size_t num_output_nodes = session.GetOutputCount();
    std::cout << num_input_nodes << std::endl;
    std::cout << num_output_nodes << std::endl;

    if (num_output_nodes > 1)
    {
        this->hasMask = true;
        std::cout << "Instance Segmentation" << std::endl;
    }
    else
    {
        std::cout << "Object Detection" << std::endl;
    }

    Ort::AllocatorWithDefaultOptions allocator;
    for (int i = 0; i < num_input_nodes; i++)
    {
        auto input_name = session.GetInputNameAllocated(i, allocator);
        // std::cout << "Node[" << i << "] runs on: " << input_name << std::endl;
        this->inputNames.push_back(input_name.get());
        input_names_ptr.push_back(std::move(input_name));

        Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(i);
        std::vector<int64_t> inputTensorShape = inputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
        this->inputShapes.push_back(inputTensorShape);
        this->isDynamicInputShape = false;
        if (inputTensorShape[2] == -1 && inputTensorShape[3] == -1)
        {
            // std::cout << "Dynamic input shape" << std::endl;
            this->isDynamicInputShape = true;
        }
    }

    for (int i = 0; i < num_output_nodes; i++)
    {
        try
        {
            auto output_name = session.GetOutputNameAllocated(i, allocator);
            // std::cout << "Node[" << i << "] runs on: " << output_name << std::endl;
            this->outputNames.push_back(output_name.get());
            output_names_ptr.push_back(std::move(output_name));

            Ort::TypeInfo outputTypeInfo = session.GetOutputTypeInfo(i);
            std::vector<int64_t> outputTensorShape = outputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
            this->outputShapes.push_back(outputTensorShape);
            if (i == 0)
            {
                if (!this->hasMask)
                    classNums = outputTensorShape[1] - 4;
                else
                    classNums = outputTensorShape[1] - 4 - 32;
            }
        }
        catch (const Ort::Exception& e)
        {
            return;
        }

    }
   //int time = 2000;
   // while (time--);
   // modelToHeat();
    isInit = true; // 初始化完成
    
    //// 打印输入输出的节点名称
    //for (const char* x : this->inputNames)
    //{
    //    std::cout << x << std::endl;
    //}
    //for (const char* x : this->outputNames)
    //{
    //    std::cout << x << std::endl;
    //}
    //std::cout << classNums << std::endl;
}



void YOLOPredictor::getBestClassInfo(std::vector<float>::iterator it,
    float& bestConf,
    int& bestClassId,
    const int _classNums)
{
    // first 4 element are box
    bestClassId = 4;
    bestConf = 0;

    for (int i = 4; i < _classNums + 4; i++)
    {
        if (it[i] > bestConf)
        {
            bestConf = it[i];
            bestClassId = i - 4;
        }
    }
}
cv::Mat YOLOPredictor::getMask(const cv::Mat& maskProposals,
    const cv::Mat& maskProtos)
{
    cv::Mat protos = maskProtos.reshape(0, { (int)this->outputShapes[1][1], (int)this->outputShapes[1][2] * (int)this->outputShapes[1][3] });

    cv::Mat matmul_res = (maskProposals * protos).t();
    cv::Mat masks = matmul_res.reshape(1, { (int)this->outputShapes[1][2], (int)this->outputShapes[1][3] });
    cv::Mat dest;

    // sigmoid
    cv::exp(-masks, dest);
    dest = 1.0 / (1.0 + dest);
    cv::resize(dest, dest, cv::Size((int)this->inputShapes[0][2], (int)this->inputShapes[0][3]), cv::INTER_LINEAR);
    return dest;
}

void YOLOPredictor::preprocessing(cv::Mat& image, float*& blob, std::vector<int64_t>& inputTensorShape)
{
    cv::Mat resizedImage, floatImage;
    cv::cvtColor(image, resizedImage, cv::COLOR_BGR2RGB);
    YOLOutils::letterbox(resizedImage, resizedImage, cv::Size((int)this->inputShapes[0][2], (int)this->inputShapes[0][3]),
        cv::Scalar(114, 114, 114), this->isDynamicInputShape,
        false, true, 32);

    inputTensorShape[2] = resizedImage.rows;
    inputTensorShape[3] = resizedImage.cols;

    resizedImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);
    blob = new float[floatImage.cols * floatImage.rows * floatImage.channels()];
    cv::Size floatImageSize{ floatImage.cols, floatImage.rows };

    // hwc -> chw
    std::vector<cv::Mat> chw(floatImage.channels());
    for (int i = 0; i < floatImage.channels(); ++i)
    {
        chw[i] = cv::Mat(floatImageSize, CV_32FC1, blob + i * floatImageSize.width * floatImageSize.height);
    }
    cv::split(floatImage, chw);
}

std::vector<Yolov11Result> YOLOPredictor::postprocessing(const cv::Size& resizedImageShape,
    const cv::Size& originalImageShape,
    std::vector<Ort::Value>& outputTensors)
{

    // for box
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    std::vector<int> classIds;

    float* boxOutput = outputTensors[0].GetTensorMutableData<float>();
    //[1,4+n,8400]=>[1,8400,4+n] or [1,4+n+32,8400]=>[1,8400,4+n+32]
    cv::Mat output0 = cv::Mat(cv::Size((int)this->outputShapes[0][2], (int)this->outputShapes[0][1]), CV_32F, boxOutput).t();
    float* output0ptr = (float*)output0.data;
    int rows = (int)this->outputShapes[0][2];
    int cols = (int)this->outputShapes[0][1];
    // std::cout << rows << cols << std::endl;
    // if hasMask
    std::vector<std::vector<float>> picked_proposals;
    cv::Mat mask_protos;

    for (int i = 0; i < rows; i++)
    {
        std::vector<float> it(output0ptr + i * cols, output0ptr + (i + 1) * cols);
        float confidence;
        int classId;
        this->getBestClassInfo(it.begin(), confidence, classId, classNums);

        if (confidence > this->confThreshold)
        {
            if (this->hasMask)
            {
                std::vector<float> temp(it.begin() + 4 + classNums, it.end());
                picked_proposals.push_back(temp);
            }
            int centerX = (int)(it[0]);
            int centerY = (int)(it[1]);
            int width = (int)(it[2]);
            int height = (int)(it[3]);
            int left = centerX - width / 2;
            int top = centerY - height / 2;
            boxes.emplace_back(left, top, width, height);
            confs.emplace_back(confidence);
            classIds.emplace_back(classId);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, this->confThreshold, this->iouThreshold, indices);

    if (this->hasMask)
    {
        float* maskOutput = outputTensors[1].GetTensorMutableData<float>();
        std::vector<int> mask_protos_shape = { 1, (int)this->outputShapes[1][1], (int)this->outputShapes[1][2], (int)this->outputShapes[1][3] };
        mask_protos = cv::Mat(mask_protos_shape, CV_32F, maskOutput);
    }

    std::vector<Yolov11Result> results;
    for (int idx : indices)
    {
        Yolov11Result res;
        res.box = cv::Rect(boxes[idx]);
        if (this->hasMask)
            res.boxMask = this->getMask(cv::Mat(picked_proposals[idx]).t(), mask_protos);
        else
            res.boxMask = cv::Mat::zeros((int)this->inputShapes[0][2], (int)this->inputShapes[0][3], CV_8U);

        YOLOutils::scaleCoords(res.box, res.boxMask, this->maskThreshold, resizedImageShape, originalImageShape);
        res.conf = confs[idx];
        res.classId = classIds[idx];
        results.emplace_back(res);
    }

    return results;
}

std::vector<Yolov11Result> YOLOPredictor::predict(cv::Mat& image)
{
    float* blob = nullptr;
    std::vector<int64_t> inputTensorShape{ 1, 3, -1, -1 };
    this->preprocessing(image, blob, inputTensorShape);

    size_t inputTensorSize = YOLOutils::vectorProduct(inputTensorShape);

    std::vector<float> inputTensorValues(blob, blob + inputTensorSize);

    std::vector<Ort::Value> inputTensors;

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    inputTensors.push_back(Ort::Value::CreateTensor<float>(
        memoryInfo, inputTensorValues.data(), inputTensorSize,
        inputTensorShape.data(), inputTensorShape.size()));
    std::vector<Ort::Value> outputTensors = this->session.Run(Ort::RunOptions{ nullptr },
        this->inputNames.data(),
        inputTensors.data(),
        1,
        this->outputNames.data(),
        this->outputNames.size());

    cv::Size resizedShape = cv::Size((int)inputTensorShape[3], (int)inputTensorShape[2]);
    std::vector<Yolov11Result> result = this->postprocessing(resizedShape,
        image.size(),
        outputTensors);

    delete[] blob;

    return result;
}

/// <summary>
/// 释放内存
/// </summary>
/// <param name=""></param>
void YOLOPredictor::Free(void)
{
    this->isInit = false;
}

void YOLOPredictor::modelToHeat(void)
{
    cv::Mat image(2448, 2048, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < 5; i++)
    {
        predict(image);
    }
}
