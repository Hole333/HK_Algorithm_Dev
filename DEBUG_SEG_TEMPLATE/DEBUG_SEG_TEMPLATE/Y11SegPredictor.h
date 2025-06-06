#pragma once
#ifndef __Y11SEGPREDICTOR_H
#define __Y11SEGPREDICTOR_H

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <utility>
#include "utils.h"
#include <iostream>

class YOLOPredictor
{
public:
    YOLOPredictor() {};
    ~YOLOPredictor() {};
    void YOLOSegInit(const std::string& modelPath,
        const bool& isGPU,
        float confThreshold,
        float iouThreshold,
        float maskThreshold);
    void getParam(float confThreshold, float iouThreshold, float maskThreshold);
    void Free(void);
    void modelToHeat(void);
    std::vector<Yolov11Result> predict(cv::Mat& image);
    int classNums = 2;
    bool isInit = false;

private:
    Ort::Env env{ nullptr };
    Ort::SessionOptions sessionOptions{ nullptr };
    Ort::Session session{ nullptr };

    void preprocessing(cv::Mat& image, float*& blob, std::vector<int64_t>& inputTensorShape);
    std::vector<Yolov11Result> postprocessing(const cv::Size& resizedImageShape,
        const cv::Size& originalImageShape,
        std::vector<Ort::Value>& outputTensors);

    static void getBestClassInfo(std::vector<float>::iterator it,
        float& bestConf,
        int& bestClassId,
        const int _classNums);
    cv::Mat getMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos);
   
    bool isDynamicInputShape{};
    
    std::vector<const char*> inputNames;
    std::vector<Ort::AllocatedStringPtr> input_names_ptr;

    std::vector<const char*> outputNames;
    std::vector<Ort::AllocatedStringPtr> output_names_ptr;

    std::vector<std::vector<int64_t>> inputShapes;
    std::vector<std::vector<int64_t>> outputShapes;
    float confThreshold = 0.3f;
    float iouThreshold = 0.4f;

    bool hasMask = false;
    float maskThreshold = 0.5f;
};




















#endif