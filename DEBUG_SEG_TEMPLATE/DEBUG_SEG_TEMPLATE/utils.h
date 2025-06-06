#pragma once
#ifndef __UTILS_H
#define __UTILS_H

#include <codecvt>
#include <fstream>
#include <opencv2/opencv.hpp>



struct Yolov11Result
{
    cv::Rect box;
    cv::Mat boxMask; // mask in box
    float conf{};
    int classId{};
};


namespace YOLOutils
{
    static std::vector<cv::Scalar> colors;

    size_t vectorProduct(const std::vector<int64_t>& vector);
    std::wstring charToWstring(const char* str);
    std::vector<std::string> loadNames(const std::string& path);
    void visualizeDetection(cv::Mat& image, std::vector<Yolov11Result>& results,
        const std::vector<std::string>& classNames);
    cv::Mat getBinaryMask(cv::Mat& img, std::vector<Yolov11Result>& results);
    void letterbox(const cv::Mat& image, cv::Mat& outImage,
        const cv::Size& newShape,
        const cv::Scalar& color,
        bool auto_,
        bool scaleFill,
        bool scaleUp,
        int stride);

    void scaleCoords(cv::Rect& coords, cv::Mat& mask,
        const float maskThreshold,
        const cv::Size& imageShape, const cv::Size& imageOriginalShape);

    template <typename T>
    T clip(const T& n, const T& lower, const T& upper);
}











#endif
