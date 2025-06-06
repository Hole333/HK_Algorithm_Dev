#pragma once
#ifndef __LINEDETECT_H
#define __LINEDETECT_H

#include <opencv2/opencv.hpp>

#define DEBUG 0
#if DEBUG
#define ENABLE_DEBUG
#endif

using namespace cv;


#define WHITE  255
#define BLACK  0
#define CIRCLE_ANGLE 360
#define _PI_2  1.57079632679
#define _PI    3.14159265359
#define _3PI_2 4.71238898038
#define _2PI   6.28318530718
#define NO_LINE -99999.999 

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
#define _pow2(x) ((x) * (x))

class LineDetectClass
{

public:
#ifdef ENABLE_DEBUG
    Mat show;
#endif
    Mat OutputMask;
    Mat mark;
    Mat copyMat;
    Mat coverMat; // 存储提取到的图像

	LineDetectClass();
	~LineDetectClass();
    void updateInputParam(int c_x, int c_y, int rIn, int rOut);
    void updateRunningParam(int InputThreshold, float InputMinLength, float InputMaxLineLength, float InputMaxLineGap, int ROI_Width, int ROI_Height, int InputCannyLow, int InputCannyHigh);
	RotatedRect detect(Mat& mat);

private:



	int cx;  // 待检测中心X
	int cy;  // 待检测中心Y
	int r_in;  // 内径
	int r_out;  // 外径

    int Threshold; // 阈值
    float minLength; // 最小检测长度
    float maxLineGap; // 最大跨越距离
    float maxLineLength; // 最长线长

    int CannyThreshold_LOW; // canny低阈值
    int CannyThreshold_HIGH; // canny高阈值

    int rotateRectWidth;  // 划分旋转框的宽度
    int rotateRectHeight; // 划分旋转框的高度
    Vec4i DetectedLine;   // 检测到的侧缝线

    bool checkIsLine(float rads);
    bool checkIsLineV2(float rads);
    float checkIsLine_HOUGH(void);
    float getRotateRectAngle(float angle);
    RotatedRect createROIRotatedRect(float rads);
    float calculateAngleUsingCosineTheorem(const cv::Point2f& Point1, const cv::Point2f& Point2);
    float Q_rsqrt(float number);
    const int sine_array[200] = {
    0, 79, 158, 237, 316, 395, 473, 552, 631, 710,
    789, 867, 946, 1024, 1103, 1181, 1260, 1338, 1416, 1494,
    1572, 1650, 1728, 1806, 1883, 1961, 2038, 2115, 2192, 2269,
    2346, 2423, 2499, 2575, 2652, 2728, 2804, 2879, 2955, 3030,
    3105, 3180, 3255, 3329, 3404, 3478, 3552, 3625, 3699, 3772,
    3845, 3918, 3990, 4063, 4135, 4206, 4278, 4349, 4420, 4491,
    4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046, 5113, 5181,
    5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839,
    5903, 5967, 6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461,
    6521, 6581, 6640, 6699, 6758, 6815, 6873, 6930, 6987, 7043,
    7099, 7154, 7209, 7264, 7318, 7371, 7424, 7477, 7529, 7581,
    7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977, 8025, 8072,
    8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512,
    8553, 8594, 8634, 8673, 8712, 8751, 8789, 8826, 8863, 8899,
    8935, 8970, 9005, 9039, 9072, 9105, 9138, 9169, 9201, 9231,
    9261, 9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481, 9506,
    9530, 9554, 9577, 9599, 9621, 9642, 9663, 9683, 9702, 9721,
    9739, 9757, 9774, 9790, 9806, 9821, 9836, 9850, 9863, 9876,
    9888, 9899, 9910, 9920, 9930, 9939, 9947, 9955, 9962, 9969,
    9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000
    };
    float _sin(float a);
    float _cos(float a);
};

#endif
