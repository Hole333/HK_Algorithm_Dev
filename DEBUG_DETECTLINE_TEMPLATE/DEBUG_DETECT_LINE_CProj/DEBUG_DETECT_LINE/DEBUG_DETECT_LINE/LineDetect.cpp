#include "LineDetect.h"
#include <iostream>
#include <float.h>
#include <cmath>
#include <algorithm> // ����std::max

using namespace std;

LineDetectClass::LineDetectClass()
{

}


LineDetectClass::~LineDetectClass()
{

}






/// <summary>
/// ���sin
/// </summary>
/// <param name="a"></param>
/// <returns></returns>
inline float LineDetectClass::_sin(float a) {
    if (a < _PI_2) {
        return 0.0001 * sine_array[_round(126.6873 * a)];
    }
    else if (a < _PI) {
        return 0.0001 * sine_array[398 - _round(126.6873 * a)];
    }
    else if (a < _3PI_2) {
        return -0.0001 * sine_array[-398 + _round(126.6873 * a)];
    }
    else {
        return -0.0001 * sine_array[796 - _round(126.6873 * a)];
    }
}

/// <summary>
/// ���cos
/// </summary>
/// <param name="a"></param>
/// <returns></returns>
inline float LineDetectClass::_cos(float a) {
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}



/// <summary>
/// ��ȡ��ת��ĽǶ�
/// </summary>
/// <param name="rads"></param>
/// <returns></returns>
float LineDetectClass::getRotateRectAngle(float angle)
{
    if (angle >= 0 && angle <= 90)
    {
        return (90.0f - angle);
    }
    else if (angle > 90 && angle <= 180)
    {
        return (270.0f - angle);
    }
    else if (angle >= -90 && angle < 0)
    {
        return (90 + fabs(angle));
    }
    else if (angle <= -90 && angle >= -180)
    {
        return (fabs(angle) - 90);
    }
}




/// <summary>
/// ���ٿ�����
/// </summary>
/// <param name="number"></param>
/// <returns></returns>
float LineDetectClass::Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long*)&y; // evil floating point bit level hacking 
    i = 0x5f3759df - (i >> 1); // what the fuck? 
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration 
    // y  = y * ( threehalfs - ( x2 * y * y ) ); // 2nd iteration, this can be removed 

#ifndef Q3_VM 
#ifdef __linux__ 
    assert(!isnan(y)); // bk010122 - FPE? 
#endif 
#endif 
    return (1.0f / y);
}

bool areCollinear(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, double epsilon = 1e-6) {
    // 1. ����������Ƿ����غϵ㣨��Ծ���ƽ����ֵ��
    const double epsilon_sq = epsilon * epsilon;

    // ���AB����ƽ��
    const double ab_dist_sq = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    if (ab_dist_sq < epsilon_sq) return true;  // A��B�غ�

    // ���AC����ƽ��
    const double ac_dist_sq = (c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y);
    if (ac_dist_sq < epsilon_sq) return true;  // A��C�غ�

    // ���BC����ƽ��
    const double bc_dist_sq = (c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y);
    if (bc_dist_sq < epsilon_sq) return true;  // B��C�غ�

    // 2. ������ƽ�������⸡�����ľ���ֵ��
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    const double cross_sq = cross * cross;

    // 3. ������������ƽ���˻�
    const double ab_len_sq = ab_dist_sq;  // ����ǰ�����
    const double ac_len_sq = ac_dist_sq;  // ����ǰ�����
    const double len_product = ab_len_sq * ac_len_sq;

    // 4. �������жϣ���������㣩
    return cross_sq <= epsilon_sq * len_product;
}


// �ж������Ƿ���ͬһ��ֱ���ϣ�ʹ�����������
float arePointsCollinear(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3) 
{
    Point2f vector1 = Point2f(p2.x - p1.x, p2.y - p1.y);
    Point2f vector2 = Point2f(p3.x - p1.x, p3.y - p1.y);
    float crossProduct = vector1.x * vector2.y - vector1.y * vector2.x;
    //double K, B;
    //calculateLineParams(p1, p2, K, B);

    return fabs(crossProduct);  // �������ӽ��㣬˵�����㹲��
}


void drawCross(cv::Mat& image, cv::Point center, int lineLength, cv::Scalar color, int thickness) {
    // ����ˮƽ��
    cv::line(image, cv::Point(center.x - lineLength, center.y), cv::Point(center.x + lineLength, center.y), color, thickness);

    // ���ƴ�ֱ��
    cv::line(image, cv::Point(center.x, center.y - lineLength), cv::Point(center.x, center.y + lineLength), color, thickness);
}

// ����������֮��ľ���
float distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// �����������㶨��ĽǶȣ������ǵ�һ����
float LineDetectClass::calculateAngleUsingCosineTheorem(const cv::Point2f& Point1, const cv::Point2f& Point2) {

    Point2f A, B, C;
    float distance1 = distance(Point1, Point2f(cx, cy));
    float distance2 = distance(Point2, Point2f(cx, cy));
    
    if (distance1 < distance2)
    {
        A = Point1;
        B = Point2f(Point1.x + 1.0f, Point1.y);
        C = Point2;
    }
    else
    {
        A = Point2;
        B = Point2f(Point2.x + 1.0f, Point2.y);
        C = Point1;
    }

    
    // ���������ߵĳ���
    float a = distance(B, C); // BC
    float b = distance(A, B); // AB
    float c = distance(A, C); // AC

    // ���Ҷ���cos(��BAC) = (b^2 + c^2 - a^2) / (2bc)
    float cosAngle = (b * b + c * c - a * a) / (2 * b * c);

    // ����cosֵ�ķ�Χ���Է��������³���[-1, 1]�ķ�Χ
    if (cosAngle > 1.0f) cosAngle = 1.0f;
    if (cosAngle < -1.0f) cosAngle = -1.0f;

    // ����нǣ����ȣ�
    float angleRad = std::acos(cosAngle);
    float angle = angleRad * 180.0f / _PI;

    if (A.y < C.y)
    {
        angle = -angle;
    }


    // ���ؼн�
    return angle;
}

void calculateLineParams(Point2d p1, Point2d p2, double& k, double& b) {
    // ����б��k
    k = (p2.y - p1.y) / (p2.x - p1.x);
    // ����ؾ�b
    b = p1.y - k * p1.x;
}

Point2f findCircleLineIntersections(Point2d center, double radius, double k, double b, Point2f base) {
    Point2f intersections;

    // ������η��̵�ϵ��
    double A = 1 + k * k;
    double B = 2 * (k * (b - center.y) - center.x);
    double C = center.x * center.x + (b - center.y) * (b - center.y) - radius * radius;

    // �����б�ʽ
    double discriminant = B * B - 4 * A * C;

    if (discriminant < 0) {
        // û�н���
        cout << "No intersection." << endl;
        return intersections;
    }

    // ���㽻��
    double x1 = (-B + sqrt(discriminant)) / (2 * A);
    double x2 = (-B - sqrt(discriminant)) / (2 * A);

    // ��ÿ��xֵ�������Ӧ��yֵ
    double y1 = k * x1 + b;
    double y2 = k * x2 + b;


    Point2f p1(x1, y1);

    if (discriminant > 0) {
        Point2f p2(x2, y2);
        float distancePoint2OrgPoint = distance(p1, base);
        float distancePoint2OrgPoint2 = distance(p2, base);
        if (distancePoint2OrgPoint < distancePoint2OrgPoint2)
        {
            return p1;
        }
        else
        {
            return p2;
        }
    }




    return intersections;
}



float getMaxTriangleAngle(const cv::Point2f& a,
    const cv::Point2f& b,
    const cv::Point2f& c,
    float epsilon = 1e-6f)
{
    // 1. ��������Ƿ��˻��������غϻ����㹲�ߣ�
    const float ab_dist_sq = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    const float ac_dist_sq = (c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y);
    const float bc_dist_sq = (c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y);

    // ���������غϵ����
    if (ab_dist_sq < epsilon || ac_dist_sq < epsilon || bc_dist_sq < epsilon) {
        return CV_PI; // ����180�ȣ��˻������
    }

    // 2. ���������ߵ�����
    const cv::Point2f ab = b - a;
    const cv::Point2f ac = c - a;
    const cv::Point2f bc = c - b;

    // 3. ���������Ƕȣ�ʹ�����Ҷ���
    const auto calcAngle = [](const cv::Point2f& v1, const cv::Point2f& v2) {
        const float dot = v1.dot(v2);
        const float norm1 = cv::norm(v1);
        const float norm2 = cv::norm(v2);
        if (norm1 < 1e-6f || norm2 < 1e-6f) return 0.0f;
        return std::acos(dot / (norm1 * norm2));
    };

    // ���������ǣ���λ�����ȣ�
    const float angleA = calcAngle(-ab, ac);       // ��A��ab��ac�ļн�
    const float angleB = calcAngle(ab, bc);        // ��B��ab��bc�ļн�
    const float angleC = CV_PI - angleA - angleB;  // ��C��ͨ���������ڽǺͼ���

    // 4. �������Ƕȣ���λ�����ȣ�
    return std::max({ angleA, angleB, angleC });
}



float evaluateCollinearity(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, double epsilon = 1e-12) {
    // 1. ����Ƿ�����غϵ㣨ֱ�ӷ���0��
    const double threshold_sq = epsilon * epsilon;
    if (cv::norm(b - a) * cv::norm(b - a) < threshold_sq) return 0.0f; // AB�غ�
    if (cv::norm(c - a) * cv::norm(c - a) < threshold_sq) return 0.0f; // AC�غ�
    if (cv::norm(c - b) * cv::norm(c - b) < threshold_sq) return 0.0f; // BC�غ�

    // 2. ������ƽ�������Ĺ�����ָ�꣩
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    const double cross_sq = cross * cross;

    // 3. ������������ƽ���˻�����һ�����ӣ�
    const double ab_len_sq = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    const double ac_len_sq = (c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y);

    // 4. ���ع�һ�����ֵ������һ���ԣ�
    return static_cast<float>(cross_sq / (ab_len_sq * ac_len_sq + epsilon));
}

/// <summary>
/// ���߷��� ʹ�û���任
/// </summary>
/// <param name=""></param>
/// <returns>������ת���εĽǶ�</returns> 
float LineDetectClass::checkIsLine_HOUGH(void)
{
    Canny(copyMat, copyMat, CannyThreshold_LOW, CannyThreshold_HIGH); // Ӧ��Canny��Ե����㷨
    //Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //dilate(copyMat, copyMat, element);
    std::vector<Vec4i> lines; // �洢��⵽��ֱ��
    HoughLinesP(copyMat, lines, 1, CV_PI / 180, Threshold, (r_out - r_in) * minLength, (r_out - r_in) * maxLineGap); // Ӧ�û���ֱ�߼���㷨
#ifdef ENABLE_DEBUG
    resize(show, show, Size(2448, 2048));
    putText(show,
        format("Threshold:%d MinL:%.2f  MaxL:%.2f MaxG:%2f Clow:%d CHigh:%d cx: %d ,cy:%d", 
                Threshold, minLength, maxLineGap, maxLineLength, CannyThreshold_LOW, CannyThreshold_HIGH, cx * 4, cy *4 ),
        Point(1000, 100),
        FONT_HERSHEY_COMPLEX,
        0.8,
        Scalar(0, 255, 0), 2, LINE_8);
    uint16_t y2 = 20;
    uint16_t y = 20;
    putText(show,
        format("maxlength:%.4f minlength�� %.2f Lengh:%d", (maxLineLength * (r_out - r_in)), (minLength * (r_out - r_in)), r_out - r_in),
            Point(1000, 600),
            FONT_HERSHEY_COMPLEX,
            0.81,
            Scalar(0, 255, 255), 2, LINE_8);
    drawCross(show, Point(cx * 4, cy * 4), 5, Scalar(0, 0, 255), 5);
    line(show, Point(cx * 4, cy * 4), Point((cx + r_in) * 4, cy * 4), Scalar(255, 255, 0), 3, LINE_AA);
    line(show, Point(cx * 4, cy * 4), Point(cx * 4, (cy + r_out) * 4), Scalar(255, 255, 0), 3, LINE_AA);
#endif // ENABLE_DEBUG
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        float powLength = (_pow2(l[2] - l[0])) + _pow2((l[3] - l[1])); // δ����������
        float LineLength = Q_rsqrt(powLength); // ��ȡ��ǰ�߶γ���
#ifdef ENABLE_DEBUG
        //putText(show,
        //    format("length:%.4f ", LineLength),
        //    Point(300, y2),
        //    FONT_HERSHEY_COMPLEX,
        //    0.7,
        //    Scalar(0, 0, 255), 2, LINE_8);
        // line(show, Point(l[0] * 4, l[1] * 4), Point(l[2] * 4, l[3] * 4), Scalar(0, 0, 255), 3, LINE_AA);
       
#endif
        if (LineLength <= (maxLineLength * (r_out - r_in)) && LineLength >= (minLength * (r_out - r_in)))
        {
            //float avgSigma = fabs((sigma1 + sigma2 + sigma3) / 3.0f);
            float avgSigma = getMaxTriangleAngle(Point2f((float)cx, (float)cy), Point2f(l[0], l[1]), Point2f(l[2], l[3])) * 180.0f / CV_PI; 
            // �����Ѿ��ҵ�����ߣ� avgSigma �������������Բ�����������ֱ����ϳ̶ȣ����Խ�� �������߹���
            if (fabs(avgSigma - 180) < 10)
            {
                // ���������arccos �ýǶ� �Ƕȷ�Χ -pi ~ pi 
                float Theata = calculateAngleUsingCosineTheorem(Point(l[0], l[1]), Point(l[2], l[3])); // ����
                DetectedLine = l;
#ifdef ENABLE_DEBUG
                putText(show,
                    format("detectLength: %.2f maxLine: %.2f",
                        LineLength, (maxLineLength * (r_out - r_in))),
                    Point(1000, 150),
                    FONT_HERSHEY_COMPLEX,
                    0.8,
                    Scalar(0, 255, 0), 2, LINE_8);
                line(show, Point(l[0] * 4, l[1] * 4), Point(l[2] * 4, l[3] * 4), Scalar(0, 255, 0), 3, LINE_AA);
                putText(show,
                    format("sigma:%.4f ", avgSigma),
                    Point(30, y2),
                    FONT_HERSHEY_COMPLEX,
                    0.7,
                    Scalar(0, 255, 0), 2, LINE_8);
                //putText(show,
                //    format("angle:%.4f ", Theata),
                //    Point(30, y2),
                //    FONT_HERSHEY_COMPLEX,
                //    0.7,
                //    Scalar(0, 255, 0), 2, LINE_8);
#endif

               return Theata;
            }
            else
            {
#ifdef ENABLE_DEBUG
                line(show, Point(l[0] * 4, l[1] * 4), Point(l[2] * 4, l[3] * 4), Scalar(0, 0, 255), 3, LINE_AA);
                putText(show,
                    format("sigma:%f ", avgSigma),
                    Point(30, y2),
                    FONT_HERSHEY_COMPLEX,
                    0.7,
                    Scalar(0, 0, 255), 2, LINE_8);
#endif
            }
        }
#ifdef ENABLE_DEBUG
        y2 += 30;
#endif
        
    }
    return NO_LINE; // ����û�м�⵽�����
}

/// <summary>
/// ������Ӧ�Ƕȵ�ROI��ת��
/// </summary> 
/// <param name="">rotateAngle: �������������ˮƽ�ߵĽǶ�</param>
/// <returns></returns>
RotatedRect LineDetectClass::createROIRotatedRect(float rotateAngle)
{
    float rads = rotateAngle * _PI / 180.0f;    //float t = std::cos(rads);
    //float t2 = std::sin(rads);

    int startX = (int)(cx + r_in * std::cos(rads));
    int startY = (int)(cy + r_in * (-std::sin(rads)));

    int endX = (int)(cx + r_out * std::cos(rads));
    int endY = (int)(cy + r_out * (-std::sin(rads)));

    int rect_cx = (int)((startX + endX) / 2.0f); // ȷ����ת�������X
    int rect_cy = (int)((startY + endY) / 2.0f); // ȷ����ת�������Y


    int rect_cx2 = (int)((DetectedLine[0] + DetectedLine[2]) / 2.0f);
    int rect_cy2 = (int)((DetectedLine[1] + DetectedLine[3]) / 2.0f);
    int distance2 = distance(Point2f(DetectedLine[0], DetectedLine[1]), Point2f(DetectedLine[2], DetectedLine[3]));

    double k, b;
    calculateLineParams(Point2f(DetectedLine[0], DetectedLine[1]), Point2f(DetectedLine[2], DetectedLine[3]), k, b);
    Point2f innerPoint = findCircleLineIntersections(Point2d(cx, cy), r_in, k, b, Point2f(rect_cx2, rect_cy2));
    Point2f outPoint = findCircleLineIntersections(Point2d(cx, cy), r_out, k, b, Point2f(rect_cx2, rect_cy2));

    Point2f fixedCenterPoint((innerPoint.x + outPoint.x) / 2.0f * 4.0f, (innerPoint.y + outPoint.y) / 2.0f * 4.0f);

    float ROI_Angle = getRotateRectAngle(rotateAngle);
    RotatedRect ROI = RotatedRect(fixedCenterPoint, Size2f(rotateRectWidth, rotateRectHeight), ROI_Angle);//(r_out - r_in) * 1.40)

    return ROI;
}

/// <summary>
/// �ⲿ���²���
/// </summary>
/// <param name="c_x"></param>
/// <param name="c_y"></param>
/// <param name="rIn"></param>
/// <param name="rOut"></param>
void LineDetectClass::updateInputParam(int c_x, int c_y, int rIn, int rOut)
{
    cx = c_x;
    cy = c_y;
    r_in = rIn;
    r_out = rOut;
}

/// <summary>
/// �������в���
/// </summary> 
/// <param name="threshold"></param>  ��ֵΪ����߶�ռ�ܳ��ٷֱ� ��Χ(0.00 -- 1.00)
/// <param name="Step"></param>
/// <param name="linePixel"></param>
void LineDetectClass::updateRunningParam(int InputThreshold, float InputMinLength, float InputMaxLineLength, float InputMaxLineGap, int ROI_Width, int ROI_Height, int InputCannyLow, int InputCannyHigh)
{
    Threshold = InputThreshold;
    minLength = InputMinLength;
    maxLineLength = InputMaxLineLength;
    maxLineGap = InputMaxLineGap;

    rotateRectWidth = ROI_Width;
    rotateRectHeight = ROI_Height;

    CannyThreshold_LOW = InputCannyLow;
    CannyThreshold_HIGH = InputCannyHigh;
}

/// <summary>
/// �����ֱ���㷨
/// </summary>
/// <param name="mat"></param>
/// <returns></returns>
RotatedRect LineDetectClass::detect(Mat& mat)
{
    copyMat = mat; // ���ƴ���ͼ���ڲ�˽�г�Ա
    RotatedRect ROI; 
#ifdef ENABLE_DEBUG
    show = mat;
    cvtColor(show, show, COLOR_GRAY2BGR);
#endif
    float getAngle = checkIsLine_HOUGH();
    if (getAngle != NO_LINE)
    {
        ROI = createROIRotatedRect(getAngle);
    }
    else
    {
        ROI = RotatedRect(Point2f(0, 0), Size2f(rotateRectWidth, rotateRectHeight), 0);
    }

    
#ifdef ENABLE_DEBUG
    namedWindow("test", WINDOW_FREERATIO);
    imshow("test", show);
    waitKey(0);
#endif
    // û���ҵ��߶���Ĭ�ϵĲ���ת�Ƕ�
    
    return ROI;
}




//for (float angle = 0; angle < CIRCLE_ANGLE; angle += 1.0)
//{
//    float rad = (float)angle * (_PI / 180.0f);
//    // ������ֱ��
//    if (checkIsLine_HOUGH())
//    {
//        return createROIRotatedRect(angle);
//    }
//}



/// <summary>
/// ����Ƿ�����Ƕ��ж�Ӧ�Ĳ���� �ο�VM�����߶μ�ⷽ��
/// </summary>
/// <param name="rads"></param>
/// <returns></returns>
//bool LineDetectClass::checkIsLineV2(float rads)
//{
//    // �ýǶ����
//    int currX = (int)(cx + r_in * _cos(rads));
//    int currY = (int)(cy + r_in * _sin(rads));
//    int LinePointNum = 0; // ����߶�Ӧ�ĵ���
//    std::vector<Point> LinePoint; // �洢���õı�Ե��
//    //// ���ýǶȵ�����
//    for (int lineOffset = 0; lineOffset <= (r_out - r_in); lineOffset += step)
//    {
//        currX = (int)(cx + (r_in + lineOffset) * _cos(rads));
//        currY = (int)(cy + (r_in + lineOffset) * _sin(rads));
//
//        int startX = currX - rotateRectWidth / 2;                                                                 
//        int startY = currY - ((float)rotateRectWidth / 2.0f) * ((float)(currX - cx) / (float)(currY - cy));
//        for (int checkLineOffset = 0; checkLineOffset <= rotateRectWidth; checkLineOffset++)
//        {
//
//
//        }
//    }
//
//    //// ����߶ζ�Ӧ�����Ƿ������ֵ
//    if (LinePointNum >= Threshold)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}

///// <summary>
///// ����Ƿ�����Ƕ��ж�Ӧ�Ĳ����
///// </summary>
///// <param name="rads"></param>
///// <returns></returns>
//bool LineDetectClass::checkIsLine(float rads)
//{
//    // �ýǶ����
//    int currX = (int)(cx + r_in * _cos(rads)); 
//    int currY = (int)(cy + r_in * _sin(rads));
//    int LinePointNum = 0; // ����߶�Ӧ�ĵ���
//    // ���ýǶȵ�����
//    for (int lineOffset = 0; lineOffset <= (r_out - r_in); lineOffset += step)
//    {
//        currX = (int)(cx + (r_in + lineOffset) * _cos(rads));
//        currY = (int)(cy + (r_in + lineOffset) * _sin(rads));
//        uint8_t pixel = copyMat.at<uchar>(currY, currX); // ��ȡ��Ӧλ�õ�����ֵ
//        if (pixel == targetLinePixel)
//        {
//            LinePointNum++;
//#ifdef ENABLE_DEBUG
//            circle(show, Point(currX, currY), 0, Scalar(0, 255, 0), 2);
//#endif
//        }
//#ifdef ENABLE_DEBUG
//        else
//        {
//            circle(show, Point(currX, currY), 0, Scalar(0, 0, 255), 0);
//        }
//        //waitKey(0);
//#endif
//    }
//
//
//
//    // ����߶ζ�Ӧ�����Ƿ������ֵ
//    if (LinePointNum >= Threshold)
//    {
//        return true;
//    }
//    else
//    { 
//        return false;
//    }
//}
