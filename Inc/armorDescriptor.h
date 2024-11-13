#ifndef ARMORDESCRIPTOR_H
#define ARMORDESCRIPTOR_H

#include <LightDescriptor.h>
#include <armorParam.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
#define BIG_ARMOR 1
#define SMALL_ARMOR 0
#define UNKNOWN_ARMOR -1

class ArmorDescriptor
{
public:
    // 默认构造函数
    ArmorDescriptor();

    // 带参数的构造函数
    ArmorDescriptor(const LightDescriptor& lLight, const LightDescriptor& rLight, const int armorType, const cv::Mat& srcImg, const float rotationScore, ArmorParam param);

    // 清除所有信息，包括顶点信息
    void clear()
    {
        rotationScore = 0;
        sizeScore = 0;
        distScore = 0;
        finalScore = 0;
        for(int i = 0; i < 4; i++)
        {
            vertex[i] = cv::Point2f(0, 0);
        }
        type = UNKNOWN_ARMOR;
    }

    // 获取前置图像
    void getFrontImg(const cv::Mat& grayImg);
    
    // 计算装甲板的测量数据
    cv::Mat measure(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);

public:
    // 成员变量
    std::array<cv::RotatedRect, 2> lightPairs;  // 存放灯条的旋转矩形
    float sizeScore;  // 大小评分
    float distScore;  // 距离评分
    float rotationScore;  // 旋转评分
    float finalScore;  // 最终评分
    double distance;  // 距离
    cv::Mat coordinate;  // 坐标数据
    std::vector<cv::Point2f> vertex;  // 装甲板的四个顶点
    std::vector<cv::Point2d> points;  // 装甲板中点的集合
    cv::Mat frontImg;  // 前视图像
    int type;  // 装甲板类型

};

#endif
