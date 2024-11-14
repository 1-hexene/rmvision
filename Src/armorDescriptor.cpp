#include "armorDescriptor.h"


ArmorDescriptor::ArmorDescriptor()// 默认构造函数，初始化armorDescriptor的一些基本属性
{
    rotationScore = 0;
    sizeScore = 0;
    vertex.resize(4);
    for(int i = 0; i < 4; i++)
    {
        vertex[i] = cv::Point2f(0, 0);
    }
    type = UNKNOWN_ARMOR;
}

ArmorDescriptor::ArmorDescriptor(const LightDescriptor & lLight, const LightDescriptor & rLight, const int armorType, const cv::Mat & grayImg, float rotaScore, ArmorParam _param)
{
    // 将左右灯条的旋转矩形的信息储存在lightPairs中
    lightPairs[0] = lLight.rec();
    lightPairs[1] = rLight.rec();
    // 创建更大的矩形区域，方便后续处理
    cv::Size exLSize(int(lightPairs[0].size.width), int(lightPairs[0].size.height * 2));
    cv::Size exRSize(int(lightPairs[1].size.width), int(lightPairs[1].size.height * 2));
    cv::RotatedRect exLLight(lightPairs[0].center, exLSize, lightPairs[0].angle);
    cv::RotatedRect exRLight(lightPairs[1].center, exRSize, lightPairs[1].angle);

    cv::Point2f pts_l[4];
    exLLight.points(pts_l);// 将左灯条的四个顶点赋值给pts_l
    cv::Point2f upper_l = pts_l[2];
    cv::Point2f lower_l = pts_l[3];

    cv::Point2f pts_r[4];
    exRLight.points(pts_r);
    cv::Point2f upper_r = pts_r[1];
    cv::Point2f lower_r = pts_r[0];

    vertex.resize(4); // 创建一个储存四个角点的容器
    vertex[0] = upper_l;
    vertex[1] = upper_r;
    vertex[2] = lower_r;
    vertex[3] = lower_l;


    type = armorType;
    getFrontImg(grayImg);

    rotationScore = rotaScore;


    float normalized_area = contourArea(vertex) / _param.area_normalized_base; // 指定多边形面积/1000，归一化面积计算
    sizeScore = exp(normalized_area); // 计算normalized_area评分 exp函数能将较小的面积比例映射到较大的评分值，从而强调较小装甲板的重要性


}

// 提取roi区域，方便后续使用SVM或者模板匹配
void ArmorDescriptor::getFrontImg(const Mat& grayImg)
{

}

cv::Mat ArmorDescriptor::measure(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    std::vector<cv::Point3d> objectPoints;
    objectPoints.push_back(cv::Point3d(-65, 28, 0));
    objectPoints.push_back(cv::Point3d(-65, -28, 0));
    objectPoints.push_back(cv::Point3d(65, 28, 0));
    objectPoints.push_back(cv::Point3d(65, -28, 0));

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat rmat;
    solvePnP(objectPoints, vertex, cameraMatrix, distCoeffs, rvec, tvec);
    Rodrigues(rvec, rmat);

    cv::Mat armor_cam_matrix;
    cv::Mat supplement = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::Mat zero_coordinate = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    hconcat(rmat, tvec, armor_cam_matrix);
    vconcat(armor_cam_matrix, supplement, armor_cam_matrix);

    this -> coordinate = armor_cam_matrix * zero_coordinate;
    cv::Mat distance_square = coordinate.t() * coordinate;
    this -> distance = std::sqrt(distance_square.at<double>(0, 0));

    return this -> coordinate;
}

void ArmorDescriptor::show(cv::Mat& img, cv::Scalar color) {
    std::vector<cv::Point2i> points;
    for (int j = 0; j < 4; j++) {
        points.push_back(Point(static_cast<int>(vertex[j].x), static_cast<int>(vertex[j].y)));
    }
    polylines(img, points, true, color, 2, 8, 0);
}

void ArmorDescriptor::show_distance(cv::Mat& img) {
    std::stringstream ss;
    ss << "Distance: " << distance;
    putText(img, ss.str(), vertex[0], FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
}

