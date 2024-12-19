#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "armorDescriptor.h"  // Assuming this is another class you might have
#include "LightDescriptor.h"  // Assuming this is another class you might have

using namespace cv;
using namespace std;

class ArmorDetector {
public:
    ArmorDetector();
    void init(int selfColor);
    void loadImg(Mat& img);
    Mat CaptureVideo(VideoCapture& video);
    int detect();
    Mat Solve_PnP(Mat &cameraMatrix, Mat &distCoeffs, vector<Point3f> &objectPoints, vector<Point2f> &ImagePoints);
    ArmorDescriptor armor_select(const std::vector<ArmorDescriptor>& armor_list, int method);
    Mat separateColors();
    Mat separateColors_2();
    void filterContours(vector<vector<Point>>& lightContours, vector<LightDescriptor>& lightInfos);
    void drawLightInfo(vector<LightDescriptor>& LD);
    vector<ArmorDescriptor> matchArmor(vector<LightDescriptor>& lightInfos);
    static bool armor_distance_sort_compare(ArmorDescriptor armor_1, ArmorDescriptor armor_2);
    const std::vector<ArmorDescriptor>& getArmors() const { return _armors; }
    const cv::Mat& getDebugImg() const { return _debugImg; }
    
    void adjustRec(cv::RotatedRect& rec);
    template<typename T>
    float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2);

private:
    int _enemy_color;
    int _self_color;
    cv::Rect _roi;
    cv::Mat _srcImg;
    Mat _grayImg;
    Mat _roiImg;
    ArmorParam _param;
    vector<ArmorDescriptor> _armors;
    cv::Mat _debugImg;
};

#endif // ARMOR_DETECTOR_HPP
