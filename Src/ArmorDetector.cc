#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <armorParam.h>
#include "LightDescriptor.h"
#include "armorDescriptor.h"
#include "ArmorDetector.hpp"
#include <iostream>

using namespace cv;
using namespace std;

ArmorDetector::ArmorDetector() : _enemy_color(0), _self_color(0) {
    // 默认构造函数，可以初始化成员变量
}

void ArmorDetector::init(int selfColor) {
    if (selfColor == RED) { // RED = 0
        _enemy_color = BLUE; // BLUE = 1
        _self_color = RED;
    }
    else {
        _enemy_color = RED;
        _self_color = BLUE;
    }
    cout << "initialization complete." << endl;
}

void ArmorDetector::loadImg(Mat& img) {
    _srcImg = img;
    Rect imgBound = Rect(Point(0, 0), Point(_srcImg.cols, _srcImg.rows));
    _roi = imgBound;
    _roiImg = _srcImg(_roi).clone();
}

Mat ArmorDetector::CaptureVideo(VideoCapture& video) {
    cv::Mat frame;
    while (video.isOpened()) {
        video >> frame;
        if (frame.empty()) {
            cout << "something is wrong." << endl;
            break;
        }
        cv::Mat hsv_img, gray_img;
        vector<cv::Mat> channels;
        cvtColor(frame, hsv_img, COLOR_BGR2HSV);
        cvtColor(frame, gray_img, COLOR_BGR2GRAY, 1);
        split(frame, channels);
        return channels.at(0) - channels.at(2); // Simplified grayimg calculation
    }
    return Mat(); // No valid frame found, return empty Mat
}

int ArmorDetector::detect() {
    _grayImg = separateColors_2();
    imshow("grayImg", _grayImg);

    int brightness_threshold = 205;
    Mat binBrightImg;
    threshold(_grayImg, binBrightImg, brightness_threshold, 255, THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    dilate(binBrightImg, binBrightImg, element);
    imshow("dilate", binBrightImg);

    vector<vector<Point>> lightContours;
    findContours(binBrightImg.clone(), lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    _debugImg = _roiImg.clone();

    vector<LightDescriptor> lightInfos;
    filterContours(lightContours, lightInfos);
    if (lightInfos.empty()) {
        cout << "No lights have been detected" << endl;
        return -1;
    }

    drawLightInfo(lightInfos);
    _armors = matchArmor(lightInfos);
    if (_armors.empty()) {
        cout << "armor empty" << endl;
        // return -1; // Optional, depending on the requirements
    }

    for (size_t i = 0; i < _armors.size(); i++) {
        vector<Point2i> points;
        for (int j = 0; j < 4; j++) {
            points.push_back(Point(static_cast<int>(_armors[i].vertex[j].x),
                                   static_cast<int>(_armors[i].vertex[j].y)));
        }
        polylines(_debugImg, points, true, Scalar(0, 255, 0), 1, 8, 0);
    }

    return 0; // Added return value to indicate success
}

Mat ArmorDetector::Solve_PnP(Mat &cameraMatrix, Mat &distCoeffs, vector<Point3f> &objectPoints, vector<Point2f> &ImagePoints) {
    Mat rvec, tvec;
    bool success = solvePnP(objectPoints, ImagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    if (success) {
        Mat rotationMatrix;
        Rodrigues(rvec, rotationMatrix);
        return rotationMatrix;
    }
    return Mat(); // Failed, return empty matrix
}

bool ArmorDetector::armor_distance_sort_compare(ArmorDescriptor armor_1, ArmorDescriptor armor_2) {
    return armor_1.distance < armor_2.distance;
}

ArmorDescriptor ArmorDetector::armor_select(const std::vector<ArmorDescriptor>& armor_list, int method) {
    std::vector<ArmorDescriptor> armor_list_clone = armor_list;
    ArmorDescriptor armor_selected;
    if (method == 0) {
        std::sort(armor_list_clone.begin(), armor_list_clone.end(), armor_distance_sort_compare);
        armor_selected = armor_list_clone[0];
    }
    return armor_selected;
}

Mat ArmorDetector::separateColors() {
    vector<Mat> channels;
    split(_roiImg, channels);
    Mat grayImg;
    if (_enemy_color == RED) {
        grayImg = channels.at(2) - channels.at(0); // R - B
    }
    else {
        grayImg = channels.at(0) - channels.at(2); // B - R
    }
    return grayImg;
}

Mat ArmorDetector::separateColors_2() {
    vector<Mat> channels;
    split(_roiImg, channels);
    return channels[0]; // Simplified to only return the blue channel
}

void ArmorDetector::filterContours(vector<vector<Point>>& lightContours, vector<LightDescriptor>& lightInfos) {
    for (const auto& contour : lightContours) {
        float lightContourArea = contourArea(contour);
        if (lightContourArea < _param.light_min_area) continue;

        RotatedRect lightRec = fitEllipse(contour);
        adjustRec(lightRec);

        if (lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
            lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity) continue;

        lightRec.size.width *= _param.light_color_detect_extend_ratio;
        lightRec.size.height *= _param.light_color_detect_extend_ratio;

        lightInfos.push_back(LightDescriptor(lightRec));
    }
}

void ArmorDetector::drawLightInfo(vector<LightDescriptor>& LD) {
    _debugImg = _roiImg.clone();
    vector<vector<Point>> cons;
    int i = 0;
    for (auto &lightinfo : LD) {
        RotatedRect rotate = lightinfo.rec();
        auto vertices = new Point2f[4];
        rotate.points(vertices);
        vector<Point> con;
        for (int i = 0; i < 4; i++) {
            con.push_back(vertices[i]);
        }
        cons.push_back(con);
        drawContours(_debugImg, cons, i, Scalar(0, 255, 255), 1, 8);
        i++;
        delete vertices;
    }
}

vector<ArmorDescriptor> ArmorDetector::matchArmor(vector<LightDescriptor>& lightInfos) {
    vector<ArmorDescriptor> armors;
    sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2){
        return ld1.center.x < ld2.center.x;
    });

    for (size_t i = 0; i < lightInfos.size(); i++) {
        for (size_t j = i + 1; j < lightInfos.size(); j++) {
            const LightDescriptor& leftLight = lightInfos[i];
            const LightDescriptor& rightLight = lightInfos[j];

            float angleDiff = abs(leftLight.angle - rightLight.angle);
            float lenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);

            if (angleDiff > _param.light_max_angle_diff_ || lenDiff_ratio > _param.light_max_height_diff_ratio_) {
                continue;
            }
            // Additional matching conditions and armor creation can be added here
        }
    }
    return armors;
}

void ArmorDetector::adjustRec(cv::RotatedRect& rec) {
    using std::swap;
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;

    while (angle >= 90.0) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;

    if (angle >= 45.0) {
        swap(width, height);
        angle -= 90.0;
    }
    else if (angle < -45.0) {
        swap(width, height);
        angle += 90.0;
    }
}

