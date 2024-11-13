#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <armorParam.h>
#include "LightDescriptor.h"
#include "armorDescriptor.h"
#include "controller.hpp"

class ArmorDetector {
public:
    float ArmorDetector::distance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    return cv::norm(pt1 - pt2);
}

    void init(int selfColor) {
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

    void loadImg(Mat& img) {
        _srcImg = img;
        Rect imgBound = Rect(cv::Point(0,0), Point(_srcImg.cols, _srcImg.rows));
        _roi = imgBound;
        _roiImg = _srcImg(_roi).clone();
    }

    Mat CaptureVideo(VideoCapture& video) {
        cv::Mat src_img, resized_img, _grayimg, hsv_img, gray_img, debug_img;
        vector<Mat> channels;
        while (video.isOpened()) {
            Mat frame;
            video >> frame;
            if (frame.empty()) {
                cout << "something is wrong." << endl;
                break;
            }
            cvtColor(frame, hsv_img, COLOR_BGR2HSV);
            cvtColor(frame, gray_img, COLOR_BGR2GRAY, 1);
            split(frame, channels);
            _grayimg=channels.at(0)-channels.at(2);
            return _grayimg;
        }
    }

    int detect() {
        _grayImg = separateColors_2();
        imshow("grayImg", _grayImg);

        
        int brightness_threshold = 205;
        Mat binBrightImg;
        threshold(_grayImg, binBrightImg, brightness_threshold, 255, cv::THRESH_BINARY);
        // imshow("thresh", binBrightImg);

        Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        dilate(binBrightImg, binBrightImg, element);
        imshow("dilate", binBrightImg);

        vector<vector<Point>> lightContours; 
        findContours(binBrightImg.clone(), lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // debug
        _debugImg = _roiImg.clone();
        for(size_t i = 0; i < lightContours.size(); i++) {
        }
        // imshow("contours", _debugImg);

        
        vector<LightDescriptor> lightInfos;
        filterContours(lightContours, lightInfos);
        if(lightInfos.empty()) {
            cout << "No lights have been detected" << endl;
            return -1;
        }

        drawLightInfo(lightInfos);

        _armors = matchArmor(lightInfos);
        if(_armors.empty()) {
            cout << "armor empty" << endl;
            // return -1;
        }

        for(size_t i = 0; i < _armors.size(); i++) {
            vector<Point2i> points;
            for (int j = 0; j < 4; j++) {
                points.push_back(Point(static_cast<int>(_armors[i].vertex[j].x), 
                    static_cast<int>(_armors[i].vertex[j].y)));
            }
            polylines(_debugImg, points, true, Scalar(0,255,0),
                    1, 8, 0);
        // imshow("armors", _debugImg);
    }

    Mat Solve_PnP(Mat &cameraMatrix, Mat &distCoeffs, vector<Point3f> &objectPoints, vector<Point2f> &ImagePoints) {
        Mat rvec, tvec; 
        bool success = solvePnP(objectPoints, ImagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        if (success) {
            Mat rotationMatrix;
            Rodrigues(rvec, rotationMatrix); 
            return rotationMatrix;
        }
    };

    static bool armor_distance_sort_compare(ArmorDescriptor armor_1, ArmorDescriptor armor_2)
    {
    return armor_1.distance < armor_2.distance;
    }

    ArmorDescriptor armor_select(vector<ArmorDescriptor> &armor_list, int method)
    {
        vector<ArmorDescriptor> armor_list_clone = armor_list;
        ArmorDescriptor armor_selected;
        if (method == 0)
        {
            sort(armor_list_clone.begin(), armor_list_clone.end(), armor_distance_sort_compare);
            armor_selected = armor_list_clone[0];
        }
        return armor_selected;
    }


    Mat separateColors() {
        vector<Mat> channels;
        split(_roiImg, channels);
        Mat grayImg;
        if (_enemy_color==RED) {
            grayImg = channels.at(2)-channels.at(0);//R-B
        }
        else {
            grayImg = channels.at(0)-channels.at(2);//B-R
        }
        return grayImg;
    }

    Mat separateColors_2() {
        vector<Mat> channels;
        split(_roiImg, channels);
        Mat blueImg = channels[0];
        Mat grayImg;
        // GaussianBlur(blueImg, grayImg, Size(5,5), 0);
        return blueImg;
    }

    void filterContours(vector<vector<Point>>& lightContours, vector<LightDescriptor>& lightInfos) {
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

    void drawLightInfo(vector<LightDescriptor>& LD) {
        _debugImg = _roiImg.clone();
        vector<vector<Point>> cons;
        int i = 0;
        for (auto &lightinfo: LD) {
            RotatedRect rotate = lightinfo.rec();
            auto vertices = new Point2f[4];
            rotate.points(vertices);
            vector<Point> con;
            for (int i = 0; i < 4; i++) {
                con.push_back((vertices[i]));
            }
            cons.push_back(con);
            drawContours(_debugImg, cons, i, Scalar(0, 255, 255), 1, 8);
            // imshow("rotateRec", _debugImg);
            i++;
            delete vertices;
        }
    }

    vector<ArmorDescriptor> matchArmor(vector<LightDescriptor>& lightInfos) {
        vector<ArmorDescriptor> armors;
        
        sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2){
            return ld1.center.x < ld2.center.x;
        });
        
        for (size_t i = 0; i < lightInfos.size(); i++) {
            
            for (size_t j = i + 1; (j < lightInfos.size()); j++) {
                const LightDescriptor& leftLight = lightInfos[i];
                const LightDescriptor& rightLight = lightInfos[j];

                float angleDiff = abs(leftLight.angle - rightLight.angle);
                
                float lenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                
                if (angleDiff > _param.light_max_angle_diff_ || lenDiff_ratio > _param.light_max_height_diff_ratio_) {
                    continue;
                }

                float dis = distance(leftLight.center, rightLight.center);
                int delta_x = leftLight.center.x - rightLight.center.x;
                int delta_y = leftLight.center.y - rightLight.center.y;
                float distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
               
                float meanLen = (leftLight.length + rightLight.length) / 2;
               
                float yDiff = abs(leftLight.center.y - rightLight.center.y);
                
                float yDiff_ratio = yDiff / meanLen;
                
                float xDiff = abs(leftLight.center.x - rightLight.center.x);
                
                float xDiff_ratio = xDiff / meanLen;
                
                float ratio = dis / meanLen;
               
                if (yDiff_ratio > _param.light_max_y_diff_ratio_ || 
                    xDiff_ratio < _param.light_min_x_diff_ratio_ ||
                    ratio > _param.armor_max_aspect_ratio_ || 
                    ratio < _param.armor_min_aspect_ratio_) {
                    continue;
                }

                int armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;
                
                float ratiOff = (armorType == BIG_ARMOR) ? max(_param.armor_big_armor_ratio-ratio, float(0)) :
                max(_param.armor_small_armor_ratio-ratio, float(0));
                float yOff = yDiff / meanLen;
                float rotationScore = -(ratiOff * ratiOff + yOff * yOff);
                
                ArmorDescriptor armor(leftLight, rightLight, armorType,
                    _grayImg, rotationScore, _param);
                armors.emplace_back(armor);
                break;
            }
        }
        return armors;
    }

    void distance() {

    }

    void adjustRec(cv::RotatedRect& rec)
    {
        using std::swap; 
        float& width = rec.size.width;
        float& height = rec.size.height;
        float& angle = rec.angle;
        
        while(angle >= 90.0) angle -= 180.0;
        while(angle < -90.0) angle += 180.0;

        if(angle >= 45.0)
        {
            swap(width, height);
            angle -= 90.0;
        }
        else if(angle < -45.0)
        {
            swap(width, height);
            angle += 90.0;
        }
    }
    cv::Mat _debugImg;
private:
    int _enemy_color;
    int _self_color;
    cv::Rect _roi;
    cv::Mat _srcImg;
    Mat _grayImg;

    Mat _roiImg;
    ArmorParam _param;
    vector<ArmorDescriptor> _armors;
};
}