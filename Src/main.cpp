#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <armorParam.h>
#include <Inc/LightDescriptor.h>
#include "Inc/armorDescriptor.h"

using namespace std;
using namespace cv;

// cameara set
Mat cameraMatrix = (Mat_<double>(3,3) <<
                    2413.60174323956, 0, 719.915121318174,
		            0, 2412.32933298837, 532.984303033096,
		            0, 0, 1);

Mat distCoeffs = (Mat_<double>(1,5) << -0.0331357167629687, 0.149106285066012, 0, 0, 0);

// 鏋涓庣浉鏈虹殑鐩稿浣嶅Э
Mat camera_gun_matrix = (Mat_<double>(4,4) <<
                    0, 0, 1, 140,
		            -1, 0, 0, 0,
		            0, -1, 0, 40,
                    0, 0, 0, 1);



template<typename T>
float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2) // &锟斤拷示直锟接对碉拷锟斤拷锟斤拷枚锟斤拷锟斤拷歉锟斤拷锟�
{
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2)); // 锟斤拷锟缴讹拷锟斤拷
}

// 锟斤拷锟斤拷ArmorDetector锟斤拷
class ArmorDetector {
public:
    // 锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷色
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
        Rect imgBound = Rect(cv::Point(0,0), Point(_srcImg.cols, _srcImg.rows));// rect锟斤拷为cv锟叫憋拷示锟斤拷锟轿碉拷锟斤拷锟斤拷
        _roi = imgBound;
        _roiImg = _srcImg(_roi).clone();
    }

    // 锟剿猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
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

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟�
    int detect() {
        _grayImg = separateColors_2();
        imshow("grayImg", _grayImg);

        // 锟斤拷值锟斤拷图锟今，憋拷锟节诧拷锟斤拷锟斤拷锟斤拷
        int brightness_threshold = 205;// 锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷,锟斤拷锟斤拷实锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟饺ｏ拷锟斤拷锟斤拷
        Mat binBrightImg;
        threshold(_grayImg, binBrightImg, brightness_threshold, 255, cv::THRESH_BINARY);
        // imshow("thresh", binBrightImg);

        // 锟斤拷锟酵达拷锟斤拷锟斤拷锟斤拷止锟斤拷锟街碉拷锟斤拷锟叫讹拷锟斤拷锟斤拷锟斤拷傻亩锟斤拷锟斤拷锟�
        Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        dilate(binBrightImg, binBrightImg, element);
        imshow("dilate", binBrightImg);

        // 寻锟揭撅拷锟斤拷锟斤拷锟酵达拷锟斤拷锟侥讹拷值锟斤拷图锟斤拷锟斤拷锟斤拷锟�
        vector<vector<Point>> lightContours; // 锟斤拷维锟斤拷锟介，每锟叫达拷锟斤拷一锟斤拷锟斤拷锟斤拷
        findContours(binBrightImg.clone(), lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // debug
        _debugImg = _roiImg.clone();
        for(size_t i = 0; i < lightContours.size(); i++) {
            // drawContours(_debugImg, lightContours, i, Scalar(0,0,255), 1, 8);
        }
        // imshow("contours", _debugImg);

        // 锟斤拷锟斤拷锟斤拷锟斤拷息锟斤拷锟接碉拷LightDescriptor锟斤拷锟絣ightInfos锟斤拷锟斤拷锟斤拷
        vector<LightDescriptor> lightInfos;
        filterContours(lightContours, lightInfos);
        if(lightInfos.empty()) {
            cout << "No lights have been detected" << endl;
            return -1;
        }

        // 锟斤拷锟狡碉拷锟斤拷锟斤拷锟斤拷
        drawLightInfo(lightInfos);

        _armors = matchArmor(lightInfos);
        if(_armors.empty()) {
            cout << "armor empty" << endl;
            // return -1;
        }

        // 锟斤拷锟斤拷每锟斤拷装锟阶板，锟斤拷锟斤拷装锟阶帮拷锟斤拷锟斤拷
        for(size_t i = 0; i < _armors.size(); i++) {
            vector<Point2i> points;
            for (int j = 0; j < 4; j++) {
                points.push_back(Point(static_cast<int>(_armors[i].vertex[j].x), // 锟斤拷一锟斤拷装锟阶帮拷亩锟斤拷锟斤拷锟斤拷锟�
                    static_cast<int>(_armors[i].vertex[j].y)));
            }
            polylines(_debugImg, points, true, Scalar(0,255,0),
                    1, 8, 0);//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥讹拷锟斤拷锟�

        }
        // imshow("armors", _debugImg);
    }

    Mat Solve_PnP(Mat &cameraMatrix, Mat &distCoeffs, vector<Point3f> &objectPoints, vector<Point2f> &ImagePoints) {
        Mat rvec, tvec; // 锟斤拷锟斤拷锟斤拷锟阶拷锟斤拷锟斤拷锟狡斤拷锟斤拷锟斤拷锟�
        bool success = solvePnP(objectPoints, ImagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        if (success) {
            Mat rotationMatrix;
            Rodrigues(rvec, rotationMatrix); // 锟斤拷锟斤拷转锟斤拷锟斤拷转锟斤拷为锟斤拷转锟斤拷锟斤拷
            return rotationMatrix;
        }
    };

    bool armor_distance_sort_compare(armor armor_1, armor armor_2)
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


    Mat ArmorDescriptor::measure(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
    {
    vector<Point3d> objectPoints;
	objectPoints.push_back(Point3d(-65, 28, 0));
	objectPoints.push_back(Point3d(-65, -28, 0));
	objectPoints.push_back(Point3d(65, 28, 0));
	objectPoints.push_back(Point3d(65, -28, 0));
    Mat rvec;
    Mat tvec;
    Mat rmat;
    solvePnP(objectPoints, this->points, cameraMatrix, distCoeffs, rvec, tvec);
    Rodrigues(rvec, rmat);
    Mat armor_cam_matrix;
    Mat supplement = (Mat_<double>(1, 4) << 0, 0, 0, 1);
    Mat zero_coordinate = (Mat_<double>(4, 1) << 0, 0, 0, 1);
    hconcat(rmat, tvec, armor_cam_matrix);
    vconcat(armor_cam_matrix, supplement, armor_cam_matrix);
    this -> coordinate = armor_cam_matrix * zero_coordinate;
    Mat distance_square = coordinate.t() * coordinate;
    this -> distance = sqrt(distance_square.at<double>(0, 0));
    return this -> coordinate;
    }


    //锟斤拷锟斤拷锟斤拷色锟斤拷锟斤拷取锟斤拷锟斤拷锟斤拷色锟斤拷锟斤拷锟截灰讹拷图
    Mat separateColors() {
        vector<Mat> channels;
        split(_roiImg, channels);
        Mat grayImg;
        // 锟睫筹拷锟斤拷锟斤拷要锟斤拷锟斤拷色
        // 锟斤拷锟斤拷锟斤拷要锟斤拷锟斤拷色锟斤拷去锟斤拷锟揭凤拷锟斤拷色锟斤拷
        if (_enemy_color==RED) {
            grayImg = channels.at(2)-channels.at(0);//R-B
        }
        else {
            grayImg = channels.at(0)-channels.at(2);//B-R
        }
        return grayImg;
    }

    // 锟节讹拷锟街达拷锟斤拷锟斤拷式锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷头锟斤拷锟姐泛色锟斤拷锟�
    Mat separateColors_2() {
        vector<Mat> channels;
        split(_roiImg, channels);
        Mat blueImg = channels[0];
        Mat grayImg;
        // GaussianBlur(blueImg, grayImg, Size(5,5), 0);
        return blueImg;
    }


    // 筛选锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    // 锟斤拷锟斤拷娲拷锟斤拷锟斤拷木锟斤拷螅锟斤拷卮娲拷锟斤拷锟斤拷锟较拷锟斤拷锟接撅拷锟斤拷
    void filterContours(vector<vector<Point>>& lightContours, vector<LightDescriptor>& lightInfos) {
        for (const auto& contour : lightContours) { // 锟斤拷auto锟皆讹拷锟狡碉拷lightcontour锟斤拷锟酵ｏ拷每锟斤拷循锟斤拷锟斤拷contour锟斤拷锟叫ｏ拷冒锟脚讹拷锟斤拷循锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
            // 锟矫碉拷锟斤拷锟�
            float lightContourArea = contourArea(contour);
            // 锟睫筹拷锟斤拷锟叫★拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷小锟侥碉拷锟斤拷锟杰癸拷锟绞碉拷锟斤拷小锟斤拷锟斤拷锟斤拷锟�
            if (lightContourArea < _param.light_min_area) continue;
            // 锟斤拷圆锟斤拷锟斤拷锟斤拷锟矫碉拷锟斤拷泳锟斤拷锟�
            RotatedRect lightRec = fitEllipse(contour);
            // 锟斤拷锟斤拷锟斤拷锟斤拷锟角度ｏ拷约锟斤拷锟斤拷锟斤拷45锟斤拷45锟斤拷使锟矫筹拷锟斤拷投锟斤拷锟绞硷拷锟轿拷锟饺凤拷锟揭伙拷锟�
            adjustRec(lightRec);
            // 锟斤拷锟竭比★拷凸锟斤拷锟斤拷筛选锟斤拷锟斤拷 凸锟斤拷=锟斤拷锟斤拷锟斤拷锟�/锟斤拷泳锟斤拷锟斤拷锟斤拷
            // 锟斤拷锟斤拷锟侥匡拷锟竭比诧拷锟斤拷锟斤拷0.4 凸锟饺诧拷锟斤拷锟斤拷0.5 太锟酵碉拷凸锟饺达拷锟斤拷锟斤拷一锟斤拷锟接斤拷直锟斤拷没锟叫匡拷锟饺的碉拷锟斤拷
            // 锟绞碉拷锟斤拷锟接匡拷锟竭憋拷锟斤拷锟斤拷锟接对诧拷影锟斤拷锟斤拷锟斤拷锟轿碉拷锟斤拷锟斤拷锟斤拷识锟斤拷锟斤拷
            if (lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
                lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity) continue;
            // 锟皆碉拷锟斤拷锟斤拷围锟绞碉拷锟斤拷锟斤拷
            lightRec.size.width *= _param.light_color_detect_extend_ratio;
            lightRec.size.height *= _param.light_color_detect_extend_ratio;
            // 锟斤拷锟斤拷锟斤拷锟斤拷锟窖撅拷锟斤拷separate锟斤拷锟斤拷锟斤拷直锟接憋拷锟斤拷锟斤拷锟�
            lightInfos.push_back(LightDescriptor(lightRec));
        }
    }

    // 锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷
    void drawLightInfo(vector<LightDescriptor>& LD) {
        _debugImg = _roiImg.clone();
        vector<vector<Point>> cons;
        int i = 0;
        for (auto &lightinfo: LD) {
            RotatedRect rotate = lightinfo.rec();
            auto vertices = new Point2f[4];// 锟斤拷锟斤拷一锟斤拷锟斤拷锟斤拷锟侥革拷point2f锟斤拷锟斤拷锟斤拷
            rotate.points(vertices);// 锟斤拷锟斤拷转锟斤拷锟轿碉拷锟侥革拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
            vector<Point> con;
            for (int i = 0; i < 4; i++) {
                con.push_back((vertices[i]));// 锟斤拷vertices锟叫碉拷锟侥革拷锟斤拷锟姐返锟截革拷con
            }
            cons.push_back(con);// 锟斤拷一锟脚憋拷锟斤拷锟�
            drawContours(_debugImg, cons, i, Scalar(0, 255, 255), 1, 8);
            // imshow("rotateRec", _debugImg);
            i++;
            delete vertices;
        }
    }

    vector<ArmorDescriptor> matchArmor(vector<LightDescriptor>& lightInfos) {
        vector<ArmorDescriptor> armors;
        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷x锟斤拷小锟斤拷锟斤拷锟斤拷锟斤拷
        sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2){
            return ld1.center.x < ld2.center.x;
        });
        // 锟斤拷锟斤拷锟斤拷锟叫碉拷锟斤拷锟斤拷锟斤拷匹锟斤拷
        for (size_t i = 0; i < lightInfos.size(); i++) {
            // 锟斤拷锟皆讹拷锟斤拷锟斤拷锟斤拷锟斤拷业锟斤拷锟斤拷锟斤拷冉锟�
            for (size_t j = i + 1; (j < lightInfos.size()); j++) {
                const LightDescriptor& leftLight = lightInfos[i];
                const LightDescriptor& rightLight = lightInfos[j];

                // 锟角差，装锟阶帮拷锟斤拷锟斤拷锟斤拷锟角诧拷锟斤拷锟�
                float angleDiff = abs(leftLight.angle - rightLight.angle);
                // 锟斤拷锟饺诧拷锟斤拷剩锟斤拷锟斤拷锟皆叫∷碉拷锟斤拷锟斤拷锟斤拷锟斤拷某锟斤拷锟皆斤拷锟斤拷锟斤拷锟斤拷锟狡讹拷越锟斤拷
                float lenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                // 筛选
                if (angleDiff > _param.light_max_angle_diff_ || lenDiff_ratio > _param.light_max_height_diff_ratio_) {
                    continue;
                }

                // 锟斤拷锟揭碉拷锟斤拷锟斤拷锟斤拷锟斤拷
                float dis = distance(leftLight.center, rightLight.center);
                // 锟斤拷锟揭碉拷锟斤拷锟斤拷锟饺碉拷平锟斤拷值
                float meanLen = (leftLight.length + rightLight.length) / 2;
                // 锟斤拷锟揭碉拷锟斤拷锟斤拷锟侥碉拷y锟侥诧拷值
                float yDiff = abs(leftLight.center.y - rightLight.center.y);
                // y锟斤拷谋锟斤拷锟�
                float yDiff_ratio = yDiff / meanLen;
                // 锟斤拷锟揭碉拷锟斤拷锟斤拷锟侥碉拷x锟侥诧拷值
                float xDiff = abs(leftLight.center.x - rightLight.center.x);
                // x锟斤拷谋锟斤拷锟�
                float xDiff_ratio = xDiff / meanLen;
                // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷缺锟街�
                float ratio = dis / meanLen;
                // 筛选
                if (yDiff_ratio > _param.light_max_y_diff_ratio_ || // 确锟斤拷锟斤拷锟揭碉拷锟斤拷锟节达拷直锟斤拷锟斤拷锟较碉拷位锟斤拷锟斤拷越咏锟斤拷锟斤拷锟街わ拷锟斤拷锟斤拷锟斤拷锟揭伙拷锟剿斤拷锟斤拷锟�
                    xDiff_ratio < _param.light_min_x_diff_ratio_ || // 确锟斤拷锟斤拷锟揭碉拷锟斤拷锟斤拷水平锟斤拷锟斤拷锟斤拷锟斤拷锟姐够锟侥撅拷锟诫，锟斤拷锟斤拷装锟阶帮拷锟斤拷锟斤拷幕锟斤拷锟斤拷锟斤拷锟�
                    ratio > _param.armor_max_aspect_ratio_ || // 确锟斤拷锟斤拷锟斤拷之锟斤拷木锟斤拷锟酵碉拷锟斤拷锟斤拷锟斤拷锟缴猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷确锟斤拷锟斤拷锟斤拷之锟斤拷锟斤拷锟斤拷锟斤拷
                    ratio < _param.armor_min_aspect_ratio_) {
                    continue;
                }

                // 锟斤拷锟秸憋拷值锟斤拷确锟斤拷锟斤拷小装锟斤拷
                int armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR; // 锟斤拷锟诫长锟斤拷为锟斤拷装锟阶板，锟斤拷之为小
                // 锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷
                float ratiOff = (armorType == BIG_ARMOR) ? max(_param.armor_big_armor_ratio-ratio, float(0)) :
                max(_param.armor_small_armor_ratio-ratio, float(0));
                float yOff = yDiff / meanLen;
                float rotationScore = -(ratiOff * ratiOff + yOff * yOff);
                // 锟矫碉拷匹锟斤拷锟阶帮拷装锟�
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
        using std::swap; // 锟斤拷准锟解交锟斤拷锟斤拷锟斤拷
        // 锟斤拷锟斤拷&使锟斤拷锟叫硷拷锟斤拷锟斤拷锟街憋拷锟斤拷薷锟斤拷锟阶拷锟斤拷蔚锟街�
        float& width = rec.size.width;
        float& height = rec.size.height;
        float& angle = rec.angle;
        // 锟角讹拷为0锟斤拷锟斤拷示锟斤拷锟竭猴拷水平锟斤拷平锟斤拷 锟斤拷锟角度ｏ拷锟斤拷示锟斤拷锟斤拷顺时锟斤拷锟斤拷转
        // 锟斤拷锟角讹拷锟斤拷锟斤拷锟斤拷 -90 锟斤拷 90 锟饺凤拷围锟节★拷锟斤拷锟斤拷锟斤拷为锟斤拷转锟斤拷锟轿的角讹拷通锟斤拷锟斤拷 -180 锟斤拷 180 锟斤拷之锟戒，锟芥范锟斤拷锟斤拷 -90 锟斤拷 90 锟饺革拷锟斤拷锟斤拷锟节达拷锟斤拷锟斤拷
        // 锟斤拷证锟斤拷锟斤拷锟斤拷锟揭伙拷锟斤拷锟�
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



int main() {
    VideoCapture cap(0);

    // 妫€鏌ユ槸鍚︽墦寮€鍓嶇疆鎽勫儚澶�
    if (!cap.isOpened()){
        cout << "Error in opening camera" << endl;
        return -1;
    }

    int nRet = -1;
    void*  m_handle = NULL;
    unsigned int nTLayerType = MV_USB_DEVICE;
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
    if (MV_OK != nRet)
    {
        printf("error: EnumDevices fail [%x]\n", nRet);
        return -1;
    }
    int i = 0;
    if (m_stDevList.nDeviceNum == 0)
    {
        printf("no camera found!\n");
        return -1;
    }

    int nDeviceIndex = 0;
    MV_CC_DEVICE_INFO m_stDevInfo = {0};
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
    nRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);
    if (MV_OK != nRet)
    {
        printf("error: CreateHandle fail [%x]\n", nRet);
        return -1;
    }

    unsigned int nAccessMode = MV_ACCESS_Exclusive;
    unsigned short nSwitchoverKey = 0;
    do
    {
        nRet = MV_CC_OpenDevice(m_handle, nAccessMode, nSwitchoverKey);
        if (MV_OK != nRet)
        {
            printf("error: OpenDevice fail [%x]\n", nRet);
            return -1;
        }
    } while (MV_OK != nRet);

    nRet = MV_CC_SetEnumValue(m_handle, "PixelFormat", 0x02180014);
    if (MV_OK != nRet)
    {
        printf("error: Setting PixelFormat for camera[0x%x]\n", nRet);
        return -1;
    }

    MV_FRAME_OUT stOutFrame = {0};
    MV_FRAME_OUT_INFO_EX stFrameInfo;
    memset(&stFrameInfo, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_StartGrabbing(m_handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }
    while(waitKey(1) != 'q')
    {
        do
        {
            nRet = MV_CC_GetOneFrameTimeout(m_handle, frame_buffer, sizeof(frame_buffer), &stFrameInfo, 100);

            if (MV_OK != nRet)
            {
                printf("error: GetFrame fail [%x]\n", nRet);
                controller_input(can, 0, 0);
                continue;
            }

        } while (MV_OK != nRet);

    ArmorDetector detector;
    detector.init(RED);

    // Mat img = imread("D:/OneDrive/锟斤拷锟斤拷/锟斤拷/vision_lib/test.jpeg");
    // VideoCapture capture("/mnt/hgfs/Ubuntu鏂囦欢/zhuangjiabanTEST.mp4");
    // if (!capture.isOpened()) {
        // cout << "Error epening video file" << endl;
        // return -1;
    // }

    double fps = cap.get(CAP_PROP_FPS);
    cout << "Frames fps:" << fps << endl;

    while (true) {
        Mat frame, adjustImg;
        cap >> frame;
        if (frame.empty()) break;
        imshow("capture", frame);

        frame.convertTo(adjustImg, -1, 1, -45); 
        detector.loadImg(adjustImg);
        // imshow("adjustImg", adjustImg);

        detector.detect();
        for (int id = 0; id < detector.matchArmor.size(); id++)
        {
            detector.matchArmor[id].measure(cameraMatrix, distCoeffs);
        }
        
        ArmorDescriptor armor_target;
        armor_target = armor_select(detector.matchArmor, 0);

        Mat aim_target;
	double yaw_angle = 0;
	double pitch_angle = 0;
        aim_target = camera_gun_matrix * armor_target.coordinate;
	double x, y, z;
	double distance = armor_target.distance;
	x = aim_target.at<double>(0, 0);
	y = aim_target.at<double>(1, 0);
	z = aim_target.at<double>(2, 0);
	int index = distance / 1000;
	if (index > 4)
	{
	    index = 4;
	}
	if (distance >= 0)
	{
	    yaw_angle = asin(y / distance);
	    pitch_angle = asin(z / distance) + compensation[index];
	}
	controller_input(can, yaw_angle, pitch_angle);
    cout << "yaw angle:" << yaw_angle << endl;
	cout << "pitch angle" << pitch_angle << endl;
        //imshow("img", frame_show);
        //imshow("v_and_h", frame_v_and_h);

        double current_fps = cap.get(CAP_PROP_FPS);
        string FPS = "fps:" + to_string(current_fps);

        
        putText(detector._debugImg, FPS, Point(3, 20), FONT_HERSHEY_TRIPLEX, 0.5,
            Scalar(255, 255, 255), 1);
        imshow("armors", detector._debugImg);

        int key = waitKey(1);
        if (key == 27 || key == 'q') break; 
    }

    cap.release();
    destroyAllWindows();
    return 0;
}