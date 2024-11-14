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
#include "usb_can.hpp"
#include "MvCameraControl.h"
#include "ArmorDetector.hpp"

using namespace std;
using namespace cv;

float compensation[5] = {0, 0.01, 0.02, 0.03, 0.04};

unsigned char frame_buffer[5000000] = {};
// cameara set
Mat cameraMatrix = (Mat_<double>(3,3) <<
                    2413.60174323956, 0, 719.915121318174,
		            0, 2412.32933298837, 532.984303033096,
		            0, 0, 1);

Mat distCoeffs = (Mat_<double>(1,5) << -0.0331357167629687, 0.149106285066012, 0, 0, 0);

Mat camera_gun_matrix = (Mat_<double>(4,4) <<
                    0, 0, 1, 140,
		            -1, 0, 0, 0,
		            0, -1, 0, 40,
                    0, 0, 0, 1);



template<typename T>
float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2) 
{
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2)); 
}

int main() 
{
    VideoCapture cap(0);
    usb_can can("/dev/ttyS1", 2000000);
    can.usb_open();
    if (!cap.isOpened()) {
        cout << "Error in opening camera" << endl;
        return -1;
    }

    int nRet = -1;
    void* m_handle = NULL;
    unsigned int nTLayerType = MV_USB_DEVICE;
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
    if (MV_OK != nRet) {
        printf("error: EnumDevices fail [%x]\n", nRet);
        return -1;
    }

    if (m_stDevList.nDeviceNum == 0) {
        printf("no camera found!\n");
        return -1;
    }

    int nDeviceIndex = 0;
    MV_CC_DEVICE_INFO m_stDevInfo = {0};
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
    nRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);
    if (MV_OK != nRet) {
        printf("error: CreateHandle fail [%x]\n", nRet);
        return -1;
    }

    unsigned int nAccessMode = MV_ACCESS_Exclusive;
    unsigned short nSwitchoverKey = 0;
    nRet = MV_CC_OpenDevice(m_handle, nAccessMode, nSwitchoverKey);
    if (MV_OK != nRet) {
        printf("error: OpenDevice fail [%x]\n", nRet);
        return -1;
    }

    nRet = MV_CC_SetEnumValue(m_handle, "PixelFormat", 0x02180014);
    if (MV_OK != nRet) {
        printf("error: Setting PixelFormat for camera[0x%x]\n", nRet);
        return -1;
    }

    nRet = MV_CC_StartGrabbing(m_handle);
    if (MV_OK != nRet) {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }

    ArmorDetector detector;
    detector.init(RED);

    double fps = cap.get(CAP_PROP_FPS);
    cout << "Frames fps:" << fps << endl;

    while (true) {
        Mat frame, adjustImg;
        cap >> frame;
        if (frame.empty()) break;

        imshow("capture", frame);

        frame.convertTo(adjustImg, -1, 1, -45); 
        detector.loadImg(adjustImg);
        detector.detect();

        ArmorDescriptor armor_target;
        if (detector.getArmors().size() > 0) {
            armor_target = detector.armor_select(detector.getArmors(), 0);

            cv::Mat coordinate = armor_target.measure(cameraMatrix, distCoeffs);
            Mat aim_target;
            double yaw_angle = 0;
            double pitch_angle = 0;
            aim_target = camera_gun_matrix * armor_target.coordinate;
            double x = aim_target.at<double>(0, 0);
            double y = aim_target.at<double>(1, 0);
            double z = aim_target.at<double>(2, 0);
            double distance = armor_target.distance;
            int index = std::min(static_cast<int>(distance / 1000), 4);

            if (distance >= 0) {
                yaw_angle = asin(y / distance);
                pitch_angle = asin(z / distance) + compensation[index];
            }

            controller_input(can, yaw_angle, pitch_angle);
            cout << "yaw angle:" << yaw_angle << endl;
            cout << "pitch angle:" << pitch_angle << endl;
        } else {
            controller_input(can, 0, 0);
        }

        // FPS display
        double current_fps = cap.get(CAP_PROP_FPS);
        string FPS = "fps:" + to_string(current_fps);
        putText(detector.getDebugImg(), FPS, Point(3, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 255, 0), 1);
        imshow("armors", detector.getDebugImg());

        int key = waitKey(1);
        if (key == 27 || key == 'q') break; 
    }

    can.usb_close();
    cap.release();
    destroyAllWindows();
    return 0;
}
