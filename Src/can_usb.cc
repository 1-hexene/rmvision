#include "usb_can.hpp"
#include <iostream>
#include <fcntl.h>
#include <cstring>
#include <sys/select.h>
#include <unistd.h>
#include <iostream>

using namespace std;

usb_can::usb_can()
{
    this -> fd = -1;
}

usb_can::usb_can(const char* path, int baud_rate)
{
    this -> fd = -1;
    this -> path = path;
    this -> baud_rate = baud_rate;
}

usb_can::~usb_can()
{
}

bool usb_can::usb_open()
{
    fd = open(this -> path.c_str(), O_RDWR);
    if(fd == -1)
    {
        std::cout << "open uart fail" << std::endl;
    }
    memset(&ios, 0, sizeof (ios));
    ios.c_cflag |= CLOCAL | CREAD;
    cfmakeraw(&ios);
    speed_t speed = baud_rate;
    cfsetispeed(&ios, speed);
    cfsetospeed(&ios, speed);
    ios.c_cflag &= ~CSIZE;
    ios.c_cflag |= CS8;
    ios.c_cflag &= ~PARENB;
    ios.c_cflag &= ~CSTOPB;
    ios.c_cc[VTIME] = 0;
    ios.c_cc[VMIN] = 1;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &ios);
    return USB_CAN_OK;
}

bool usb_can::usb_close()
{
    int ret;
 
    ret = close(fd);
    fd = -1;
 
    return USB_CAN_OK;
}

bool usb_can::data_write(unsigned char* data, int id, int length)
{
    int ret = -1;
    int value;
    unsigned char can_data[10];
    unsigned char buf[100];
    can_data[0] = id >> 8;
    can_data[1] = id;
    memcpy(can_data + 2, data, 8);
    fd_set write_set;
    struct timeval tv;
 
    if (fd == -1)
    {
        return -1;
    }
    FD_ZERO(&write_set);
    FD_SET(fd, &write_set);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    ret = select(fd + 1, 0, &write_set, 0, &tv);
    switch (ret)
    {
        case 0:
            break;
        case -1:
            break;
        default:
            if (FD_ISSET(fd, &write_set))
            {
                ret = write(fd, can_data, length + 2);
            }
            break;
    }
    return USB_CAN_OK;
}

int usb_can::data_read(unsigned char* data, int* id)
{
    return 0;
}
