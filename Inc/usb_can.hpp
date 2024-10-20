#ifndef __USB_CAN_HPP__
#define __USB_CAN_HPP__

#define USB_CAN_OK 0

#include <string>
#include <termios.h>

class usb_can
{
private:
    std::string path;
    int baud_rate;
    int fd;
    struct termios ios;
    
public:
    usb_can();
    usb_can(const char* path, int baud_rate);
    ~usb_can();
    bool usb_open();
    bool usb_close();
    bool data_write(unsigned char* data, int id, int length);
    int data_read(unsigned char* data, int *id);
};



#endif