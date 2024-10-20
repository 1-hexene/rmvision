#include "controller.hpp"
#include <cstring>

void controller_input(usb_can &can, float angle_yaw, float angle_pitch)
{
    unsigned char data[8] = {};
    float angle_rate_yaw = angle_yaw;
    float angle_rate_pitch = angle_pitch;
    memcpy(data, &angle_rate_yaw, 4);
    memcpy(data + 4, &angle_rate_pitch, 4);
    can.data_write(data, 0x121, 8);
}
