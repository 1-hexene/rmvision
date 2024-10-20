#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "usb_can.hpp"

void controller_input(usb_can &can, float angle_yaw, float angle_pitch);

#endif