#ifndef _USB32_H
#define _USB32_H

#include <ros/ros.h>

typedef struct
{
    bool SetPosition;
    float Altitude;
}USB32_Structure;

void usb32_init(void);
void usb32_thread(void);
USB32_Structure get_USB32_Status(void);
void usb32_send_frame(int16_t x, int16_t y, int16_t z);





#endif
