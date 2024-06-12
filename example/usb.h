#ifndef __LIDAR_USB_H__
#define __LIDAR_USB_H__

#include "lidar.h"

void usb_init();

void usb_handle_frame(struct lidar_frame *frame);

#endif /* __LIDAR_USB_H__ */
