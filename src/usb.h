#ifndef __LIDAR_USB_H__
#define __LIDAR_USB_H__

void usb_init();

void usb_handle_frame(LiDARFrameTypeDef *frame);

#endif /* __LIDAR_USB_H__ */
