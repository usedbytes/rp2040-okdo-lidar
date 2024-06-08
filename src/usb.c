#include <string.h>

#include "tusb.h"
#include "device/usbd_pvt.h"

#include "usb_descriptors.h"

static void lidar_usb_driver_init(void);
static void lidar_usb_driver_reset(uint8_t rhport);
static uint16_t lidar_usb_driver_open(uint8_t rhport, tusb_desc_interface_t const * desc_intf, uint16_t max_len);
static bool lidar_usb_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request);
static bool lidar_usb_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

static usbd_class_driver_t const lidar_usb_driver =
{
#if CFG_TUSB_DEBUG >= 2
    .name = "LIDAR",
#endif
    .init             = lidar_usb_driver_init,
    .reset            = lidar_usb_driver_reset,
    .open             = lidar_usb_driver_open,
    .control_xfer_cb  = lidar_usb_driver_control_xfer_cb,
    .xfer_cb          = lidar_usb_driver_xfer_cb,
    .sof              = NULL
};

// TinyUSB Device Callbacks

bool opened = false;

uint8_t const * tud_descriptor_device_cb(void)
{
	return (uint8_t *)&lidar_device_descriptor;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
	(void)index;

	return (uint8_t *)&lidar_config_descriptor;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void) langid;

	static uint8_t __attribute__((aligned(2)))buf[64] = { 0 };
	tusb_desc_string_t *desc = (tusb_desc_string_t *)&buf;

	desc->bDescriptorType = TUSB_DESC_STRING;
	desc->bLength = 0;

	if (index == 0) {
		desc->bLength = 4;
		desc->unicode_string[0] = 0x409;
	} else if (index < (sizeof(lidar_strings) / sizeof(lidar_strings[0]))) {
		char *str = lidar_strings[index - 1];
		int len = strlen(str);

		desc->bLength = 2 + len * 2;
		for (int i = 0; i < len; i++) {
			desc->unicode_string[i] = str[i];
		}
	}

	return (uint16_t *)desc;
}

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count)
{
	*driver_count = 1;

	return &lidar_usb_driver;
}

// End callbacks

static void lidar_usb_driver_init(void)
{
	printf("%s\n", __func__);
}

static void lidar_usb_driver_reset(uint8_t rhport)
{
	opened = false;
	printf("%s\n", __func__);
}

static uint16_t lidar_usb_driver_open(uint8_t rhport, tusb_desc_interface_t const *desc_intf, uint16_t max_len)
{
	printf("%s\n", __func__);

	if (opened) {
		return 0;
	}

	tusb_desc_endpoint_t *ep_desc = (tusb_desc_endpoint_t *)tu_desc_next(desc_intf);
	printf("ep_desc type: %d\n", ep_desc->bDescriptorType);

	int ret = usbd_edpt_open(rhport, ep_desc);
	printf("edpt_open: %d\n", ret);

	printf("max_size: %d, size: %d\n", max_len,
		sizeof(tusb_desc_interface_t) + sizeof(tusb_desc_endpoint_t));

	opened = true;

	return sizeof(tusb_desc_interface_t) + sizeof(tusb_desc_endpoint_t);
}

static bool lidar_usb_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
	printf("%s\n", __func__);
	return true;
}

void send_in()
{
	if (!opened) {
		return;
	}

	if (usbd_edpt_busy(0, 1 | TUSB_DIR_IN_MASK)) {
		return;
	}

	static char *buf = "Hello";
	usbd_edpt_xfer(0, 1 | TUSB_DIR_IN_MASK, buf, 6);
}

static bool lidar_usb_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	printf("%s %d\n", __func__, xferred_bytes);

	return true;
}

void usb_init()
{
	tusb_init();
}
