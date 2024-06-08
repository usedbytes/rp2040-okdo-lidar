#include <string.h>

#include "pico/unique_id.h"

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
#define STRING_HEADER(_str_len) ((TUSB_DESC_STRING << 8) | ((_str_len * 2) + 2))
#define STRING_MAX_LEN 16

	(void) langid;

	// tusb_desc_string_t is packed, so we can't return it as uint16_t *
	// without a warning. So, build it manually.

	static uint16_t buf[STRING_MAX_LEN + 1] = { 0 };

	buf[0] = STRING_HEADER(0);

	if (index == 0) {
		buf[0] = STRING_HEADER(2);
		buf[1] = 0x409;
	} else if (index < (sizeof(lidar_strings) / sizeof(lidar_strings[0]))) {
		const char *str = lidar_strings[index - 1];

		if (!strcmp(str, "SERIALNO")) {
			char id_str[(PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2) + 1];
			pico_get_unique_board_id_string(id_str, sizeof(id_str));

			int len = PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2;
			if (len > STRING_MAX_LEN) {
				len = STRING_MAX_LEN;
			}

			buf[0] = STRING_HEADER(len);
			for (int i = 0; i < len; i++) {
				buf[i + 1] = id_str[i];
			}
		} else {
			int len = strlen(str);

			if (len > STRING_MAX_LEN) {
				len = STRING_MAX_LEN;
			}

			buf[0] = STRING_HEADER(len);
			for (int i = 0; i < len; i++) {
				buf[i + 1] = str[i];
			}
		}
	}

	return buf;
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
