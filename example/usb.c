#include <string.h>

#include "pico/unique_id.h"
#include "pico/util/queue.h"

#include "tusb.h"
#include "device/usbd_pvt.h"

#include "lidar.h"
#include "usb_descriptors.h"

#ifdef DEBUG
#include <stdio.h>
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) { }
#endif

enum usb_ctx_state {
	CTX_STATE_CLOSED,
	CTX_STATE_OPENED,
};

struct usb_ctx {
	enum usb_ctx_state state;
	uint8_t rhport;
	uint8_t ep_in;
	queue_t tx_queue;
	bool overflowed;
};

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

struct usb_ctx ctx;

static void lidar_usb_driver_init(void)
{
	DBG_PRINTF("%s\n", __func__);

	// At worst, we should only need to buffer 2 frames
	// for the interrupt endpoint
	queue_init(&ctx.tx_queue, sizeof(LiDARFrameTypeDef), 2);

	ctx.state = CTX_STATE_CLOSED;
}

static void lidar_usb_driver_reset(uint8_t rhport)
{
	DBG_PRINTF("%s\n", __func__);

	ctx.state = CTX_STATE_CLOSED;
	ctx.overflowed = false;

	LiDARFrameTypeDef frame;

	// Clear the queue
	while (queue_try_remove(&ctx.tx_queue, &frame));
}

static uint16_t lidar_usb_driver_open(uint8_t rhport, tusb_desc_interface_t const *desc_intf, uint16_t max_len)
{
	DBG_PRINTF("%s bInterfaceNumber: %d\n", __func__, desc_intf->bInterfaceNumber);

	if ((desc_intf->bInterfaceClass != 0xff) ||
	    (desc_intf->bInterfaceSubClass != 0xff) ||
	    (desc_intf->bInterfaceProtocol != 0xff)) {
		// Not our interface
		return 0;
	}

	tusb_desc_endpoint_t *ep_desc = (tusb_desc_endpoint_t *)tu_desc_next(desc_intf);

	usbd_edpt_open(rhport, ep_desc);

	ctx.state = CTX_STATE_OPENED;
	ctx.rhport = rhport;
	ctx.overflowed = false;
	ctx.ep_in = ep_desc->bEndpointAddress;

	return sizeof(tusb_desc_interface_t) + sizeof(tusb_desc_endpoint_t);
}

static bool lidar_usb_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
	DBG_PRINTF("%s\n", __func__);
	return true;
}

void __write_string(char *str, int len)
{
	while (len) {
		int avail = (int) tud_cdc_write_available();

		int to_write = len;
		if (to_write > avail) {
			to_write = avail;
		}

		if (to_write) {
			tud_cdc_write(str, to_write);
		} else {
			tud_cdc_write_flush();
			tud_task();
			continue;
		}

		len -= to_write;
		str += to_write;
	}
}

void __write_frame_cdc(LiDARFrameTypeDef *frame)
{
	char buf[32];
	int start_angle = frame->start_angle;
	int end_angle = frame->end_angle;
	if (end_angle < start_angle) {
		end_angle += 36000;
	}

	float angle_per_sample = ((end_angle - start_angle) / POINT_PER_PACK) * 0.01;
	float angle = start_angle * 0.01;
	for (int i = 0; i < POINT_PER_PACK; i++) {
		int len = snprintf(buf, 32, "%3.2f, %d\r\n", angle, frame->point[i].distance);
		if (len >= sizeof(buf)) {
			len = sizeof(buf);
		}

		__write_string(buf, len);

		angle += angle_per_sample;
		if (angle > 360.0) {
			angle -= 360.0;
		}
	}
	tud_cdc_write_flush();
}

void usb_handle_frame(LiDARFrameTypeDef *frame)
{
	if (tud_cdc_connected()) {
		__write_frame_cdc(frame);
	}

	if (ctx.state != CTX_STATE_OPENED) {
		return;
	}

	if (!tud_mounted()) {
		lidar_usb_driver_reset(ctx.rhport);
	}

	if (usbd_edpt_busy(ctx.rhport, ctx.ep_in)) {
		if (!queue_try_add(&ctx.tx_queue, frame) && !ctx.overflowed) {
			DBG_PRINTF("USB queue full!\n");
			ctx.overflowed = true;
		}
	} else {
		usbd_edpt_claim(ctx.rhport, ctx.ep_in);
		usbd_edpt_xfer(ctx.rhport, ctx.ep_in, (uint8_t *)frame, sizeof(*frame));
	}
}

static bool lidar_usb_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	DBG_PRINTF("%s %d\n", __func__, xferred_bytes);

	if (ep_addr == ctx.ep_in) {
		LiDARFrameTypeDef frame;
		if (queue_try_remove(&ctx.tx_queue, &frame)) {
			usbd_edpt_xfer(ctx.rhport, ctx.ep_in, (uint8_t *)&frame, sizeof(frame));
		} else {
			usbd_edpt_release(ctx.rhport, ctx.ep_in);
		}
	}

	return true;
}

void usb_init()
{
	tusb_init();
}
