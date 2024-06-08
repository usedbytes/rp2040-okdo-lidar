
tusb_desc_device_t const lidar_device_descriptor = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x1010,

	.bDeviceClass       = TUSB_CLASS_VENDOR_SPECIFIC,
	.bDeviceSubClass    = TUSB_CLASS_VENDOR_SPECIFIC,
	.bDeviceProtocol    = TUSB_CLASS_VENDOR_SPECIFIC,

	.bMaxPacketSize0    = 64,

	.idVendor           = 0x1209,
	.idProduct          = 0x0001,
	.bcdDevice          = 0x0100,

	.iManufacturer      = 0x01,
	.iProduct           = 0x02,
	.iSerialNumber      = 0x03,

	.bNumConfigurations = 0x01
};

const char *lidar_strings[] = {
	"usedbytes",
	"lidar-usb",
	"SERIALNO",
	"lidar data",
	"reset",
};

struct __attribute__((packed)) lidar_config_descriptor {
	tusb_desc_configuration_t config;
	tusb_desc_interface_t lidar_interface;
	tusb_desc_endpoint_t lidar_ep;
	tusb_desc_interface_t reset_interface;
};

enum {
	ITF_NUM_LIDAR,
	//ITF_NUM_RESET,
	ITF_NUM_TOTAL,
};

#define NUM_ENDPOINTS 1

struct lidar_config_descriptor lidar_config_descriptor = {
	.config = {
		.bLength = sizeof(tusb_desc_configuration_t),
		.bDescriptorType = TUSB_DESC_CONFIGURATION,
		.wTotalLength = sizeof(tusb_desc_configuration_t) +
			ITF_NUM_TOTAL * sizeof(tusb_desc_interface_t) +
			NUM_ENDPOINTS * sizeof(tusb_desc_endpoint_t),
		.bNumInterfaces = ITF_NUM_TOTAL,
		.bConfigurationValue = 1,
		.iConfiguration = 0, // String index
		.bmAttributes = 0x80, // Reserved / bus powered
		.bMaxPower = 250,
	},
	.lidar_interface = {
		.bLength = sizeof(tusb_desc_interface_t),
		.bDescriptorType =	TUSB_DESC_INTERFACE,
		.bInterfaceNumber =	ITF_NUM_LIDAR,
		.bAlternateSetting =	0,
		.bNumEndpoints =	NUM_ENDPOINTS,
		.bInterfaceClass =	0xFF,
		.bInterfaceSubClass =	0xFF,
		.bInterfaceProtocol =	0xFF,
		.iInterface =		4,
	},
	.lidar_ep = {
		.bLength =		sizeof(tusb_desc_endpoint_t),
		.bDescriptorType =	TUSB_DESC_ENDPOINT,
		.bEndpointAddress =	1 | TUSB_DIR_IN_MASK,
		.bmAttributes =		TUSB_XFER_INTERRUPT,
		.wMaxPacketSize =	57,
		.bInterval =		2,
	},
	.reset_interface = {
		.bLength = sizeof(tusb_desc_interface_t),
		.bDescriptorType =	TUSB_DESC_INTERFACE,
		//.bInterfaceNumber =	ITF_NUM_RESET,
		.bAlternateSetting =	0,
		.bNumEndpoints =	0,
		.bInterfaceClass =	0xFF,
		.bInterfaceSubClass =	0,
		.bInterfaceProtocol =	1,
		.iInterface =		5,
	},
};
