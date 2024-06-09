
tusb_desc_device_t const lidar_device_descriptor = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x1010,

	.bDeviceClass       = TUSB_CLASS_MISC,
	.bDeviceSubClass    = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol    = MISC_PROTOCOL_IAD,

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
	"Raw data",
	"reset",
	"Serial data",
};

struct __attribute__((packed)) lidar_config_descriptor {
	tusb_desc_configuration_t config;
	uint8_t cdc_descriptor[TUD_CDC_DESC_LEN];
	tusb_desc_interface_t lidar_interface;
	tusb_desc_endpoint_t lidar_ep;
	tusb_desc_interface_t reset_interface;
};

enum {
	ITF_NUM_CDC,
	ITF_NUM_CDC_DATA,
	ITF_NUM_LIDAR,
	//ITF_NUM_RESET,
	ITF_NUM_TOTAL,
};

#define NUM_ENDPOINTS 4
#define EP_CDC_NOTIF (1 | TUSB_DIR_IN_MASK)
#define EP_CDC_OUT   (2)
#define EP_CDC_IN    (3 | TUSB_DIR_IN_MASK)
#define EP_LIDAR_IN  (4 | TUSB_DIR_IN_MASK)

struct lidar_config_descriptor lidar_config_descriptor = {
	.config = {
		.bLength = sizeof(tusb_desc_configuration_t),
		.bDescriptorType = TUSB_DESC_CONFIGURATION,
		.wTotalLength = sizeof(tusb_desc_configuration_t) +
			TUD_CDC_DESC_LEN +
			sizeof(tusb_desc_interface_t) +
			sizeof(tusb_desc_endpoint_t),
		.bNumInterfaces = ITF_NUM_TOTAL,
		.bConfigurationValue = 1,
		.iConfiguration = 0, // String index
		.bmAttributes = 0x80, // Reserved / bus powered
		.bMaxPower = 250,
	},
	.cdc_descriptor = { TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 6, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64) },
	.lidar_interface = {
		.bLength = sizeof(tusb_desc_interface_t),
		.bDescriptorType =	TUSB_DESC_INTERFACE,
		.bInterfaceNumber =	ITF_NUM_LIDAR,
		.bAlternateSetting =	0,
		.bNumEndpoints =	1,
		.bInterfaceClass =	0xFF,
		.bInterfaceSubClass =	0xFF,
		.bInterfaceProtocol =	0xFF,
		.iInterface =		4,
	},
	.lidar_ep = {
		.bLength =		sizeof(tusb_desc_endpoint_t),
		.bDescriptorType =	TUSB_DESC_ENDPOINT,
		.bEndpointAddress =	EP_LIDAR_IN,
		.bmAttributes =		TUSB_XFER_INTERRUPT,
		.wMaxPacketSize =	57,
		.bInterval =		2,
	},
	/*
	.reset_interface = {
		.bLength = sizeof(tusb_desc_interface_t),
		.bDescriptorType =	TUSB_DESC_INTERFACE,
		.bInterfaceNumber =	ITF_NUM_RESET,
		.bAlternateSetting =	0,
		.bNumEndpoints =	0,
		.bInterfaceClass =	0xFF,
		.bInterfaceSubClass =	0,
		.bInterfaceProtocol =	1,
		.iInterface =		5,
	},
	*/
};
