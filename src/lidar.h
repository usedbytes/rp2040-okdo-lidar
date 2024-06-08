#ifndef __LIDAR_H__
#define __LIDAR_H__

// From: https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf
// No copyright attribution mentioned.

#define POINT_PER_PACK 12
#define HEADER 0x54
typedef struct __attribute__((packed)) {
	uint16_t distance;
	uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
} LiDARFrameTypeDef;

// End thirdparty

void lidar_init(queue_t *frame_queue);

void dump_frame(LiDARFrameTypeDef *frame);

#endif /* __LIDAR_H__ */
