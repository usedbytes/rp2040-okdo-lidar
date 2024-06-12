// Interface for the OKDO LIDAR_LD06
//
// Copyright 2024 Brian Starkey <stark3y@gmail.com>
#ifndef __LIDAR_H__
#define __LIDAR_H__

// Structure definitions based on:
// https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf
// No copyright attribution mentioned.

#define LIDAR_SAMPLES_PER_FRAME 12
#define LIDAR_FRAME_HEADER 0x54

struct __attribute__((packed)) lidar_sample {
	uint16_t distance_mm;
	uint8_t intensity;
};

struct __attribute__((packed)) lidar_frame {
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	struct lidar_sample samples[LIDAR_SAMPLES_PER_FRAME];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
};

typedef void (*frame_cb_t)(void *priv, struct lidar_frame *frame);

void lidar_init(frame_cb_t frame_cb, void *priv);

void dump_frame(struct lidar_frame *frame);

#endif /* __LIDAR_H__ */
