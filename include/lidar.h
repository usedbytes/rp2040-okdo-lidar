// Interface for the OKDO LIDAR_LD06
//
// Copyright 2024 Brian Starkey <stark3y@gmail.com>
#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <stdint.h>

#include "hardware/dma.h"

// By default, this library takes exclusive control of DMA IRQ1.
// If you don't want that, you can set this to zero, but you _MUST_ register
// and enable a handler for DMA_IRQ_1 yourself, and call lidar_dma_irq_handler()
// from that handler!
#define LIDAR_EXCLUSIVE_DMA_IRQ_1 1

void lidar_dma_irq_handler(void);

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

typedef void (*frame_cb_t)(void *cb_data, struct lidar_frame *frame);

// Populate this structure with your desired values and pass it to lidar_init.
struct lidar_cfg {
	// The UART RX pin connected to the LIDAR. lidar_init will claim the
	// corresponding UART HW.
	uint uart_pin;
	// The PWM pin connected to the LIDAR. lidar_init will claim the
	// corresponding DMA slice. If PWM control is not used or desired,
	// set to -1.
	int pwm_pin;

	// Callback function which will be called for each received valid frame.
	// It will receive frame_cb_data as its cb_data argument.
	// This is called from the DMA IRQ, so only do things which are OK in
	// interrupt context.
	frame_cb_t frame_cb;
	void *frame_cb_data;
};

// A lidar_frame is 57 bytes.
// We need a well-aligned power-of-two buffer so we can use the DMA's ring-buffer mode.
//
// We process frames serially, so we only need to store one - so 64 bytes
// should be OK.
#define LIDAR_FRAME_SIZE sizeof(struct lidar_frame)
#define LIDAR_HW_BUF_BITS 6
#define LIDAR_HW_BUF_SIZE (1 << LIDAR_HW_BUF_BITS)
static_assert(LIDAR_HW_BUF_SIZE >= LIDAR_FRAME_SIZE);

// This structure stores the internal state of the lidar driver.
// You should NOT directly access anything in this structure!
// The definition is only provided so that the library can be used with zero
// dynamic allocations.
struct lidar_hw {
	uint8_t __attribute__((aligned(LIDAR_HW_BUF_SIZE))) buf[LIDAR_HW_BUF_SIZE];

	uint64_t insert;
	uint64_t extract;
	int dma_chan;
	dma_channel_config dma_cfg;
	uint32_t last_nbytes;
	uint8_t *dma_read_addr;
	frame_cb_t frame_cb;
	void *frame_cb_data;
};

// Initialise and start handling data from the lidar.
//
// 'cfg' describes the pins to use and the callback for each valid frame.
// It is only used during lidar_init, and can safely be freed/go out of scope
// once lidar_init returns.
//
// 'hw' stores the internal driver state - all initialisation is done by
// lidar_init. This structure MUST NOT be freed or go out of scope.
void lidar_init(struct lidar_hw *hw, struct lidar_cfg *cfg);

// Print a textual representation of a lidar frame to stdout.
void dump_frame(struct lidar_frame *frame);

#endif /* __LIDAR_H__ */
