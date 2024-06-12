// Interface for the OKDO LIDAR_LD06
// Uses DMA to capture the frame data, validates it, and then passes the
// validated frames to a user-provided callback.
//
// Copyright 2024 Brian Starkey <stark3y@gmail.com>

#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

#include "crc8.h"
#include "lidar.h"

#define PWM_PIN 2
#define RX_PIN 5

#define UART_ID uart1
#define BAUD_RATE 230400

void dump_frame(struct lidar_frame *frame)
{
	printf("header: %2x\n", frame->header);
	printf("ver_len: %2x\n", frame->ver_len);
	printf("speed: %u\n", frame->speed);
	printf("start: %3.2f\n", frame->start_angle * 0.01);
	printf("end: %3.2f\n", frame->end_angle * 0.01);

	printf("samples:");
	int i = 0;
	for (i = 0; i < sizeof(frame->samples) / sizeof(frame->samples[0]); i++) {
		if (i % 4 == 0) {
			printf("\n");
		}
		struct lidar_sample *sample = &frame->samples[i];
		printf("(%5d mm, %3.2f) ", sample->distance_mm, sample->intensity / 255.0);
	}
	printf("\n");
	printf("timestamp: %u\n", frame->timestamp);
	printf("crc8: %u\n", frame->crc8);
}

static bool frame_valid(struct lidar_frame *frame)
{
	uint8_t crc = CalCRC8((uint8_t *)frame, sizeof(*frame) - 1);

	return crc == frame->crc8;
}

// A lidar_frame is 57 bytes.
// We need a well-aligned power-of-two buffer so we can use the DMA's ring-buffer mode.
//
// We process frames serially, so we only need to store one - so 64 bytes
// should be OK.
#define LIDAR_FRAME_SIZE sizeof(struct lidar_frame)
#define LIDAR_HW_BUF_BITS 6
#define LIDAR_HW_BUF_SIZE (1 << LIDAR_HW_BUF_BITS)
static_assert(LIDAR_HW_BUF_SIZE >= LIDAR_FRAME_SIZE);

struct lidar_hw {
	uint8_t __attribute__((aligned(LIDAR_HW_BUF_SIZE))) buf[LIDAR_HW_BUF_SIZE];

	uint64_t insert;
	uint64_t extract;
	int dma_chan;
	dma_channel_config dma_cfg;
	uint32_t last_nbytes;
	uint8_t *dma_read_addr;
	frame_cb_t frame_cb;
	void *frame_cb_priv;
};

struct lidar_hw lidar_hw;

static void lidar_hw_request_bytes(struct lidar_hw *hw, uint32_t nbytes)
{
	uint8_t *dst = &hw->buf[hw->insert % LIDAR_HW_BUF_SIZE];
	hw->last_nbytes = nbytes;
	dma_channel_configure(hw->dma_chan, &hw->dma_cfg,
	                      dst, hw->dma_read_addr,
	                      nbytes, true);
}

static inline uint32_t min_u32(uint32_t a, uint32_t b)
{
	return a < b ? a : b;
}

static void ring_buffer_memcpy(uint8_t *dst, uint8_t *src_base,
                        uint32_t start_offs, uint32_t buf_size,
                        uint32_t size)
{
	uint32_t space = buf_size - start_offs;
	if (space >= size) {
		memcpy(dst, &src_base[start_offs], size);
	} else {
		memcpy(dst, &src_base[start_offs], space);
		memcpy(dst + space, &src_base[0], size - space);
	}
}

static uint32_t lidar_hw_scan(struct lidar_hw *hw)
{
	for (;;) {
		const uint32_t start_offset = hw->extract % LIDAR_HW_BUF_SIZE;
		const uint32_t available = hw->insert - hw->extract;
		const uint32_t before_wrap = min_u32(available, LIDAR_HW_BUF_SIZE - start_offset);

		if (available == 0) {
			return LIDAR_FRAME_SIZE;
		}

		uint8_t *p = &hw->buf[start_offset];
		const uint8_t *end = p + before_wrap;
		uint32_t consumed = 0;

		while (p < end) {
			if (*p != LIDAR_FRAME_HEADER) {
				// Not a header, just advance
				p++;
				consumed += 1;
				continue;
			}

			uint32_t remainder = available - consumed;
			if (remainder < LIDAR_FRAME_SIZE) {
				// Not enough data to copy a full packet
				// Request more.
				hw->extract += consumed;
				return LIDAR_FRAME_SIZE - remainder;
			}

			// Full packet available
			struct lidar_frame frame;

			ring_buffer_memcpy((uint8_t *)&frame, hw->buf, p - hw->buf,
					   LIDAR_HW_BUF_SIZE, sizeof(frame));

			if (frame_valid(&frame)) {
				hw->frame_cb(hw->frame_cb_priv, &frame);

				p += sizeof(frame);
				consumed += sizeof(frame);
			} else {
				p += 1;
				consumed += 1;
			}
		}

		hw->extract += consumed;
	}
}

// We only support HW UARTs, so there can be at most NUM_UARTS instances
// of the lidar.
// We store pointers to the HW structures here, then try and find the matching
// one in the DMA ISR
struct lidar_hw *hw_ctxs[NUM_UARTS];

static struct lidar_hw *__find_lidar_hw(uint32_t ints)
{
	struct lidar_hw *hw = NULL;

	while (ints) {
		int chan = __builtin_ffs(ints);
		if (chan == 0) {
			panic("No interrupts left! Something went wrong.");
		}

		chan = chan - 1;
		ints &= ~(1 << chan);

		for (int i = 0; i < NUM_UARTS; i++) {
			struct lidar_hw *tmp = hw_ctxs[i];
			if (!tmp) {
				continue;
			}

			// TODO: Could also check the DMA pointer(s) as an
			// extra check, but this is probably fine.
			if (tmp->dma_chan == chan) {
				hw = tmp;
				break;
			}
		}
	}

	if (!hw) {
		panic("Couldn't match interrupt with lidar_hw instance.");
	}

	return hw;
}

static void dma_irq_handler()
{
	uint32_t ints = dma_hw->ints0;
	struct lidar_hw *hw = __find_lidar_hw(ints);

	hw->insert += hw->last_nbytes;

	uint32_t next_req = lidar_hw_scan(hw);

	// Clear the interrupt request, *before* requesting more
	dma_hw->ints0 = 1u << hw->dma_chan;

	lidar_hw_request_bytes(hw, next_req);
}

static void lidar_hw_init(uart_inst_t *uart, struct lidar_hw *hw, frame_cb_t frame_cb, void *priv)
{
	memset(hw, 0, sizeof(*hw));
	hw->frame_cb = frame_cb;
	hw->frame_cb_priv = priv;
	uart_set_fifo_enabled(uart, true);

	uart_hw_t *uart_hw = uart_get_hw(uart);
	uint dreq = uart_get_dreq(uart, false);

	// Set FIFO watermark to 50% (16)
	uart_hw->ifls = (2 << UART_UARTIFLS_RXIFLSEL_LSB);

	// Set up DMA to transfer from UART to ring-buffer
	hw->dma_chan = dma_claim_unused_channel(true);
	hw->dma_cfg = dma_channel_get_default_config(hw->dma_chan);
	hw->dma_read_addr = (uint8_t *)&uart_hw->dr;

	// Store our context so the ISR can get at it
	hw_ctxs[uart_get_index(uart)] = hw;

	channel_config_set_read_increment(&hw->dma_cfg, false);
	channel_config_set_write_increment(&hw->dma_cfg, true);
	channel_config_set_dreq(&hw->dma_cfg, dreq);
	channel_config_set_transfer_data_size(&hw->dma_cfg, DMA_SIZE_8);
	channel_config_set_ring(&hw->dma_cfg, true, LIDAR_HW_BUF_BITS);
	channel_config_set_enable(&hw->dma_cfg, true);

	dma_channel_set_irq0_enabled(hw->dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	// Initially request a full packet, and we will adjust when we
	// see the first header.
	lidar_hw_request_bytes(hw, LIDAR_FRAME_SIZE);
}

void lidar_init(frame_cb_t frame_cb, void *priv)
{
	// LD1 wants 30 kHz PWM
	// "Scan rate around 10Hz at PWM 40%"

	gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

	const uint pwm_slice = pwm_gpio_to_slice_num(PWM_PIN);
	const uint32_t sys_clk_rate = clock_get_hz(clk_sys);

	// 30 kHz PWM, with 1000 ticks a cycle
	const float clock_div = sys_clk_rate / (30000.0 * 1000);

	pwm_config cfg = pwm_get_default_config();
	pwm_config_set_clkdiv(&cfg, clock_div);
	pwm_config_set_wrap(&cfg, 1000);

	pwm_init(pwm_slice, &cfg, false);
	pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 400);
	pwm_set_enabled(pwm_slice, true);

	uart_init(UART_ID, BAUD_RATE);
	gpio_set_function(RX_PIN, GPIO_FUNC_UART);

	uint baud = uart_set_baudrate(UART_ID, BAUD_RATE);

	lidar_hw_init(UART_ID, &lidar_hw, frame_cb, priv);
}
