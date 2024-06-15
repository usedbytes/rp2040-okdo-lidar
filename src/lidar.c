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
				hw->frame_cb(hw->frame_cb_data, &frame);

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
#if LIDAR_EXCLUSIVE_DMA_IRQ_1
			panic("No interrupts left! Something went wrong.");
#endif
			return NULL;
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

	return hw;
}

void lidar_dma_irq_handler(void)
{
	uint32_t ints = dma_hw->ints1;
	struct lidar_hw *hw = __find_lidar_hw(ints);
	if (!hw) {
#if LIDAR_EXCLUSIVE_DMA_IRQ_1
		panic("Couldn't match interrupt with lidar_hw instance.");
#endif
		return;
	}

	hw->insert += hw->last_nbytes;

	uint32_t next_req = lidar_hw_scan(hw);

	// Clear the interrupt request, *before* requesting more
	dma_hw->ints1 = 1u << hw->dma_chan;

	lidar_hw_request_bytes(hw, next_req);
}

static void lidar_hw_init(struct lidar_hw *hw, uart_inst_t *uart, frame_cb_t frame_cb, void *cb_data)
{
	memset(hw, 0, sizeof(*hw));
	hw->frame_cb = frame_cb;
	hw->frame_cb_data = cb_data;

	uart_hw_t *uart_hw = uart_get_hw(uart);
	uint dreq = uart_get_dreq(uart, false);
	uart_set_fifo_enabled(uart, true);

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

	dma_channel_set_irq1_enabled(hw->dma_chan, true);

#if LIDAR_EXCLUSIVE_DMA_IRQ_1
	irq_set_exclusive_handler(DMA_IRQ_1, lidar_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_1, true);
#endif

	// Initially request a full packet, and we will adjust when we
	// see the first header.
	lidar_hw_request_bytes(hw, LIDAR_FRAME_SIZE);
}

static uart_inst_t *__find_uart_for_pin(uint uart_pin)
{
	const uint32_t uart0_pins = (1 << 1) | (1 << 13) | (1 << 17) | (1 << 29);
	const uint32_t uart1_pins = (1 << 5) | (1 << 9) | (1 << 21) | (1 << 25);

	uint32_t pin_mask = (1 << uart_pin);

	if (pin_mask & uart0_pins) {
		return uart0;
	} else if (pin_mask & uart1_pins) {
		return uart1;
	}

	return NULL;
}

void lidar_init(struct lidar_hw *hw, struct lidar_cfg *cfg)
{
	if (cfg->pwm_pin >= 0) {
		// LD1 wants 30 kHz PWM
		// "Scan rate around 10Hz at PWM 40%"
		const uint pwm_slice = pwm_gpio_to_slice_num(cfg->pwm_pin);
		const uint32_t sys_clk_rate = clock_get_hz(clk_sys);

		// 30 kHz PWM, with 1000 ticks a cycle
		const float clock_div = sys_clk_rate / (30000.0 * 1000);

		pwm_config pwm_cfg = pwm_get_default_config();
		pwm_config_set_clkdiv(&pwm_cfg, clock_div);
		pwm_config_set_wrap(&pwm_cfg, 1000);

		pwm_init(pwm_slice, &pwm_cfg, false);

		const uint pwm_chan = cfg->pwm_pin & 1 ? PWM_CHAN_B : PWM_CHAN_A;
		pwm_set_chan_level(pwm_slice, pwm_chan, 400);
		pwm_set_enabled(pwm_slice, true);

		gpio_set_function(cfg->pwm_pin, GPIO_FUNC_PWM);
	}

	uart_inst_t *uart = __find_uart_for_pin(cfg->uart_pin);
	if (!uart) {
		panic("Invalid uart_pin - couldn't match with a UART instance");
	}

	uart_init(uart, BAUD_RATE);
	gpio_set_function(cfg->uart_pin, GPIO_FUNC_UART);
	uart_set_baudrate(uart, BAUD_RATE);

	lidar_hw_init(hw, uart, cfg->frame_cb, cfg->frame_cb_data);
}
