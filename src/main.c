#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#define PWM_PIN 2
#define RX_PIN 5

#define UART_ID uart1
#define BAUD_RATE 230400

// From: https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf

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

void dump_frame(LiDARFrameTypeDef *frame)
{
	printf("header: %2x\n", frame->header);
	printf("ver_len: %2x\n", frame->ver_len);
	printf("speed: %u\n", frame->speed);
	printf("start: %3.2f\n", frame->start_angle * 0.01);
	printf("end: %3.2f\n", frame->end_angle * 0.01);

	printf("points:");
	int i = 0;
	for (i = 0; i < sizeof(frame->point) / sizeof(frame->point[0]); i++) {
		if (i % 4 == 0) {
			printf("\n");
		}
		LidarPointStructDef *pt = &frame->point[i];
		printf("(%5d mm, %3.2f) ", pt->distance, pt->intensity / 255.0);
	}
	printf("\n");
	printf("timestamp: %u\n", frame->timestamp);
	printf("crc8: %u\n", frame->crc8);
}

static const uint8_t CrcTable[256] ={
	0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
	0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
	0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
	0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
	0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
	0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
	0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
	0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
	0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
	0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
	0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
	0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
	0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
	0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
	0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
	0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
	0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
	0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
	0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
	0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
	0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
	0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
};

uint8_t CalCRC8(uint8_t *p, uint8_t len){
	uint8_t crc = 0;
	uint16_t i;

	for (i = 0; i < len; i++){
		crc = CrcTable[(crc ^ *p++) & 0xff];
	}

	return crc;
}

// End thirdparty

bool frame_valid(LiDARFrameTypeDef *frame)
{
	uint8_t crc = CalCRC8((uint8_t *)frame, sizeof(*frame) - 1);

	return crc == frame->crc8;
}

#define FRAME_SIZE sizeof(LiDARFrameTypeDef)

struct uart_buf {
#define UART_BUF_BITS 7
#define UART_BUF_SIZE (1 << UART_BUF_BITS)
	uint64_t insert;
	uint64_t extract;
	int dma_chan;
	dma_channel_config dma_cfg;
	uint32_t last_nbytes;
	uint8_t *dma_read_addr;
	uint8_t __attribute__((aligned(UART_BUF_SIZE))) buf[UART_BUF_SIZE];
};

struct uart_buf uart_buf;

void uart_buf_request_bytes(struct uart_buf *buf, uint32_t nbytes)
{
	uint8_t *dst = &buf->buf[buf->insert % UART_BUF_SIZE];
	buf->last_nbytes = nbytes;
	dma_channel_configure(buf->dma_chan, &buf->dma_cfg,
	                      dst, buf->dma_read_addr,
	                      nbytes, true);
}

static inline uint32_t min_u32(uint32_t a, uint32_t b)
{
	return a < b ? a : b;
}

void ring_buffer_memcpy(uint8_t *dst, uint8_t *src_base,
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

void handle_frame(LiDARFrameTypeDef *frame)
{
	printf("%p -> (%d) crc: %02x\n", frame, frame->timestamp, frame->crc8);
}

uint32_t uart_buf_scan(struct uart_buf *buf)
{
	for (;;) {
		const uint32_t start_offset = buf->extract % UART_BUF_SIZE;
		const uint32_t available = buf->insert - buf->extract;
		const uint32_t before_wrap = min_u32(available, UART_BUF_SIZE - start_offset);

		if (available == 0) {
			return FRAME_SIZE;
		}

		uint8_t *p = &buf->buf[start_offset];
		const uint8_t *end = p + before_wrap;
		uint32_t consumed = 0;

		while (p < end) {
			if (*p != HEADER) {
				// Not a header, just advance
				p++;
				consumed += 1;
				continue;
			}

			uint32_t remainder = available - consumed;
			if (remainder < FRAME_SIZE) {
				// Not enough data to copy a full packet
				// Request more.
				buf->extract += consumed;
				return FRAME_SIZE - remainder;
			}

			// Full packet available
			LiDARFrameTypeDef frame;

			ring_buffer_memcpy((uint8_t *)&frame, buf->buf, p - buf->buf,
					   UART_BUF_SIZE, sizeof(frame));

			if (frame_valid(&frame)) {
				handle_frame(&frame);
				p += sizeof(frame);
				consumed += sizeof(frame);
			} else {
				p += 1;
				consumed += 1;
			}
		}

		buf->extract += consumed;
	}
}

void dma_irq_handler()
{
	struct uart_buf *buf = &uart_buf;

	buf->insert += buf->last_nbytes;

	uint32_t next_req = uart_buf_scan(buf);

	// Clear the interrupt request, *before* requesting more
	dma_hw->ints0 = 1u << buf->dma_chan;

	uart_buf_request_bytes(buf, next_req);
}

void uart_buf_init(uart_inst_t *uart, struct uart_buf *buf)
{
	memset(buf, 0, sizeof(*buf));
	uart_set_fifo_enabled(uart, true);

	uart_hw_t *uart_hw = uart_get_hw(uart);
	uint dreq = uart_get_dreq(uart, false);

	// Set FIFO watermark to 50% (16)
	uart_hw->ifls = (2 << UART_UARTIFLS_RXIFLSEL_LSB);

	// Set up DMA to transfer from UART to ring-buffer
	buf->dma_chan = dma_claim_unused_channel(true);
	buf->dma_cfg = dma_channel_get_default_config(buf->dma_chan);
	buf->dma_read_addr = (uint8_t *)&uart_hw->dr;

	channel_config_set_read_increment(&buf->dma_cfg, false);
	channel_config_set_write_increment(&buf->dma_cfg, true);
	channel_config_set_dreq(&buf->dma_cfg, dreq);
	channel_config_set_transfer_data_size(&buf->dma_cfg, DMA_SIZE_8);
	channel_config_set_ring(&buf->dma_cfg, true, UART_BUF_BITS);
	channel_config_set_enable(&buf->dma_cfg, true);

	dma_channel_set_irq0_enabled(buf->dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	// Initially request a full packet, and we will adjust when we
	// see the first header.
	uart_buf_request_bytes(buf, FRAME_SIZE);
}

void main(void) {
	stdio_init_all();
	int i = 0;

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	// LD1 wants 30 kHz PWM
	// "Scan rate around 10Hz at PWM 40%"

	gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

	const uint pwm_slice = pwm_gpio_to_slice_num(PWM_PIN);
	const uint32_t sys_clk_rate = clock_get_hz(clk_sys);
	printf("clock: %u\n", sys_clk_rate);

	// 30 kHz PWM, with 1000 ticks a cycle
	const float clock_div = sys_clk_rate / (30000.0 * 1000);
	printf("div: %f\n", clock_div);

	pwm_config cfg = pwm_get_default_config();
	pwm_config_set_clkdiv(&cfg, clock_div);
	pwm_config_set_wrap(&cfg, 1000);

	pwm_init(pwm_slice, &cfg, false);
	pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 400);
	pwm_set_enabled(pwm_slice, true);

	uart_init(UART_ID, BAUD_RATE);
	gpio_set_function(RX_PIN, GPIO_FUNC_UART);

	uint baud = uart_set_baudrate(UART_ID, BAUD_RATE);
	printf("Baud rate: %d\n", baud);

	uart_buf_init(UART_ID, &uart_buf);

	for ( ;; ) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		sleep_ms(300);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		sleep_ms(300);
	}
}
