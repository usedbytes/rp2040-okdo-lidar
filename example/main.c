#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "tusb.h"

#include "lidar.h"
#include "usb.h"

void main(void) {
	stdio_init_all();

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	queue_t frame_queue;
	queue_init(&frame_queue, sizeof(LiDARFrameTypeDef), 8);

	usb_init();
	lidar_init(&frame_queue);

	int i = 0;

	for ( ;; ) {
		LiDARFrameTypeDef frame;
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		if (queue_try_remove(&frame_queue, &frame)) {
			gpio_put(PICO_DEFAULT_LED_PIN, 1);
			//printf("Frame: %d\n", frame.timestamp);
			usb_handle_frame(&frame);

			if (i % 320 == 0) {
				printf("Speed: %d\n", frame.speed);
				printf("Angle: %.3f\n", (frame.end_angle - frame.start_angle) * 0.01);
			}
			i++;
		}

		sleep_ms(1);
		tud_task();
	}
}