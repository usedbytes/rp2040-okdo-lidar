#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/util/queue.h"

#include "lidar.h"

void main(void) {
	stdio_init_all();
	int i = 0;

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	queue_t frame_queue;
	queue_init(&frame_queue, sizeof(LiDARFrameTypeDef), 8);

	lidar_init(&frame_queue);

	for ( ;; ) {
		LiDARFrameTypeDef frame;
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		queue_remove_blocking(&frame_queue, &frame);
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		printf("Frame: %d\n", frame.timestamp);
	}
}
