target_sources(lidar INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/lidar.c
	${CMAKE_CURRENT_LIST_DIR}/crc8.c
)

target_link_libraries(lidar INTERFACE
	hardware_dma
	hardware_pwm
	pico_stdlib
	pico_util
)
