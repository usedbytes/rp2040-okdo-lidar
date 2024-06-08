
add_executable(main
	main.c
	lidar.c
	usb.c
)

target_compile_definitions(main PRIVATE
	PICO_DEFAULT_UART_BAUD_RATE=230400
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(main PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

pico_enable_stdio_uart(main 1)
pico_enable_stdio_semihosting(main 0)

target_link_libraries(main
	hardware_dma
	hardware_pwm
	pico_stdlib
	pico_util
	tinyusb_board
	tinyusb_device
)

pico_add_extra_outputs(main)
