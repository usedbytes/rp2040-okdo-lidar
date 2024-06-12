add_executable(lidar_example
	main.c
	usb.c
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(lidar_example PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. lidar_example)

target_link_libraries(lidar_example
	pico_stdlib
	pico_unique_id
	pico_util
	tinyusb_board
	tinyusb_device

	lidar
)

pico_add_extra_outputs(lidar_example)
