cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(spi_lcd_demo)

pico_sdk_init()

add_executable(spi_lcd_demo)

target_sources(spi_lcd_demo PRIVATE
	spi_lcd_demo.c
	bb_spi_lcd.c
        bb_spi_lcd.h
	gif.c
	AnimatedGIF.h
	)

target_link_libraries(spi_lcd_demo pico_stdlib hardware_spi hardware_dma)

#enable serial output through usb
pico_enable_stdio_usb(spi_lcd_demo 1)
pico_enable_stdio_uart(spi_lcd_demo 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(spi_lcd_demo)

