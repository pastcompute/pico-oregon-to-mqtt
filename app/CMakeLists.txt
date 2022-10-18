add_executable(
        relay_serial
        relay_serial_main.cpp
        )
target_link_libraries(
        relay_serial
        pico_stdlib
        pico_multicore
        hardware_spi
        hardware_adc
        )

        
pico_enable_stdio_usb(relay_serial 0)
pico_enable_stdio_uart(relay_serial 1)

pico_add_extra_outputs(relay_serial)
