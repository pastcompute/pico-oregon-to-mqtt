add_executable(
        relay_serial
        relay_serial_main.cpp
        )
target_link_libraries(relay_serial pico_stdlib)
pico_add_extra_outputs(relay_serial)
