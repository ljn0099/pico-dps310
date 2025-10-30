add_executable(dps310_test
    dps310Test.c
)

target_link_libraries(dps310_test
    pico_stdlib
    hardware_i2c
    sensor_dps310
    Unity
)

add_test(dps310_test dps310_test)

pico_enable_stdio_usb(dps310_test 0)
pico_enable_stdio_uart(dps310_test 1)

pico_add_extra_outputs(dps310_test)
