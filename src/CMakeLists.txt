add_library(sensor_dps310 STATIC
    ${CMAKE_CURRENT_LIST_DIR}/dps310.c
)

target_include_directories(sensor_dps310
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(sensor_dps310 PUBLIC
    pico_stdlib
    hardware_i2c
)
