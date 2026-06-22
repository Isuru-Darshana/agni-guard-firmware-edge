idf_component_register(
    SRCS
        "src/bme680_bsec.c"
        "src/bme68x.c"
        "src/bme68x_hal.c"
        "src/bsec_iaq.c"
    INCLUDE_DIRS
        "inc"
    REQUIRES
        driver
        nvs_flash
        freertos
        log
        esp_timer
)

target_link_libraries(${COMPONENT_LIB} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libalgobsec.a
)
target_link_libraries(${COMPONENT_LIB} INTERFACE m)