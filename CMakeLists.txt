idf_component_register(SRCS "esp_at.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_system esp_timer freertos)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    FIRMWARE_VERSION="v0.1.0"
)
