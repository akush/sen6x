# ESP-IDF component registration (ignored by the Arduino IDE).
#
# Place this repository under your project's `components/` directory (or add
# it via the component manager) and ESP-IDF will pick it up automatically.
#
# The port (src/sen6x_port.cpp) uses the new I2C master API
# (driver/i2c_master.h), which lives in the esp_driver_i2c component since
# ESP-IDF 5.2 (this driver's stated minimum). We require esp_driver_i2c
# directly rather than the legacy `driver` meta-component: on ESP-IDF 6.0 the
# `driver` meta-component no longer propagates the per-peripheral include
# paths, so `REQUIRES driver` breaks the build. Requiring esp_driver_i2c works
# on 5.2 through 6.0. (FreeRTOS and esp_err come from the common requirements.)

idf_component_register(
    SRCS
        "src/sen6x.c"
        "src/sen6x_port.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        esp_driver_i2c
)
