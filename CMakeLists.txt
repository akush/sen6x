# ESP-IDF component registration (ignored by the Arduino IDE).
#
# Place this repository under your project's `components/` directory (or add
# it via the component manager) and ESP-IDF will pick it up automatically.
#
# REQUIRES driver  -> new i2c_master API (driver/i2c_master.h), ESP-IDF >= 5.2
# REQUIRES esp_driver_i2c is folded into `driver` on 5.2+, so requiring
# `driver` is sufficient.

idf_component_register(
    SRCS
        "src/sen6x.c"
        "src/sen6x_port.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        driver
)
