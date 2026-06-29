/*
 * sen6x_esp.h - ESP-IDF-only extensions for the SEN6x driver.
 *
 * Kept separate from the portable sen6x.h so the core public API stays
 * platform-agnostic. Everything here is guarded by ESP_PLATFORM, so the
 * header is harmless to include on the Arduino build.
 *
 * Copyright (c) 2026 Abhinav (abhinav@ipsator.com)
 * SPDX-License-Identifier: MIT
 */
#pragma once

#if defined(ESP_PLATFORM)

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The I2C master bus the driver created in sen6x_init(). Lets the app add
 * other devices (e.g. an OLED) on the same controller. Returns NULL before
 * sen6x_init() (or on the Arduino build). The driver keeps ownership of the
 * bus; this is a read-only accessor. */
i2c_master_bus_handle_t sen6x_get_i2c_bus(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_PLATFORM */
