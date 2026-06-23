/*
 * sen6x_port.cpp - Single transport implementation for the SEN6x core.
 *
 * One source file, selected at compile time:
 *   ARDUINO       -> Wire (TwoWire)
 *   ESP_PLATFORM  -> ESP-IDF new driver/i2c_master API
 *
 * ARDUINO is checked first on purpose: when Arduino is built as an
 * ESP-IDF component ("arduino-as-component"), BOTH ARDUINO and
 * ESP_PLATFORM are defined, and we want the Wire path in that case.
 *
 * Per Sensirion's protocol the controller must issue a STOP and wait the
 * command execution time before reading, so write and read are separate
 * transactions here (no combined transmit-receive / repeated start).
 *
 * Copyright (c) 2026 Abhinav (abhinav@ipsator.com)
 * SPDX-License-Identifier: MIT
 */
#include "sen6x_port.h"

/* ===================================================================== */
#if defined(ARDUINO)
/* ===================================================================== */

#include <Arduino.h>
#include <Wire.h>

static TwoWire *s_wire = &Wire;

extern "C" int sen6x_port_init(const sen6x_port_config_t *cfg)
{
    if (cfg == nullptr) {
        return -1;
    }
    s_wire = &Wire;
    /* On ESP32 cores Wire.begin(sda, scl, freq) is available. */
    bool ok = s_wire->begin(cfg->sda, cfg->scl, cfg->freq_hz);
    if (!ok) {
        return -1;
    }
    /* Clamp to the SEN6x maximum of 100 kHz (standard mode). */
    uint32_t freq = (cfg->freq_hz == 0 || cfg->freq_hz > 100000UL)
                        ? 100000UL : cfg->freq_hz;
    s_wire->setClock(freq);
    return 0;
}

extern "C" int sen6x_port_deinit(void)
{
    if (s_wire != nullptr) {
        s_wire->end();
    }
    return 0;
}

extern "C" int sen6x_port_write(uint8_t addr, const uint8_t *data, size_t len)
{
    s_wire->beginTransmission(addr);
    if (len > 0) {
        size_t written = s_wire->write(data, len);
        if (written != len) {
            s_wire->endTransmission(true);
            return -1;
        }
    }
    /* endTransmission(true) issues a STOP; 0 == success. */
    return (s_wire->endTransmission(true) == 0) ? 0 : -1;
}

extern "C" int sen6x_port_read(uint8_t addr, uint8_t *data, size_t len)
{
    size_t got = s_wire->requestFrom((int)addr, (int)len, (int)true);
    if (got != len) {
        return -1;
    }
    for (size_t i = 0; i < len; ++i) {
        int b = s_wire->read();
        if (b < 0) {
            return -1;
        }
        data[i] = (uint8_t)b;
    }
    return 0;
}

extern "C" void sen6x_port_delay_ms(uint32_t ms)
{
    delay(ms);
}

/* ===================================================================== */
#elif defined(ESP_PLATFORM)
/* ===================================================================== */

#include "sen6x.h"          /* for the canonical SEN6X_I2C_ADDR */
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define SEN6X_XFER_TIMEOUT_MS  1000

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

extern "C" int sen6x_port_init(const sen6x_port_config_t *cfg)
{
    if (cfg == nullptr) {
        return -1;
    }
    /* Re-init cleanly if called twice. */
    sen6x_port_deinit();

    uint32_t freq = (cfg->freq_hz == 0 || cfg->freq_hz > 100000UL)
                        ? 100000UL : cfg->freq_hz;

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    /* i2c_port == -1 lets the driver auto-select a free controller. */
    bus_cfg.i2c_port                     = (cfg->i2c_port < 0)
                                               ? -1 : cfg->i2c_port;
    bus_cfg.sda_io_num                   = (gpio_num_t)cfg->sda;
    bus_cfg.scl_io_num                   = (gpio_num_t)cfg->scl;
    bus_cfg.glitch_ignore_cnt            = 7;
    bus_cfg.flags.enable_internal_pullup = true; /* external ~10k still required */

    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        return -1;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = SEN6X_I2C_ADDR;   /* 0x6B, 7-bit */
    dev_cfg.scl_speed_hz    = freq;

    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return -1;
    }
    return 0;
}

extern "C" int sen6x_port_deinit(void)
{
    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus != NULL) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    return 0;
}

extern "C" int sen6x_port_write(uint8_t addr, const uint8_t *data, size_t len)
{
    (void)addr; /* device address fixed at bus_add_device time */
    if (s_dev == NULL) {
        return -1;
    }
    return (i2c_master_transmit(s_dev, data, len, SEN6X_XFER_TIMEOUT_MS) == ESP_OK)
               ? 0 : -1;
}

extern "C" int sen6x_port_read(uint8_t addr, uint8_t *data, size_t len)
{
    (void)addr;
    if (s_dev == NULL) {
        return -1;
    }
    return (i2c_master_receive(s_dev, data, len, SEN6X_XFER_TIMEOUT_MS) == ESP_OK)
               ? 0 : -1;
}

extern "C" void sen6x_port_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ===================================================================== */
#else
/* ===================================================================== */
#error "sen6x_port.cpp: no supported platform. Define ARDUINO or ESP_PLATFORM."
#endif
