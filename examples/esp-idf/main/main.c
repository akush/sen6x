/*
 * main.c - Minimal SEN6x example for ESP-IDF (>= 5.2).
 *
 * Wiring (3.3 V ONLY):
 *   SEN6x VDD -> 3V3
 *   SEN6x GND -> GND
 *   SEN6x SDA -> GPIO 8  (with external ~10k pull-up to 3V3)
 *   SEN6x SCL -> GPIO 9  (with external ~10k pull-up to 3V3)
 *   SEN6x SEL -> GND
 */
#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sen6x.h"

#define SDA_GPIO   8
#define SCL_GPIO   9

static const char *TAG = "sen6x_example";

static void log_value(const char *label, float v, const char *unit)
{
    if (isnan(v)) {
        ESP_LOGI(TAG, "%-7s n/a", label);
    } else {
        ESP_LOGI(TAG, "%-7s %.2f %s", label, v, unit);
    }
}

void app_main(void)
{
    sen6x_config_t cfg = {
        .sda      = SDA_GPIO,
        .scl      = SCL_GPIO,
        .freq_hz  = 100000,   /* SEN6x max is 100 kHz */
        .i2c_port = -1,       /* auto-select a free I2C controller */
    };

    if (sen6x_init(&cfg) != SEN6X_OK) {
        ESP_LOGE(TAG, "sen6x_init failed (check wiring / pull-ups)");
        return;
    }

    sen6x_type_t type = sen6x_detect();
    ESP_LOGI(TAG, "Detected sensor: %s", sen6x_type_name(type));
    if (type == SEN6X_TYPE_UNKNOWN) {
        ESP_LOGE(TAG, "Could not identify the sensor; aborting.");
        return;
    }

    if (sen6x_start() != SEN6X_OK) {
        ESP_LOGE(TAG, "sen6x_start failed");
        return;
    }
    ESP_LOGI(TAG, "Measurement started (~1.1 s to first data).");

    for (;;) {
        if (!sen6x_data_ready()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        sen6x_data_t d;
        if (sen6x_read(&d) != SEN6X_OK) {
            ESP_LOGW(TAG, "sen6x_read failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "---");
        log_value("PM1.0",  d.pm1,         "ug/m3");
        log_value("PM2.5",  d.pm2_5,       "ug/m3");
        log_value("PM4.0",  d.pm4,         "ug/m3");
        log_value("PM10",   d.pm10,        "ug/m3");
        log_value("RH",     d.rh,          "%");
        log_value("T",      d.temperature, "degC");
        log_value("VOC",    d.voc_index,   "idx");
        log_value("NOx",    d.nox_index,   "idx");
        log_value("HCHO",   d.hcho,        "ppb");
        log_value("CO2",    d.co2,         "ppm");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
