/*
 * sen6x.h - Public C API for Sensirion SEN6x environmental sensors.
 *
 * Supports SEN62, SEN63C and SEN66 over I2C with runtime variant
 * auto-detection. The protocol core (this header + sen6x.c) is portable
 * C99 and contains no Arduino or ESP-IDF dependencies; all bus access is
 * delegated to the platform port (sen6x_port.h / sen6x_port.cpp).
 *
 * Copyright (c) 2026 Abhinav (abhinav@ipsator.com)
 * SPDX-License-Identifier: MIT
 *
 * Portions adapted from Sensirion's official BSD-3-Clause embedded I2C
 * drivers (github.com/Sensirion/arduino-i2c-sen66 and related). See
 * README.md for attribution. Verified against datasheet PS_DS_SEN6x,
 * D1 v0.92, December 2025, section 4.8 / Table 26.
 */
#ifndef SEN6X_H
#define SEN6X_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* All SEN6x variants share the same fixed 7-bit I2C address (datasheet
 * Table 25). Variant is therefore detected via Get Product Name, not by
 * scanning addresses. */
#define SEN6X_I2C_ADDR  0x6B

/* Return / status codes. 0 == success, negative == failure. */
typedef enum {
    SEN6X_OK         =  0,
    SEN6X_ERR_ARG    = -1,  /* invalid argument (e.g. NULL pointer)        */
    SEN6X_ERR_IO     = -2,  /* I2C transport error (NACK, bus error, ...)  */
    SEN6X_ERR_CRC    = -3,  /* CRC-8 mismatch on a received data word       */
    SEN6X_ERR_TYPE   = -4,  /* operation not supported by detected variant */
    SEN6X_ERR_CALIB  = -5   /* recalibration rejected by the sensor         */
} sen6x_status_t;

/* Detected sensor variant. */
typedef enum {
    SEN6X_TYPE_UNKNOWN = 0,
    SEN6X_TYPE_SEN62,        /* PM, RH, T                          */
    SEN6X_TYPE_SEN63C,       /* PM, RH, T, CO2                     */
    SEN6X_TYPE_SEN65,        /* PM, RH, T, VOC, NOx                */
    SEN6X_TYPE_SEN66,        /* PM, RH, T, VOC, NOx, CO2           */
    SEN6X_TYPE_SEN68,        /* PM, RH, T, VOC, NOx, HCHO          */
    SEN6X_TYPE_SEN69C        /* PM, RH, T, VOC, NOx, HCHO, CO2     */
} sen6x_type_t;

/* Bus configuration passed to sen6x_init(). Pin/port fields are
 * interpreted by the platform port:
 *   - Arduino (Wire): sda/scl are GPIO numbers, i2c_port is ignored.
 *   - ESP-IDF: sda/scl are GPIO numbers, i2c_port selects the I2C
 *     controller (e.g. 0). Use -1 for "auto" where supported.
 * Max bus speed is 100 kHz (standard mode); higher values are clamped
 * by the port. */
typedef struct {
    int      sda;            /* SDA GPIO number                          */
    int      scl;            /* SCL GPIO number                          */
    uint32_t freq_hz;        /* I2C clock in Hz (<= 100000)              */
    int      i2c_port;       /* I2C controller index (ESP-IDF); -1 auto  */
} sen6x_config_t;

/* Decoded measurement. Any signal the detected variant does not provide,
 * or that the sensor reports as "unknown" (sentinel value), is returned
 * as NAN. Units:
 *   pm*           ug/m3
 *   rh            %RH
 *   temperature   degC
 *   voc_index     VOC index (SEN65/66/68/69C)
 *   nox_index     NOx index (SEN65/66/68/69C)
 *   hcho          formaldehyde, ppb (SEN68/69C only)
 *   co2           ppm       (SEN63C/66/69C only) */
typedef struct {
    float pm1;
    float pm2_5;
    float pm4;
    float pm10;
    float rh;
    float temperature;
    float voc_index;
    float nox_index;
    float hcho;
    float co2;
} sen6x_data_t;

/* Decoded Device Status Register (datasheet section 4.3, Figure 7).
 * "Error" flags are sticky until Device Reset or Read-And-Clear; the
 * fan speed warning is not sticky. Flags that do not apply to the
 * detected variant are always false. */
typedef struct {
    uint32_t raw;               /* full 32-bit register value          */
    bool fan_speed_warning;     /* bit 21 (warning, not sticky)        */
    bool co2_error;             /* bit 12 SEN63C/69C / bit 9 SEN66     */
    bool pm_error;              /* bit 11                              */
    bool hcho_error;            /* bit 10 (SEN68/69C)                  */
    bool gas_error;             /* bit 7  (SEN65/66/68/69C)            */
    bool rht_error;             /* bit 6                               */
    bool fan_error;             /* bit 4                               */
} sen6x_device_status_t;

/* Individual Device Status Register bit masks (datasheet section 4.3). */
#define SEN6X_STATUS_SPEED_WARNING  (1UL << 21)
#define SEN6X_STATUS_CO2_1_ERROR    (1UL << 12)  /* SEN63C / SEN69C    */
#define SEN6X_STATUS_PM_ERROR       (1UL << 11)
#define SEN6X_STATUS_HCHO_ERROR     (1UL << 10)  /* SEN68 / SEN69C     */
#define SEN6X_STATUS_CO2_2_ERROR    (1UL <<  9)  /* SEN66              */
#define SEN6X_STATUS_GAS_ERROR      (1UL <<  7)
#define SEN6X_STATUS_RHT_ERROR      (1UL <<  6)
#define SEN6X_STATUS_FAN_ERROR      (1UL <<  4)

/* ---- Core lifecycle ---------------------------------------------------- */

/* Bring up the I2C bus and soft-reset the device. Must be called first.
 * Performs Device Reset (0xD304) and waits its 1200 ms execution time, so
 * the sensor is left in idle mode ready for sen6x_detect()/sen6x_start().
 * Returns SEN6X_OK or a negative sen6x_status_t. */
int sen6x_init(const sen6x_config_t *cfg);

/* Read Get Product Name (0xD014) and classify the device. Updates the
 * cached type used by sen6x_read(). Returns the detected type
 * (SEN6X_TYPE_UNKNOWN on I2C/CRC error or unrecognized name). */
sen6x_type_t sen6x_detect(void);

/* Start / stop continuous measurement (0x0021 / 0x0104). After start it
 * takes ~1.1 s until the first results; poll sen6x_data_ready(). */
int sen6x_start(void);
int sen6x_stop(void);

/* Get Data Ready (0x0202): true when a new measurement can be read. */
bool sen6x_data_ready(void);

/* Read measured values into *out, selecting the opcode and word count for
 * the detected variant. Fields absent for the variant are set to NAN.
 * Requires a prior successful sen6x_detect(); otherwise SEN6X_ERR_TYPE. */
int sen6x_read(sen6x_data_t *out);

/* Cached detected type and a human-readable name. */
sen6x_type_t sen6x_get_type(void);
const char  *sen6x_type_name(sen6x_type_t t);

/* ---- Optional commands ------------------------------------------------- */

/* Start Fan Cleaning (0x5607). Idle mode only; runs the fan at max speed
 * for ~10 s. Wait >=10 s before starting a measurement. */
int sen6x_start_fan_cleaning(void);

/* Read Device Status (0xD206) and decode it into *out for the detected
 * variant. */
int sen6x_read_device_status(sen6x_device_status_t *out);

/* Set Temperature Offset Parameters (0x60B2). Slot 0..4.
 *   offset_deg_c : constant offset in degC
 *   slope        : normalized slope (dimensionless)
 *   time_const_s : smoothing time constant in seconds (0 = immediate)
 * Scaling per datasheet is applied internally. */
int sen6x_set_temperature_offset(float offset_deg_c, float slope,
                                 uint16_t time_const_s, uint16_t slot);

/* Perform Forced CO2 Recalibration (0x6707). SEN63C/SEN66/SEN69C only, idle
 * mode. On success *correction_ppm (may be NULL) receives the FRC correction
 * (raw return value - 0x8000). Returns SEN6X_ERR_CALIB if the sensor
 * reports failure (0xFFFF), SEN6X_ERR_TYPE if the variant lacks CO2. */
int sen6x_forced_co2_recalibration(uint16_t target_ppm,
                                   int32_t *correction_ppm);

/* Get / Set CO2 Sensor Automatic Self Calibration (0x6711). SEN63C/SEN66/
 * SEN69C only, idle mode. */
int sen6x_get_co2_auto_self_cal(bool *enabled);
int sen6x_set_co2_auto_self_cal(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* SEN6X_H */
