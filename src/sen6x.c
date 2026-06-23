/*
 * sen6x.c - Portable C99 protocol core for Sensirion SEN6x sensors.
 *
 * Contains the CRC-8 implementation, command framing, CRC-checked word
 * parsing, conditional per-variant decoding and auto-detection. Uses no
 * platform headers: every byte goes through sen6x_port.h. Must compile
 * clean with: gcc -std=c99 -Wall -Wextra -c sen6x.c
 *
 * Copyright (c) 2026 Abhinav (abhinav@ipsator.com)
 * SPDX-License-Identifier: MIT
 *
 * CRC-8, framing and command set adapted from Sensirion's BSD-3-Clause
 * embedded I2C drivers; verified against datasheet PS_DS_SEN6x, D1 v0.92,
 * December 2025 (sections 4.3, 4.8 / Table 26, 4.9 / Table 65).
 */
#include "sen6x.h"
#include "sen6x_port.h"

#include <math.h>
#include <string.h>

/* ---- Command IDs (datasheet Table 26). Carry an internal 3-bit CRC, so
 * no CRC byte is appended when transmitting them. ----------------------- */
#define CMD_START_MEASUREMENT      0x0021u
#define CMD_STOP_MEASUREMENT       0x0104u
#define CMD_GET_DATA_READY         0x0202u
#define CMD_READ_VALUES_SEN62      0x04A3u
#define CMD_READ_VALUES_SEN63C     0x0471u
#define CMD_READ_VALUES_SEN65      0x0446u
#define CMD_READ_VALUES_SEN66      0x0300u
#define CMD_READ_VALUES_SEN68      0x0467u
#define CMD_READ_VALUES_SEN69C     0x04B5u
#define CMD_SET_TEMP_OFFSET        0x60B2u
#define CMD_GET_PRODUCT_NAME       0xD014u
#define CMD_READ_DEVICE_STATUS     0xD206u
#define CMD_DEVICE_RESET           0xD304u
#define CMD_START_FAN_CLEANING     0x5607u
#define CMD_FORCED_CO2_RECAL       0x6707u
#define CMD_CO2_AUTO_SELF_CAL      0x6711u

/* ---- Execution times in ms (datasheet Table 26). --------------------- */
#define EXEC_START_MS              50u
#define EXEC_STOP_MS               1400u
#define EXEC_GENERIC_MS            20u    /* most read/write commands */
#define EXEC_RESET_MS              1200u
#define EXEC_FAN_CLEANING_MS       20u
#define EXEC_FORCED_RECAL_MS       500u

/* CRC-8 Dallas/Maxim (datasheet Table 65): poly 0x31, init 0xFF, no
 * reflection, no final XOR. */
#define CRC8_POLYNOMIAL            0x31u
#define CRC8_INIT                  0xFFu

/* Sentinels for "unknown" values (datasheet sections 4.8.4 - 4.8.7). */
#define SENTINEL_U16               0xFFFFu
#define SENTINEL_S16               0x7FFFu

/* Buffer sizing. Largest read is Get Product Name (48 bytes); largest
 * write is Set Temperature Offset (command + 4 words = 14 bytes). */
#define SEN6X_MAX_RX               48u
#define SEN6X_MAX_WORDS            16u
#define SEN6X_MAX_TX_WORDS         4u

/* Module state. The bus address is fixed for every variant. */
static sen6x_type_t g_type = SEN6X_TYPE_UNKNOWN;

/* --------------------------------------------------------------------- */
/* CRC-8                                                                  */
/* --------------------------------------------------------------------- */

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = CRC8_INIT;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80u) {
                crc = (uint8_t)(((unsigned)crc << 1) ^ CRC8_POLYNOMIAL);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* --------------------------------------------------------------------- */
/* Low-level framing helpers                                             */
/* --------------------------------------------------------------------- */

/* Transmit a bare 16-bit command ID (no CRC), then wait its execution
 * time so the sensor can prepare any response. */
static sen6x_status_t send_command(uint16_t cmd, uint32_t exec_ms)
{
    uint8_t frame[2];
    frame[0] = (uint8_t)(cmd >> 8);
    frame[1] = (uint8_t)(cmd & 0xFFu);
    if (sen6x_port_write(SEN6X_I2C_ADDR, frame, sizeof frame) != 0) {
        return SEN6X_ERR_IO;
    }
    if (exec_ms != 0u) {
        sen6x_port_delay_ms(exec_ms);
    }
    return SEN6X_OK;
}

/* Transmit a command ID followed by argument words, each with its CRC,
 * then wait the execution time. */
static sen6x_status_t send_command_args(uint16_t cmd, const uint16_t *args,
                                        size_t nargs, uint32_t exec_ms)
{
    uint8_t frame[2 + SEN6X_MAX_TX_WORDS * 3];
    size_t n = 0;

    frame[n++] = (uint8_t)(cmd >> 8);
    frame[n++] = (uint8_t)(cmd & 0xFFu);
    for (size_t i = 0; i < nargs; ++i) {
        frame[n]   = (uint8_t)(args[i] >> 8);
        frame[n+1] = (uint8_t)(args[i] & 0xFFu);
        frame[n+2] = crc8(&frame[n], 2);
        n += 3;
    }
    if (sen6x_port_write(SEN6X_I2C_ADDR, frame, n) != 0) {
        return SEN6X_ERR_IO;
    }
    if (exec_ms != 0u) {
        sen6x_port_delay_ms(exec_ms);
    }
    return SEN6X_OK;
}

/* Read nwords data words, validating one CRC byte per word, into out. */
static sen6x_status_t read_words(uint16_t *out, size_t nwords)
{
    uint8_t buf[SEN6X_MAX_RX];
    size_t nbytes = nwords * 3u;

    if (nbytes > sizeof buf) {
        return SEN6X_ERR_ARG;
    }
    if (sen6x_port_read(SEN6X_I2C_ADDR, buf, nbytes) != 0) {
        return SEN6X_ERR_IO;
    }
    for (size_t i = 0; i < nwords; ++i) {
        const uint8_t *w = &buf[i * 3u];
        if (crc8(w, 2) != w[2]) {
            return SEN6X_ERR_CRC;
        }
        out[i] = (uint16_t)(((uint16_t)w[0] << 8) | w[1]);
    }
    return SEN6X_OK;
}

/* Send a command and read back its CRC-checked response words. */
static sen6x_status_t query_words(uint16_t cmd, uint32_t exec_ms,
                                  uint16_t *out, size_t nwords)
{
    sen6x_status_t st = send_command(cmd, exec_ms);
    if (st != SEN6X_OK) {
        return st;
    }
    return read_words(out, nwords);
}

/* --------------------------------------------------------------------- */
/* Value decoding (datasheet sections 4.8.4 - 4.8.7)                     */
/* --------------------------------------------------------------------- */

static float decode_pm(uint16_t w)   { return (w == SENTINEL_U16) ? NAN : (float)w / 10.0f; }
static float decode_rh(uint16_t w)   { return (w == SENTINEL_S16) ? NAN : (float)(int16_t)w / 100.0f; }
static float decode_t(uint16_t w)    { return (w == SENTINEL_S16) ? NAN : (float)(int16_t)w / 200.0f; }
static float decode_idx(uint16_t w)  { return (w == SENTINEL_S16) ? NAN : (float)(int16_t)w / 10.0f; }
static float decode_hcho(uint16_t w) { return (w == SENTINEL_U16) ? NAN : (float)w / 10.0f; }     /* ppb */
static float decode_co2_s(uint16_t w){ return (w == SENTINEL_S16) ? NAN : (float)(int16_t)w; }   /* SEN63C/69C */
static float decode_co2_u(uint16_t w){ return (w == SENTINEL_U16) ? NAN : (float)w; }            /* SEN66  */

static void clear_data(sen6x_data_t *d)
{
    d->pm1 = d->pm2_5 = d->pm4 = d->pm10 = NAN;
    d->rh = d->temperature = NAN;
    d->voc_index = d->nox_index = d->hcho = d->co2 = NAN;
}

/* --------------------------------------------------------------------- */
/* Public API                                                            */
/* --------------------------------------------------------------------- */

int sen6x_init(const sen6x_config_t *cfg)
{
    sen6x_port_config_t pc;
    sen6x_status_t st;

    if (cfg == NULL) {
        return SEN6X_ERR_ARG;
    }
    g_type = SEN6X_TYPE_UNKNOWN;

    pc.sda      = cfg->sda;
    pc.scl      = cfg->scl;
    pc.freq_hz  = cfg->freq_hz;
    pc.i2c_port = cfg->i2c_port;
    if (sen6x_port_init(&pc) != 0) {
        return SEN6X_ERR_IO;
    }

    /* Soft reset to a known idle state; Device Reset waits 1200 ms. */
    st = send_command(CMD_DEVICE_RESET, EXEC_RESET_MS);
    if (st != SEN6X_OK) {
        return st;
    }
    return SEN6X_OK;
}

sen6x_type_t sen6x_detect(void)
{
    uint16_t words[SEN6X_MAX_WORDS];
    char name[SEN6X_MAX_WORDS * 2 + 1];

    g_type = SEN6X_TYPE_UNKNOWN;
    if (query_words(CMD_GET_PRODUCT_NAME, EXEC_GENERIC_MS, words,
                    SEN6X_MAX_WORDS) != SEN6X_OK) {
        return SEN6X_TYPE_UNKNOWN;
    }
    for (size_t i = 0; i < SEN6X_MAX_WORDS; ++i) {
        name[i * 2]     = (char)(words[i] >> 8);
        name[i * 2 + 1] = (char)(words[i] & 0xFFu);
    }
    name[sizeof name - 1] = '\0';

    /* Product names are like "SEN66", "SEN63C", "SEN69C". Each 5-char tag
     * is distinct, so match order is not significant. */
    if (strstr(name, "SEN62") != NULL) {
        g_type = SEN6X_TYPE_SEN62;
    } else if (strstr(name, "SEN63") != NULL) {
        g_type = SEN6X_TYPE_SEN63C;
    } else if (strstr(name, "SEN65") != NULL) {
        g_type = SEN6X_TYPE_SEN65;
    } else if (strstr(name, "SEN66") != NULL) {
        g_type = SEN6X_TYPE_SEN66;
    } else if (strstr(name, "SEN68") != NULL) {
        g_type = SEN6X_TYPE_SEN68;
    } else if (strstr(name, "SEN69") != NULL) {
        g_type = SEN6X_TYPE_SEN69C;
    }
    return g_type;
}

int sen6x_start(void)
{
    return send_command(CMD_START_MEASUREMENT, EXEC_START_MS);
}

int sen6x_stop(void)
{
    return send_command(CMD_STOP_MEASUREMENT, EXEC_STOP_MS);
}

bool sen6x_data_ready(void)
{
    uint16_t word;
    if (query_words(CMD_GET_DATA_READY, EXEC_GENERIC_MS, &word, 1) != SEN6X_OK) {
        return false;
    }
    /* Word = [byte0 padding 0x00][byte1 ready flag]. */
    return (word & 0xFFu) != 0u;
}

int sen6x_read(sen6x_data_t *out)
{
    uint16_t w[10];
    uint16_t cmd;
    size_t nwords;
    sen6x_status_t st;

    if (out == NULL) {
        return SEN6X_ERR_ARG;
    }
    clear_data(out);

    switch (g_type) {
    case SEN6X_TYPE_SEN62:  cmd = CMD_READ_VALUES_SEN62;  nwords = 6;  break;
    case SEN6X_TYPE_SEN63C: cmd = CMD_READ_VALUES_SEN63C; nwords = 7;  break;
    case SEN6X_TYPE_SEN65:  cmd = CMD_READ_VALUES_SEN65;  nwords = 8;  break;
    case SEN6X_TYPE_SEN66:  cmd = CMD_READ_VALUES_SEN66;  nwords = 9;  break;
    case SEN6X_TYPE_SEN68:  cmd = CMD_READ_VALUES_SEN68;  nwords = 9;  break;
    case SEN6X_TYPE_SEN69C: cmd = CMD_READ_VALUES_SEN69C; nwords = 10; break;
    default:
        return SEN6X_ERR_TYPE;
    }

    st = query_words(cmd, EXEC_GENERIC_MS, w, nwords);
    if (st != SEN6X_OK) {
        return st;
    }

    /* Words [0..5] are common to all variants. */
    out->pm1         = decode_pm(w[0]);
    out->pm2_5       = decode_pm(w[1]);
    out->pm4         = decode_pm(w[2]);
    out->pm10        = decode_pm(w[3]);
    out->rh          = decode_rh(w[4]);
    out->temperature = decode_t(w[5]);

    /* Variant-specific tail (datasheet sections 4.8.5 - 4.8.9). */
    switch (g_type) {
    case SEN6X_TYPE_SEN63C:
        out->co2 = decode_co2_s(w[6]);                   /* int16 ppm */
        break;
    case SEN6X_TYPE_SEN65:
        out->voc_index = decode_idx(w[6]);
        out->nox_index = decode_idx(w[7]);
        break;
    case SEN6X_TYPE_SEN66:
        out->voc_index = decode_idx(w[6]);
        out->nox_index = decode_idx(w[7]);
        out->co2       = decode_co2_u(w[8]);             /* uint16 ppm */
        break;
    case SEN6X_TYPE_SEN68:
        out->voc_index = decode_idx(w[6]);
        out->nox_index = decode_idx(w[7]);
        out->hcho      = decode_hcho(w[8]);              /* uint16 ppb */
        break;
    case SEN6X_TYPE_SEN69C:
        out->voc_index = decode_idx(w[6]);
        out->nox_index = decode_idx(w[7]);
        out->hcho      = decode_hcho(w[8]);              /* uint16 ppb */
        out->co2       = decode_co2_s(w[9]);             /* int16 ppm */
        break;
    default:
        break;
    }
    return SEN6X_OK;
}

sen6x_type_t sen6x_get_type(void)
{
    return g_type;
}

const char *sen6x_type_name(sen6x_type_t t)
{
    switch (t) {
    case SEN6X_TYPE_SEN62:  return "SEN62";
    case SEN6X_TYPE_SEN63C: return "SEN63C";
    case SEN6X_TYPE_SEN65:  return "SEN65";
    case SEN6X_TYPE_SEN66:  return "SEN66";
    case SEN6X_TYPE_SEN68:  return "SEN68";
    case SEN6X_TYPE_SEN69C: return "SEN69C";
    default:                return "UNKNOWN";
    }
}

/* --------------------------------------------------------------------- */
/* Optional commands                                                     */
/* --------------------------------------------------------------------- */

int sen6x_start_fan_cleaning(void)
{
    return send_command(CMD_START_FAN_CLEANING, EXEC_FAN_CLEANING_MS);
}

int sen6x_read_device_status(sen6x_device_status_t *out)
{
    uint16_t w[2];
    uint32_t raw;
    sen6x_status_t st;

    if (out == NULL) {
        return SEN6X_ERR_ARG;
    }
    st = query_words(CMD_READ_DEVICE_STATUS, EXEC_GENERIC_MS, w, 2);
    if (st != SEN6X_OK) {
        return st;
    }
    raw = ((uint32_t)w[0] << 16) | (uint32_t)w[1];

    out->raw               = raw;
    out->fan_speed_warning = (raw & SEN6X_STATUS_SPEED_WARNING) != 0u;
    out->pm_error          = (raw & SEN6X_STATUS_PM_ERROR)      != 0u;
    out->rht_error         = (raw & SEN6X_STATUS_RHT_ERROR)     != 0u;
    out->fan_error         = (raw & SEN6X_STATUS_FAN_ERROR)     != 0u;
    /* Gas error applies to gas-capable variants (SEN65/66/68/69C). */
    out->gas_error         = (g_type == SEN6X_TYPE_SEN65 ||
                              g_type == SEN6X_TYPE_SEN66 ||
                              g_type == SEN6X_TYPE_SEN68 ||
                              g_type == SEN6X_TYPE_SEN69C) &&
                             ((raw & SEN6X_STATUS_GAS_ERROR) != 0u);
    /* HCHO error applies to formaldehyde-capable variants (SEN68/69C). */
    out->hcho_error        = (g_type == SEN6X_TYPE_SEN68 ||
                              g_type == SEN6X_TYPE_SEN69C) &&
                             ((raw & SEN6X_STATUS_HCHO_ERROR) != 0u);
    /* CO2 error uses CO2-1 (bit 12) on SEN63C/69C, CO2-2 (bit 9) on SEN66. */
    if (g_type == SEN6X_TYPE_SEN63C || g_type == SEN6X_TYPE_SEN69C) {
        out->co2_error = (raw & SEN6X_STATUS_CO2_1_ERROR) != 0u;
    } else if (g_type == SEN6X_TYPE_SEN66) {
        out->co2_error = (raw & SEN6X_STATUS_CO2_2_ERROR) != 0u;
    } else {
        out->co2_error = false;
    }
    return SEN6X_OK;
}

int sen6x_set_temperature_offset(float offset_deg_c, float slope,
                                 uint16_t time_const_s, uint16_t slot)
{
    uint16_t args[4];
    /* Scaling per datasheet Table 40: offset x200, slope x10000. */
    args[0] = (uint16_t)(int16_t)lroundf(offset_deg_c * 200.0f);
    args[1] = (uint16_t)(int16_t)lroundf(slope * 10000.0f);
    args[2] = time_const_s;
    args[3] = slot;
    return send_command_args(CMD_SET_TEMP_OFFSET, args, 4, EXEC_GENERIC_MS);
}

/* CO2 helpers are only valid on CO2-capable variants (SEN63C/66/69C). */
static sen6x_status_t require_co2(void)
{
    if (g_type == SEN6X_TYPE_SEN63C ||
        g_type == SEN6X_TYPE_SEN66 ||
        g_type == SEN6X_TYPE_SEN69C) {
        return SEN6X_OK;
    }
    return SEN6X_ERR_TYPE;
}

int sen6x_forced_co2_recalibration(uint16_t target_ppm, int32_t *correction_ppm)
{
    uint16_t arg = target_ppm;
    uint16_t result;
    sen6x_status_t st;

    st = require_co2();
    if (st != SEN6X_OK) {
        return st;
    }
    /* Send & fetch: write target, wait 500 ms, read one correction word. */
    st = send_command_args(CMD_FORCED_CO2_RECAL, &arg, 1, EXEC_FORCED_RECAL_MS);
    if (st != SEN6X_OK) {
        return st;
    }
    st = read_words(&result, 1);
    if (st != SEN6X_OK) {
        return st;
    }
    if (result == SENTINEL_U16) {
        return SEN6X_ERR_CALIB;   /* recalibration failed */
    }
    if (correction_ppm != NULL) {
        *correction_ppm = (int32_t)result - 0x8000;
    }
    return SEN6X_OK;
}

int sen6x_get_co2_auto_self_cal(bool *enabled)
{
    uint16_t word;
    sen6x_status_t st;

    if (enabled == NULL) {
        return SEN6X_ERR_ARG;
    }
    st = require_co2();
    if (st != SEN6X_OK) {
        return st;
    }
    st = query_words(CMD_CO2_AUTO_SELF_CAL, EXEC_GENERIC_MS, &word, 1);
    if (st != SEN6X_OK) {
        return st;
    }
    /* Word = [byte0 padding 0x00][byte1 status flag]. */
    *enabled = (word & 0xFFu) != 0u;
    return SEN6X_OK;
}

int sen6x_set_co2_auto_self_cal(bool enable)
{
    uint16_t arg;
    sen6x_status_t st = require_co2();
    if (st != SEN6X_OK) {
        return st;
    }
    /* Padding byte 0x00 in MSB, status flag in LSB. */
    arg = (uint16_t)(enable ? 0x0001u : 0x0000u);
    return send_command_args(CMD_CO2_AUTO_SELF_CAL, &arg, 1, EXEC_GENERIC_MS);
}
