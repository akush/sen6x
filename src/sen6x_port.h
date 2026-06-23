/*
 * sen6x_port.h - Platform transport interface for the SEN6x core.
 *
 * The portable C99 core (sen6x.c) calls only these five functions; one
 * implementation (sen6x_port.cpp) selects Arduino Wire or the ESP-IDF
 * i2c_master driver at compile time. All functions return 0 on success
 * and a negative value on failure (except delay, which cannot fail).
 *
 * Copyright (c) 2026 Abhinav (abhinav@ipsator.com)
 * SPDX-License-Identifier: MIT
 */
#ifndef SEN6X_PORT_H
#define SEN6X_PORT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mirror of the bus settings the core forwards from sen6x_config_t. */
typedef struct {
    int      sda;
    int      scl;
    uint32_t freq_hz;
    int      i2c_port;
} sen6x_port_config_t;

/* Initialize the I2C controller / add the SEN6x device. Idempotent: a
 * second call re-initializes with the new configuration. */
int sen6x_port_init(const sen6x_port_config_t *cfg);

/* Release any resources acquired by sen6x_port_init(). */
int sen6x_port_deinit(void);

/* Write exactly len bytes to the 7-bit address addr, issuing a STOP at
 * the end (Sensirion requires STOP + execution-time wait before a read).
 * Returns 0 on ACK of all bytes, negative otherwise. */
int sen6x_port_write(uint8_t addr, const uint8_t *data, size_t len);

/* Read exactly len bytes from the 7-bit address addr. The caller has
 * already waited the command execution time. Returns 0 on success. */
int sen6x_port_read(uint8_t addr, uint8_t *data, size_t len);

/* Block for at least ms milliseconds. */
void sen6x_port_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* SEN6X_PORT_H */
