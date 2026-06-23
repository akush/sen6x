# SEN6x

A single-source C/C++ driver for the **Sensirion SEN62, SEN63C, SEN65, SEN66,
SEN68 and SEN69C** environmental sensors over I2C. It **auto-detects** the
connected variant and reads only the signals that variant supports. The same
source tree builds in **both** the Arduino IDE (ESP32) and **ESP-IDF (≥ 5.2)**.
Target MCUs: ESP32-C3 / C6.

| Variant | Signals (beyond PM1.0 / PM2.5 / PM4.0 / PM10, RH, T) |
|---------|------------------------------------------------------|
| SEN62   | — |
| SEN63C  | CO₂ |
| SEN65   | VOC index, NOx index |
| SEN66   | VOC index, NOx index, CO₂ |
| SEN68   | VOC index, NOx index, HCHO |
| SEN69C  | VOC index, NOx index, HCHO, CO₂ |

## Design

The protocol is deliberately separated from the transport:

| File | Role |
|------|------|
| [`src/sen6x.h`](src/sen6x.h)        | Public C API (`extern "C"`), enums, structs |
| [`src/sen6x.c`](src/sen6x.c)        | Portable **C99** core: CRC-8, framing, CRC-checked parsing, conditional per-variant decode, auto-detect. No `Arduino.h`, no ESP-IDF headers — calls the port only. |
| [`src/sen6x_port.h`](src/sen6x_port.h)   | Port interface: `init` / `write` / `read` / `delay_ms` (`extern "C"`) |
| [`src/sen6x_port.cpp`](src/sen6x_port.cpp) | One file, compile-time switch: `#if defined(ARDUINO)` → Wire, `#elif defined(ESP_PLATFORM)` → `driver/i2c_master` (new API). `ARDUINO` is checked first because arduino-as-an-IDF-component defines both. |

The core never touches the bus directly, so it compiles standalone as C99:

```sh
gcc -std=c99 -Wall -Wextra -c src/sen6x.c
```

No dynamic allocation is used anywhere.

## Wiring

> **3.3 V only.** The SEN6x is **not** 5 V tolerant on VDD. Powering it from 5 V
> can damage it.

- **VDD → 3V3**, **GND → GND**
- **SDA** and **SCL** are open-drain. Add **external ~10 kΩ pull-ups to 3V3** on
  both lines. (The ESP-IDF port also enables the internal pull-ups, but those
  are weak — external resistors are still required for reliable 100 kHz.)
- **Max I2C speed: 100 kHz** (standard mode). The sensor does not use clock
  stretching; it NACKs while busy, and the driver waits each command's
  execution time before reading.
- Tie the sensor's interface-select pin to **GND** to select I2C.
- All variants share I2C address **0x6B** — the variant is identified by
  *Get Product Name*, not by address.

Example pin map (ESP32-C3 / C6, adjust for your board): `SDA = GPIO 8`,
`SCL = GPIO 9`.

### Schematic

The two ~10 kΩ resistors pull SDA and SCL up to the same 3V3 rail that powers
the sensor. The left column is the ESP32; the right column is the SEN6x. A
junction **dot (`●`)** means the wires connect; the `┼` where the SCL pull-up
crosses the SDA line is **not** a connection (no dot).

```
   ESP32-C3/C6                                                SEN6x (0x6B)

   3V3  ●─────●───────────●──────────────────────────────●  VDD
              │           │
            ┌─┴─┐       ┌─┴─┐
            │10k│       │10k│      (pull-ups to 3V3)
            └─┬─┘       └─┬─┘
              │           │
  GPIO8 ●─────●───────────┼──────────────────────────────●  SDA
  GPIO9 ●─────────────────●──────────────────────────────●  SCL
   GND  ●─────────────────────────────────────●──────────●  GND
                                              └──────────●  SEL  (→ GND)
```

Connections, in plain terms:

| Net | ESP32 pin | SEN6x pin | Pull-up |
|-----|-----------|-----------|---------|
| 3V3 | 3V3       | VDD       | — (also feeds both resistors) |
| SDA | GPIO 8    | SDA       | 10 kΩ to 3V3 |
| SCL | GPIO 9    | SCL       | 10 kΩ to 3V3 |
| GND | GND       | GND **and** SEL | — |

(If several I²C devices share the bus, fit **one** pair of pull-ups for the
whole bus, not one pair per device.)

## Install & build — Arduino IDE (ESP32)

1. Download this repository as a ZIP (**Code → Download ZIP**, or zip the
   folder so that `library.properties` sits at the top level of the archive).
2. In the Arduino IDE: **Sketch → Include Library → Add .ZIP Library…** and
   select the ZIP.
3. Open **File → Examples → SEN6x → sen6x_basic**.
4. Select your ESP32-C3 / C6 board and the correct port, then upload.

The Arduino build compiles `sen6x.c` and `sen6x_port.cpp`; the port resolves to
the Wire backend because `ARDUINO` is defined.

## Install & build — ESP-IDF (≥ 5.2)

Add the driver as a component:

```sh
# from your IDF project root
mkdir -p components
git clone https://github.com/abhinav/sen6x components/sen6x
# (or: git submodule add … components/sen6x, or copy the folder in)
```

The [`CMakeLists.txt`](CMakeLists.txt) registers the component
(`idf_component_register … REQUIRES driver`) and ESP-IDF discovers it
automatically. In your `main` component, depend on it:

```cmake
idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES sen6x)
```

Then:

```sh
idf.py set-target esp32c6   # or esp32c3
idf.py build flash monitor
```

A complete buildable project is in [`examples/esp-idf/`](examples/esp-idf/).
The port resolves to the `driver/i2c_master` backend because `ESP_PLATFORM` is
defined (and `ARDUINO` is not).

## API

```c
int          sen6x_init(const sen6x_config_t *cfg);  // bus up + soft reset
sen6x_type_t sen6x_detect(void);                     // Get Product Name match
int          sen6x_start(void);
int          sen6x_stop(void);
bool         sen6x_data_ready(void);
int          sen6x_read(sen6x_data_t *out);          // opcode/word-count by type
sen6x_type_t sen6x_get_type(void);
const char  *sen6x_type_name(sen6x_type_t t);
```

`sen6x_data_t` carries `pm1, pm2_5, pm4, pm10, rh, temperature, voc_index,
nox_index, hcho, co2` as `float`. **Any signal the variant does not provide —
or that the sensor flags as unknown — is returned as `NAN`** (test with
`isnan()`).

Optional commands (CO₂/gas ones are guarded to the right variants and return
`SEN6X_ERR_TYPE` otherwise):

```c
int sen6x_start_fan_cleaning(void);
int sen6x_read_device_status(sen6x_device_status_t *out);   // decodes bits
int sen6x_set_temperature_offset(float offset_c, float slope,
                                 uint16_t time_const_s, uint16_t slot);
int sen6x_forced_co2_recalibration(uint16_t target_ppm, int32_t *correction_ppm);
int sen6x_get_co2_auto_self_cal(bool *enabled);
int sen6x_set_co2_auto_self_cal(bool enable);
```

Typical flow: `sen6x_init` → `sen6x_detect` → `sen6x_start` → poll
`sen6x_data_ready` → `sen6x_read`.

## Datasheet verification

Implemented and cross-checked against **Sensirion SEN6x datasheet
`PS_DS_SEN6x`, D1 Version 0.92, December 2025**. Mapping of every opcode,
frame and scaling to the section that defines it:

| Item | Value | Datasheet section / table |
|------|-------|---------------------------|
| I2C address (all variants) | `0x6B`, 7-bit | §4.4, Table 25 |
| Max I2C speed | 100 kHz, no clock stretching (NACK when busy) | §4.4, Table 25 |
| Word format | 16-bit, MSB first, + 1 CRC byte; no CRC on command IDs | §4.6, Figures 8–9 |
| CRC-8 | Dallas/Maxim, poly `0x31`, init `0xFF`, no reflect, no final XOR; `CRC(0xBEEF)=0x92` | §4.9, Table 65 |
| Start Continuous Measurement | `0x0021`, 50 ms (~1.1 s to first data) | §4.8 Table 26, §4.8.1 |
| Stop Measurement | `0x0104`, 1400 ms | §4.8 Table 26, §4.8.2 |
| Get Data Ready | `0x0202`, 20 ms; byte0 pad `0x00`, byte1 flag | §4.8.3, Table 29 |
| Read Measured Values SEN62 | `0x04A3`, 20 ms, 6 words | §4.8.4, Table 30 |
| Read Measured Values SEN63C | `0x0471`, 20 ms, 7 words | §4.8.5, Table 31 |
| Read Measured Values SEN65 | `0x0446`, 20 ms, 8 words | §4.8.6, Table 32 |
| Read Measured Values SEN66 | `0x0300`, 20 ms, 9 words | §4.8.7, Table 33 |
| Read Measured Values SEN68 | `0x0467`, 20 ms, 9 words | §4.8.8, Table 34 |
| Read Measured Values SEN69C | `0x04B5`, 20 ms, 10 words | §4.8.9, Table 35 |
| Get Product Name | `0xD014`, 20 ms, up to 32 ASCII chars (48 bytes w/ CRC) | §4.8.16, Table 42 |
| Device Reset | `0xD304`, 1200 ms | §4.8.21, Table 47 |
| Start Fan Cleaning | `0x5607`, 20 ms | §4.8.22, Table 48 |
| Read Device Status | `0xD206`, uint32 flags | §4.8.18, Table 44 |
| Set Temperature Offset | `0x60B2`; offset ×200, slope ×10000, slot 0–4 | §4.8.14, Table 40 |
| Forced CO₂ Recalibration | `0x6707`, 500 ms; correction = ret − `0x8000`, `0xFFFF` = failed | §4.8.31, Table 57 |
| Get/Set CO₂ ASC | `0x6711`, 20 ms; pad byte + bool | §4.8.33–34, Tables 59–60 |
| Device Status bits | SPEED warn 21, CO2-1 12, PM 11, HCHO 10, CO2-2 9, GAS 7, RH&T 6, FAN 4 | §4.3, Figure 7 / Tables 17–24 |

**Read Measured Values — word order, scaling and "unknown" sentinels**
(§4.8.4–4.8.9).

Per-signal type, scaling and "unknown" sentinel:

| Signal | Type | Scaling | Unknown → `NAN` |
|--------|------|---------|-----------------|
| PM1.0 / PM2.5 / PM4.0 / PM10 | uint16 | ÷ 10 → µg/m³ | `0xFFFF` |
| RH        | int16  | ÷ 100 → %  | `0x7FFF` |
| T         | int16  | ÷ 200 → °C | `0x7FFF` |
| VOC index | int16  | ÷ 10       | `0x7FFF` |
| NOx index | int16  | ÷ 10       | `0x7FFF` (also first ~10–11 s) |
| HCHO      | uint16 | ÷ 10 → ppb | `0xFFFF` (also first ~60 s) |
| CO₂ (SEN63C/69C) | int16  | ×1 → ppm | `0x7FFF` (also first ~22–24 s) |
| CO₂ (SEN66)      | uint16 | ×1 → ppm | `0xFFFF` (also first ~5–6 s) |

Word order returned by each variant's *Read Measured Values* (words 0–5 —
PM1.0, PM2.5, PM4.0, PM10, RH, T — are identical across all variants):

| Variant | Words | Tail after word 5 |
|---------|-------|-------------------|
| SEN62   | 6  | — |
| SEN63C  | 7  | CO₂(6) |
| SEN65   | 8  | VOC(6) NOx(7) |
| SEN66   | 9  | VOC(6) NOx(7) CO₂(8) |
| SEN68   | 9  | VOC(6) NOx(7) HCHO(8) |
| SEN69C  | 10 | VOC(6) NOx(7) HCHO(8) CO₂(9) |

Where the prompt's working spec and the datasheet differed, the datasheet
wins. Notable points confirmed/added from the datasheet: the FRC return value
is offset by `0x8000` with `0xFFFF` meaning failure (§4.8.31), and Set
Temperature Offset takes four words — offset (×200), slope (×10000), time
constant, slot (§4.8.14).

## Status — not yet run on hardware

This code was written and **verified against the datasheet (D1 v0.92), but it
has not been run on a physical sensor.** The compile-time, machine-checkable
guarantee is that the C99 core builds warning-free
(`gcc -std=c99 -Wall -Wextra -c src/sen6x.c`) and that the CRC matches the
datasheet's `CRC(0xBEEF) = 0x92` test vector.

**Your first hardware sanity check should be the `sen6x_detect()` /
Get Product Name output** — if the printed name (`SEN62` / `SEN63C` / `SEN65` /
`SEN66` / `SEN68` / `SEN69C`) is correct, the address, CRC, framing and
execution-time waits are all working.

## License

MIT — see [LICENSE](LICENSE). Portions adapted from Sensirion's official
BSD-3-Clause embedded I2C drivers (e.g.
[arduino-i2c-sen66](https://github.com/Sensirion/arduino-i2c-sen66),
[embedded-i2c-sen63c](https://github.com/Sensirion/embedded-i2c-sen63c)), whose
BSD-3-Clause notice is reproduced in `LICENSE`. No code from
`paulvha/SEN6X` (GPL-3.0) is used.
