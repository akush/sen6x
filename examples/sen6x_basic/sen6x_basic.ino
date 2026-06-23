/*
 * sen6x_basic.ino - Minimal SEN6x example for the Arduino IDE (ESP32).
 *
 * Wiring (3.3 V ONLY):
 *   SEN6x VDD  -> 3V3
 *   SEN6x GND  -> GND
 *   SEN6x SDA  -> GPIO 8  (with external ~10k pull-up to 3V3)
 *   SEN6x SCL  -> GPIO 9  (with external ~10k pull-up to 3V3)
 *   SEN6x SEL  -> GND     (selects I2C interface)
 *
 * Adjust SDA_PIN / SCL_PIN for your board (the values below suit many
 * ESP32-C3 / C6 dev boards).
 */
#include <sen6x.h>

static const int SDA_PIN = 8;
static const int SCL_PIN = 9;

void setup() {
  Serial.begin(115200);
  delay(100);

  sen6x_config_t cfg;
  cfg.sda      = SDA_PIN;
  cfg.scl      = SCL_PIN;
  cfg.freq_hz  = 100000;   // SEN6x max is 100 kHz
  cfg.i2c_port = 0;        // ignored by the Arduino/Wire port

  if (sen6x_init(&cfg) != SEN6X_OK) {
    Serial.println("sen6x_init failed (check wiring / pull-ups)");
    while (true) { delay(1000); }
  }

  // First sanity check: who is on the bus?
  sen6x_type_t type = sen6x_detect();
  Serial.print("Detected sensor: ");
  Serial.println(sen6x_type_name(type));
  if (type == SEN6X_TYPE_UNKNOWN) {
    Serial.println("Could not identify the sensor; halting.");
    while (true) { delay(1000); }
  }

  if (sen6x_start() != SEN6X_OK) {
    Serial.println("sen6x_start failed");
    while (true) { delay(1000); }
  }
  Serial.println("Measurement started (~1.1 s to first data).");
}

static void printValue(const char *label, float v, const char *unit) {
  Serial.print(label);
  if (isnan(v)) {
    Serial.println("n/a");
  } else {
    Serial.print(v);
    Serial.print(' ');
    Serial.println(unit);
  }
}

void loop() {
  if (!sen6x_data_ready()) {
    delay(200);
    return;
  }

  sen6x_data_t d;
  if (sen6x_read(&d) != SEN6X_OK) {
    Serial.println("sen6x_read failed");
    delay(1000);
    return;
  }

  Serial.println("---");
  printValue("PM1.0:  ", d.pm1,         "ug/m3");
  printValue("PM2.5:  ", d.pm2_5,       "ug/m3");
  printValue("PM4.0:  ", d.pm4,         "ug/m3");
  printValue("PM10:   ", d.pm10,        "ug/m3");
  printValue("RH:     ", d.rh,          "%");
  printValue("T:      ", d.temperature, "degC");
  printValue("VOC:    ", d.voc_index,   "idx");
  printValue("NOx:    ", d.nox_index,   "idx");
  printValue("HCHO:   ", d.hcho,        "ppb");
  printValue("CO2:    ", d.co2,         "ppm");

  delay(1000);
}
