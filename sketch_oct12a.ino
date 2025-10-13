#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "MultiSDA_I2C.h"

// Shared clock pin
#define SCL_PIN 22
// Two SDA lines
#define SDA1_PIN 21
#define SDA2_PIN 17
#define MPU_ADDR 0x68

// MPU6050 registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

MultiSDA_I2C myI2C(SCL_PIN);
TFT_eSPI tft = TFT_eSPI(135, 240); // width, height

// Helper to write a byte
bool mpuWrite(uint8_t sdaPin, uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {reg, data};
  return myI2C.writeBytes(sdaPin, MPU_ADDR, buf, 2, true);
}

// Helper to read N bytes
bool mpuRead(uint8_t sdaPin, uint8_t reg, uint8_t *buf, size_t len) {
  if (!myI2C.writeBytes(sdaPin, MPU_ADDR, &reg, 1, true)) return false;
  delayMicroseconds(10);
  return myI2C.readBytes(sdaPin, MPU_ADDR, buf, len, true);
}

// Compute pitch/roll from accelerometer data
void computeAngles(int16_t ax, int16_t ay, int16_t az, float &pitch, float &roll) {
  float axf = ax / 16384.0f;
  float ayf = ay / 16384.0f;
  float azf = az / 16384.0f;
  pitch = atan2(axf, sqrt(ayf * ayf + azf * azf)) * 180.0 / PI;
  roll  = atan2(ayf, sqrt(axf * axf + azf * azf)) * 180.0 / PI;
}

void drawSensorIndicator(int x, int y, float pitch, float roll, const char *label, uint16_t color) {
  const int boxSize = 60;
  tft.drawRect(x - boxSize/2, y - boxSize/2, boxSize, boxSize, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString(label, x, y + boxSize/2 + 10, 1);

  // Map pitch/roll (-90..90) to offset (-20..20)
  int dx = map(roll, -90, 90, -20, 20);
  int dy = map(pitch, -90, 90, -20, 20);

  // Draw circle indicating orientation
  tft.fillCircle(x + dx, y + dy, 5, color);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nMPU6050 Dual + Display Example");

  const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};
  myI2C.begin(sdaPins, 2);

  // Initialize display
  tft.init();
  tft.setRotation(1); // landscape
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawCentreString("MPU6050 Dual Visualizer", 120, 10, 2);

  // Initialize sensors
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t sda = sdaPins[i];
    if (mpuWrite(sda, PWR_MGMT_1, 0x00)) {
      Serial.printf("Sensor %d initialized (SDA=%d)\n", i + 1, sda);
    } else {
      Serial.printf("Sensor %d init failed!\n", i + 1);
    }
  }
}

void loop() {
  const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};
  float pitch[2], roll[2];

  for (uint8_t i = 0; i < 2; i++) {
    uint8_t sda = sdaPins[i];
    uint8_t buf[14];
    if (mpuRead(sda, ACCEL_XOUT_H, buf, 14)) {
      int16_t ax = (buf[0] << 8) | buf[1];
      int16_t ay = (buf[2] << 8) | buf[3];
      int16_t az = (buf[4] << 8) | buf[5];
      computeAngles(ax, ay, az, pitch[i], roll[i]);
      Serial.printf("MPU%d: Pitch=%6.2f  Roll=%6.2f\n", i + 1, pitch[i], roll[i]);
    } else {
      Serial.printf("MPU%d read error\n", i + 1);
      pitch[i] = roll[i] = 0;
    }
  }

  // Clear and redraw visualization
  tft.fillRect(0, 30, 240, 100, TFT_BLACK);
  drawSensorIndicator(80, 80, pitch[0], roll[0], "Sensor 1", TFT_RED);
  drawSensorIndicator(180, 80, pitch[1], roll[1], "Sensor 2", TFT_CYAN);

  delay(200);
}
