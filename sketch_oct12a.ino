// 6x MPU6050 -> Quaternion-driven rotating cubes on LilyGo T-Display
// Bus 1: MultiSDA_I2C (3x SDA, shared SCL)
// Bus 2: 3x independent SoftWire buses (software I2C)

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "MultiSDA_I2C.h"
#include <SoftWire.h>   // <-- Add this library (Software I2C)
#include <math.h>

// ---- Pin definitions ----
// Bus 1 (MultiSDA_I2C)
#define SCL1_PIN 22
#define SDA1_PIN 21
#define SDA2_PIN 32
#define SDA3_PIN 27

// Bus 2 (SoftWire instances)
#define SDA4_PIN 25
#define SDA5_PIN 26
#define SDA6_PIN 33
#define SCL2_PIN 17  // shared or separate depending on your wiring

#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// ---- I2C and Display setup ----
MultiSDA_I2C myI2C1(SCL1_PIN, 1, 1);
SoftWire sw4(SDA4_PIN, SCL2_PIN);
SoftWire sw5(SDA5_PIN, SCL2_PIN);
SoftWire sw6(SDA6_PIN, SCL2_PIN);

TFT_eSPI tft = TFT_eSPI();

// ---- math structs ----
struct Quaternion { float w, x, y, z; };
struct Vec3 { float x, y, z; };

// ---- constants ----
const float GYRO_SCALE = 131.0f;
const float ALPHA = 0.98f;
const float DEG2RAD = PI / 180.0f;

// ---- runtime state ----
Quaternion qs[6] = {
  {1,0,0,0},{1,0,0,0},{1,0,0,0},
  {1,0,0,0},{1,0,0,0},{1,0,0,0}
};
unsigned long lastMicros = 0;

// ---- pin arrays ----
const uint8_t sdaPins1[] = {SDA1_PIN, SDA2_PIN, SDA3_PIN};
const int pinsCnt1 = sizeof(sdaPins1)/sizeof(sdaPins1[0]);

// -------- I2C helpers ----------
bool mpuWrite(MultiSDA_I2C &bus, uint8_t sdaPin, uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {reg, data};
  return bus.writeBytes(sdaPin, MPU_ADDR, buf, 2, true);
}

bool mpuRead(MultiSDA_I2C &bus, uint8_t sdaPin, uint8_t reg, uint8_t *buf, size_t len) {
  if (!bus.writeBytes(sdaPin, MPU_ADDR, &reg, 1, true)) return false;
  delayMicroseconds(10);
  return bus.readBytes(sdaPin, MPU_ADDR, buf, len, true);
}

// --- Overloads for SoftWire ---
bool mpuWrite(SoftWire &sw, uint8_t reg, uint8_t data) {
  sw.beginTransmission(MPU_ADDR);
  sw.write(reg);
  sw.write(data);
  return (sw.endTransmission() == 0);
}

bool mpuRead(SoftWire &sw, uint8_t reg, uint8_t *buf, size_t len) {
  sw.beginTransmission(MPU_ADDR);
  sw.write(reg);
  if (sw.endTransmission(false) != 0) return false;
  if (sw.requestFrom(MPU_ADDR, (uint8_t)len) != len) return false;
  for (size_t i=0;i<len;i++) buf[i]=sw.read();
  return true;
}

// ---- math and rendering (unchanged) ----
// ... [KEEP your quaternion and drawCube functions as-is] ...

// -------- setup --------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize MultiSDA bus
  myI2C1.begin(sdaPins1, pinsCnt1);

  // Initialize SoftWire instances
  sw4.begin();
  sw5.begin();
  sw6.begin();

  // Initialize display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawCentreString("Hybrid I2C: 6x MPU6050", 120, 6, 2);

  // Wake sensors on bus 1
  for (uint8_t sda : sdaPins1) {
    if (mpuWrite(myI2C1, sda, PWR_MGMT_1, 0x00))
      Serial.printf("MPU (bus1) SDA %d OK\n", sda);
    else
      Serial.printf("MPU (bus1) SDA %d FAIL\n", sda);
  }

  // Wake sensors on individual SoftWire buses
  if (mpuWrite(sw4, PWR_MGMT_1, 0x00)) Serial.println("MPU sw4 OK"); else Serial.println("MPU sw4 FAIL");
  if (mpuWrite(sw5, PWR_MGMT_1, 0x00)) Serial.println("MPU sw5 OK"); else Serial.println("MPU sw5 FAIL");
  if (mpuWrite(sw6, PWR_MGMT_1, 0x00)) Serial.println("MPU sw6 OK"); else Serial.println("MPU sw6 FAIL");

  lastMicros = micros();
}

// -------- main loop --------
void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  tft.fillRect(0, 28, 240, 200, TFT_BLACK);
  uint16_t colors[6] = {TFT_ORANGE, TFT_CYAN, TFT_RED, TFT_YELLOW, TFT_GREEN, TFT_BLUE};

  // --- Bus 1 sensors ---
  for (int i=0; i<pinsCnt1; ++i) {
    uint8_t sda = sdaPins1[i];
    uint8_t buf[14];
    if (!mpuRead(myI2C1, sda, ACCEL_XOUT_H, buf, 14)) continue;
    int16_t ax=(buf[0]<<8)|buf[1], ay=(buf[2]<<8)|buf[3], az=(buf[4]<<8)|buf[5];
    int16_t gx=(buf[8]<<8)|buf[9], gy=(buf[10]<<8)|buf[11], gz=(buf[12]<<8)|buf[13];
    float axf=ax/16384.0f, ayf=ay/16384.0f, azf=az/16384.0f;
    float gxf=gx/GYRO_SCALE, gyf=gy/GYRO_SCALE, gzf=gz/GYRO_SCALE;
    Quaternion qGyro = integrateGyro(qs[i], gxf, gyf, gzf, dt);
    Quaternion qAccel = accelToQuat(axf, ayf, azf);
    qs[i] = fuseOrientation(qGyro, qAccel);
    int col = i % 3, row = i / 3;
    drawCube(40 + col*80, 45 + row*45, qs[i], colors[i]);
  }

  // --- SoftWire sensors ---
  SoftWire* buses[3] = {&sw4, &sw5, &sw6};
  for (int i=0; i<3; ++i) {
    uint8_t buf[14];
    if (!mpuRead(*buses[i], ACCEL_XOUT_H, buf, 14)) continue;
    int idx = i + pinsCnt1;
    int16_t ax=(buf[0]<<8)|buf[1], ay=(buf[2]<<8)|buf[3], az=(buf[4]<<8)|buf[5];
    int16_t gx=(buf[8]<<8)|buf[9], gy=(buf[10]<<8)|buf[11], gz=(buf[12]<<8)|buf[13];
    float axf=ax/16384.0f, ayf=ay/16384.0f, azf=az/16384.0f;
    float gxf=gx/GYRO_SCALE, gyf=gy/GYRO_SCALE, gzf=gz/GYRO_SCALE;
    Quaternion qGyro = integrateGyro(qs[idx], gxf, gyf, gzf, dt);
    Quaternion qAccel = accelToQuat(axf, ayf, azf);
    qs[idx] = fuseOrientation(qGyro, qAccel);
    int col = idx % 3, row = idx / 3;
    drawCube(40 + col*80, 45 + row*45, qs[idx], colors[idx]);
  }

  delay(25);
}
