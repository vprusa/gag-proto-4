#include <Arduino.h>
#include <TFT_eSPI.h>
#include "MultiSDA_I2C.h"
#include "I2Cdev.h"

// === Pin Config for ESP32 LilyGo T-Display ===
#define SCL_PIN 22
#define SDA1_PIN 21   // GY-25 #1
#define SDA2_PIN 17   // GY-25 #2
#define GY25_ADDR 0x50

// === MultiSDA_I2C Bus Setup ===
MultiSDA_I2C i2cBus(SCL_PIN, 5, 20);  // delayUs=5 (~100kHz), timeout=20ms
MultiSDA_I2C *I2Cbus = &i2cBus;
uint8_t I2C_sda_pin = SDA1_PIN;       // active SDA line for I2Cdev backend

// === Display ===
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite cube1 = TFT_eSprite(&tft);
TFT_eSprite cube2 = TFT_eSprite(&tft);

float yaw1 = 0, pitch1 = 0, roll1 = 0;
float yaw2 = 0, pitch2 = 0, roll2 = 0;

// === GY-25 Communication (via MultiSDA_I2C) ===
bool readGY25(uint8_t sdaPin, float &yaw, float &pitch, float &roll) {
  uint8_t buf[8];
  I2C_sda_pin = sdaPin;

  if (I2Cbus->readBytes(sdaPin, GY25_ADDR, buf, 8)) {
    if (buf[0] == 0xAA && buf[1] == 0x55) {
      yaw   = ((buf[2] << 8) | buf[3]) / 100.0f;
      pitch = ((buf[4] << 8) | buf[5]) / 100.0f;
      roll  = ((buf[6] << 8) | buf[7]) / 100.0f;
      return true;
    }
  }
  return false;
}

// === Simple Cube Drawing ===
struct Point3D { float x, y, z; };
struct Point2D { int16_t x, y; };
Point2D project3D(const Point3D &p, float yaw, float pitch, float roll, int16_t cx, int16_t cy, float scale);

const Point3D cubeVertices[8] = {
  {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
  {-1, -1,  1}, {1, -1,  1}, {1, 1,  1}, {-1, 1,  1}
};
const uint8_t cubeEdges[12][2] = {
  {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
};

Point2D project3D(const Point3D &p, float yaw, float pitch, float roll, int16_t cx, int16_t cy, float scale) {
  float radYaw = radians(yaw);
  float radPitch = radians(pitch);
  float radRoll = radians(roll);

  float sinY = sin(radYaw), cosY = cos(radYaw);
  float sinP = sin(radPitch), cosP = cos(radPitch);
  float sinR = sin(radRoll), cosR = cos(radRoll);

  float x = p.x * cosY * cosR + p.y * (cosY * sinR * sinP - sinY * cosP) + p.z * (cosY * sinR * cosP + sinY * sinP);
  float y = p.x * sinY * cosR + p.y * (sinY * sinR * sinP + cosY * cosP) + p.z * (sinY * sinR * cosP - cosY * sinP);
  float z = -p.x * sinR + p.y * cosR * sinP + p.z * cosR * cosP;

  int16_t px = cx + (int16_t)(x * scale);
  int16_t py = cy - (int16_t)(y * scale);
  return {px, py};
}

void drawCube(TFT_eSprite &spr, float yaw, float pitch, float roll, uint16_t color) {
  spr.fillSprite(TFT_BLACK);
  Point2D proj[8];
  for (int i = 0; i < 8; i++)
    proj[i] = project3D(cubeVertices[i], yaw, pitch, roll, spr.width() / 2, spr.height() / 2, 40);

  for (int i = 0; i < 12; i++)
    spr.drawLine(proj[cubeEdges[i][0]].x, proj[cubeEdges[i][0]].y,
                 proj[cubeEdges[i][1]].x, proj[cubeEdges[i][1]].y, color);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing MultiSDA_I2C for dual GY-25s...");
  uint8_t sdaPins[] = { SDA1_PIN, SDA2_PIN };
  i2cBus.begin(sdaPins, 2);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  cube1.createSprite(120, 120);
  cube2.createSprite(120, 120);

  cube1.setSwapBytes(true);
  cube2.setSwapBytes(true);

  Serial.println("Setup complete!");
}

// === Main Loop ===
void loop() {
  bool ok1 = readGY25(SDA1_PIN, yaw1, pitch1, roll1);
  bool ok2 = readGY25(SDA2_PIN, yaw2, pitch2, roll2);

  drawCube(cube1, yaw1, pitch1, roll1, ok1 ? TFT_GREEN : TFT_RED);
  drawCube(cube2, yaw2, pitch2, roll2, ok2 ? TFT_CYAN : TFT_RED);

  cube1.pushSprite(0, 60);
  cube2.pushSprite(120, 60);

  delay(100);
}
