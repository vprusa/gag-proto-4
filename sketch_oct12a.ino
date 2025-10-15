// 6x MPU6050 -> Quaternion-driven rotating cubes on LilyGo T-Display
// Requires: MultiSDA_I2C (bit-banged multi-SDA I2C library) and TFT_eSPI (configured for TTGO T-Display)

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "MultiSDA_I2C.h"
#include <math.h>

// ---- Pin definitions ----
#define SCL_PIN 22
#define SDA1_PIN 17
#define SDA2_PIN 21
#define SDA3_PIN 27
#define SDA4_PIN 26
#define SDA5_PIN 25
#define SDA6_PIN 32
#define MPU_ADDR 0x68

#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// ---- I2C and Display setup ----
MultiSDA_I2C myI2C(SCL_PIN, 1, 1);
TFT_eSPI tft = TFT_eSPI();

// ---- math structs ----
struct Quaternion { float w, x, y, z; };
struct Vec3 { float x, y, z; };

// ---- constants ----
const float GYRO_SCALE = 131.0f;   // LSB per deg/s for ±250dps
const float ALPHA = 0.98f;
const float DEG2RAD = PI / 180.0f;

// ---- runtime state ----
Quaternion qs[6] = {
  {1,0,0,0}, {1,0,0,0}, {1,0,0,0},
  {1,0,0,0}, {1,0,0,0}, {1,0,0,0}
};
unsigned long lastMicros = 0;

// ---- pin setup ----
const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN, SDA3_PIN, SDA4_PIN, SDA5_PIN, SDA6_PIN};
// const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN, SDA3_PIN, SDA4_PIN, SDA5_PIN};
// const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN, SDA3_PIN};
// const int pinsCnt = sizeof(sdaPins)/sizeof(sdaPins[0]);
const int pinsCnt = 6;

// -------- I2C helpers ----------
bool mpuWrite(uint8_t sdaPin, uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {reg, data};
  return myI2C.writeBytes(sdaPin, MPU_ADDR, buf, 2, true);
}

bool mpuRead(uint8_t sdaPin, uint8_t reg, uint8_t *buf, size_t len) {
  if (!myI2C.writeBytes(sdaPin, MPU_ADDR, &reg, 1, true)) return false;
  delayMicroseconds(10);
  return myI2C.readBytes(sdaPin, MPU_ADDR, buf, len, true);
}

// -------- quaternion math ----------
Quaternion quatMultiply(const Quaternion &a, const Quaternion &b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

Quaternion quatScale(const Quaternion &q, float s) {
  return {q.w*s, q.x*s, q.y*s, q.z*s};
}

Quaternion quatAdd(const Quaternion &a, const Quaternion &b) {
  return {a.w+b.w, a.x+b.x, a.y+b.y, a.z+b.z};
}

Quaternion quatNormalize(const Quaternion &q) {
  float n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-9f) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

Quaternion integrateGyro(const Quaternion &q, float gx, float gy, float gz, float dt) {
  gx *= DEG2RAD; gy *= DEG2RAD; gz *= DEG2RAD;
  Quaternion omega = {0,gx,gy,gz};
  Quaternion q_omega = quatMultiply(q, omega);
  Quaternion dqdt = quatScale(q_omega, 0.5f);
  return quatNormalize(quatAdd(q, quatScale(dqdt, dt)));
}

Quaternion quatFromEuler(float roll, float pitch, float yaw) {
  float cr = cos(roll*0.5f), sr = sin(roll*0.5f);
  float cp = cos(pitch*0.5f), sp = sin(pitch*0.5f);
  float cy = cos(yaw*0.5f), sy = sin(yaw*0.5f);
  return quatNormalize({
    cr*cp*cy + sr*sp*sy,
    sr*cp*cy - cr*sp*sy,
    cr*sp*cy + sr*cp*sy,
    cr*cp*sy - sr*sp*cy
  });
}

Quaternion accelToQuat(float ax, float ay, float az) {
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay*ay + az*az));
  return quatFromEuler(roll, pitch, 0);
}

Quaternion fuseOrientation(const Quaternion &qGyro, const Quaternion &qAccel) {
  Quaternion q = {
    ALPHA*qGyro.w + (1-ALPHA)*qAccel.w,
    ALPHA*qGyro.x + (1-ALPHA)*qAccel.x,
    ALPHA*qGyro.y + (1-ALPHA)*qAccel.y,
    ALPHA*qGyro.z + (1-ALPHA)*qAccel.z
  };
  return quatNormalize(q);
}

Vec3 rotateVec(const Vec3 &v, const Quaternion &q) {
  Quaternion vq = {0,v.x,v.y,v.z};
  Quaternion qc = {q.w,-q.x,-q.y,-q.z};
  Quaternion tmp = quatMultiply(q,vq);
  Quaternion res = quatMultiply(tmp,qc);
  return {res.x,res.y,res.z};
}

// -------- cube rendering ----------
void drawCube(int x0, int y0, const Quaternion &q, uint16_t color) {
  static const Vec3 verts[8] = {
    {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
    {-1,-1, 1},{1,-1, 1},{1,1, 1},{-1,1, 1}
  };
  static const uint8_t edges[12][2] = {
    {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
  };

  const float scale = 10.0f;
  Vec3 proj[8];
  for (int i=0;i<8;++i){
    Vec3 r = rotateVec(verts[i], q);
    proj[i] = {x0 + r.x*scale, y0 - r.y*scale, r.z};
  }
  for (int e=0;e<12;++e){
    int a = edges[e][0], b = edges[e][1];
    tft.drawLine((int)proj[a].x,(int)proj[a].y,(int)proj[b].x,(int)proj[b].y,color);
  }
}

// -------- setup --------
void setup() {
  Serial.begin(115200);
  delay(200);
  myI2C.begin(sdaPins, pinsCnt);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawCentreString("6x MPU6050 Cubes", 120, 6, 2);

  for (uint8_t sda : sdaPins) {
    if (mpuWrite(sda, PWR_MGMT_1, 0x00))
      Serial.printf("MPU on SDA %d woke up\n", sda);
    else
      Serial.printf("MPU on SDA %d failed to wake\n", sda);
  }

  lastMicros = micros();
}

// -------- main loop --------
void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  tft.fillRect(0, 28, 240, 200, TFT_BLACK);

  for (int i=0; i<pinsCnt; ++i) {
    uint8_t sda = sdaPins[i];
    uint8_t buf[14];
    if (!mpuRead(sda, ACCEL_XOUT_H, buf, 14)) {
      continue;
    };

    int16_t ax = (buf[0]<<8)|buf[1];
    int16_t ay = (buf[2]<<8)|buf[3];
    int16_t az = (buf[4]<<8)|buf[5];
    int16_t gx = (buf[8]<<8)|buf[9];
    int16_t gy = (buf[10]<<8)|buf[11];
    int16_t gz = (buf[12]<<8)|buf[13];

    float axf=ax/16384.0f, ayf=ay/16384.0f, azf=az/16384.0f;
    float gxf=gx/GYRO_SCALE, gyf=gy/GYRO_SCALE, gzf=gz/GYRO_SCALE;

    Quaternion qGyro = integrateGyro(qs[i], gxf, gyf, gzf, dt);
    Quaternion qAccel = accelToQuat(axf, ayf, azf);
    qs[i] = fuseOrientation(qGyro, qAccel);

    // --- Visualization layout (6 cubes = 2 rows of 3) ---
    int row = i / 3;
    int col = i % 3;
    int xCenter = 40 + col * 80;
    int yCenter = 45 + row * 45;

    uint16_t colors[6] = {TFT_ORANGE, TFT_CYAN, TFT_RED, TFT_YELLOW, TFT_GREEN, TFT_BLUE};
    drawCube(xCenter, yCenter, qs[i], colors[i % 6]);
  }

  delay(25);
}
