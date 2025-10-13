// Dual MPU6050 -> Quaternion-driven rotating cubes on LilyGo T-Display
// Requires: MultiSDA_I2C (your bit-banged I2C), TFT_eSPI (configured for TTGO T-Display)

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "MultiSDA_I2C.h"
#include <math.h>

#define SCL_PIN 22
#define SDA1_PIN 21
#define SDA2_PIN 17
#define MPU_ADDR 0x68

#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

MultiSDA_I2C myI2C(SCL_PIN);
TFT_eSPI tft = TFT_eSPI(); // use default setup file for TTGO T-Display

// ---- types ----
struct Quaternion {
  float w, x, y, z;
};

struct Vec3 {
  float x, y, z;
};

// ---- constants ----
const float GYRO_SCALE = 131.0f;   // LSB per deg/s for +/-250dps
const float ALPHA = 0.98f;         // complementary filter weight
const float DEG2RAD = PI / 180.0f;

// ---- state ----
Quaternion q1 = {1, 0, 0, 0};
Quaternion q2 = {1, 0, 0, 0};
unsigned long lastMicros = 0;

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
  Quaternion r;
  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return r;
}

Quaternion quatScale(const Quaternion &q, float s) {
  return {q.w * s, q.x * s, q.y * s, q.z * s};
}

Quaternion quatAdd(const Quaternion &a, const Quaternion &b) {
  return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
}

Quaternion quatNormalize(const Quaternion &q) {
  float n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-9f) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

// integrate angular velocity (deg/s) into quaternion using quaternion derivative
Quaternion integrateGyro(const Quaternion &q, float gx_deg, float gy_deg, float gz_deg, float dt) {
  // convert to radians/s
  float gx = gx_deg * DEG2RAD;
  float gy = gy_deg * DEG2RAD;
  float gz = gz_deg * DEG2RAD;

  // omega quaternion (0, gx, gy, gz)
  Quaternion omega = {0.0f, gx, gy, gz};

  // dq/dt = 0.5 * q * omega
  Quaternion q_omega = quatMultiply(q, omega);
  Quaternion dqdt = quatScale(q_omega, 0.5f);

  // integrate (Euler)
  Quaternion qNew = quatAdd(q, quatScale(dqdt, dt));
  return quatNormalize(qNew);
}

// Build quaternion from roll (x-axis), pitch (y-axis), yaw (z-axis) - radians
Quaternion quatFromEuler(float roll, float pitch, float yaw) {
  float cr = cos(roll * 0.5f);
  float sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f);
  float sp = sin(pitch * 0.5f);
  float cy = cos(yaw * 0.5f);
  float sy = sin(yaw * 0.5f);

  Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return quatNormalize(q);
}

// Convert accel vector to approximate quaternion (roll/pitch, yaw unknown -> 0)
// ax,ay,az expected normalized (g)
Quaternion accelToQuat(float ax, float ay, float az) {
  // roll  = atan2(ay, az)
  // pitch = atan2(-ax, sqrt(ay^2 + az^2))
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay*ay + az*az));
  float yaw = 0.0f;
  return quatFromEuler(roll, pitch, yaw);
}

// Complementary fuse (simple linear blend on quaternion components then renormalize)
Quaternion fuseOrientation(const Quaternion &qGyro, const Quaternion &qAccel) {
  Quaternion q;
  q.w = ALPHA * qGyro.w + (1.0f - ALPHA) * qAccel.w;
  q.x = ALPHA * qGyro.x + (1.0f - ALPHA) * qAccel.x;
  q.y = ALPHA * qGyro.y + (1.0f - ALPHA) * qAccel.y;
  q.z = ALPHA * qGyro.z + (1.0f - ALPHA) * qAccel.z;
  return quatNormalize(q);
}

// rotate vector v by quaternion q
Vec3 rotateVec(const Vec3 &v, const Quaternion &q) {
  // v' = q * v_quat * q_conj
  Quaternion vq = {0.0f, v.x, v.y, v.z};
  Quaternion qc = {q.w, -q.x, -q.y, -q.z};
  Quaternion tmp = quatMultiply(q, vq);
  Quaternion res = quatMultiply(tmp, qc);
  return {res.x, res.y, res.z};
}

// -------- rendering cube ----------
void drawCube(int x0, int y0, const Quaternion &q, uint16_t color) {
  static const Vec3 verts[8] = {
    {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1},
    {-1,-1, 1}, {1,-1, 1}, {1,1, 1}, {-1,1, 1}
  };

  static const uint8_t edges[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
  };

  const float scale = 24.0f; // cube size
  Vec3 proj[8];
  for (int i = 0; i < 8; ++i) {
    Vec3 r = rotateVec(verts[i], q);
    // simple orthographic projection (drop z) and center
    float sx = x0 + r.x * scale;
    float sy = y0 - r.y * scale; // invert y for screen coordinates
    proj[i] = { sx, sy, r.z };
  }

  // Draw edges
  for (int e = 0; e < 12; ++e) {
    int a = edges[e][0];
    int b = edges[e][1];
    tft.drawLine((int)proj[a].x, (int)proj[a].y, (int)proj[b].x, (int)proj[b].y, color);
  }
}

// -------- setup & loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};
  myI2C.begin(sdaPins, 2);

  tft.init();
  tft.setRotation(1); // landscape
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawCentreString("Dual MPU6050 3D Cubes", 120, 6, 2);

  // Wake-up sensors
  for (uint8_t sda : sdaPins) {
    if (mpuWrite(sda, PWR_MGMT_1, 0x00)) {
      Serial.printf("MPU on SDA %d woke up\n", sda);
    } else {
      Serial.printf("MPU on SDA %d failed to wake\n", sda);
    }
  }

  lastMicros = micros();
}

/*
void loop() {
  const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // Clear the two cube areas (top and bottom)
  tft.fillRect(0, 28, 240, 100, TFT_BLACK);   // top cube area
  tft.fillRect(0, 128, 240, 100, TFT_BLACK);  // bottom cube area

  for (int i = 0; i < 2; ++i) {
    uint8_t sda = sdaPins[i];
    uint8_t buf[14];
    if (!mpuRead(sda, ACCEL_XOUT_H, buf, 14)) {
      Serial.printf("MPU SDA %d read error\n", sda);
      continue;
    }

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    float axf = ax / 16384.0f;
    float ayf = ay / 16384.0f;
    float azf = az / 16384.0f;
    float gxf = gx / GYRO_SCALE;
    float gyf = gy / GYRO_SCALE;
    float gzf = gz / GYRO_SCALE;

    // integrate gyro to get new orientation
    Quaternion *qptr = (i == 0) ? &q1 : &q2;
    Quaternion qGyro = integrateGyro(*qptr, gxf, gyf, gzf, dt);

    // accel-derived orientation
    Quaternion qAccel = accelToQuat(axf, ayf, azf);

    // fuse
    *qptr = fuseOrientation(qGyro, qAccel);

    // draw cube
    int yCenter = (i == 0) ? 78 : 178;
    uint16_t color = (i == 0) ? TFT_ORANGE : TFT_CYAN;
    drawCube(120, yCenter, *qptr, color);

    // optional: show numeric pitch/roll estimation for debug
    // compute pitch/roll from accel
    float roll  = atan2(ayf, azf) * (180.0f / PI);
    float pitch = atan2(-axf, sqrt(ayf*ayf + azf*azf)) * (180.0f / PI);
    char buftxt[32];
    snprintf(buftxt, sizeof(buftxt), "S%d R:%5.1f P:%5.1f", i+1, roll, pitch);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(buftxt, 8, (i==0)? 38 : 138, 2);
    Serial.printf("S%d ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f q=(%.3f,%.3f,%.3f,%.3f)\n",
                  i+1, axf, ayf, azf, gxf, gyf, gzf, qptr->w, qptr->x, qptr->y, qptr->z);
  }

  // target ~30-40 FPS
  delay(25);
}
*/

void loop() {
  const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // Clear the cube drawing area
  tft.fillRect(0, 28, 240, 200, TFT_BLACK);

  for (int i = 0; i < 2; ++i) {
    uint8_t sda = sdaPins[i];
    uint8_t buf[14];
    if (!mpuRead(sda, ACCEL_XOUT_H, buf, 14)) {
      Serial.printf("MPU SDA %d read error\n", sda);
      continue;
    }

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    float axf = ax / 16384.0f;
    float ayf = ay / 16384.0f;
    float azf = az / 16384.0f;
    float gxf = gx / GYRO_SCALE;
    float gyf = gy / GYRO_SCALE;
    float gzf = gz / GYRO_SCALE;

    // integrate + fuse
    Quaternion *qptr = (i == 0) ? &q1 : &q2;
    Quaternion qGyro = integrateGyro(*qptr, gxf, gyf, gzf, dt);
    Quaternion qAccel = accelToQuat(axf, ayf, azf);
    *qptr = fuseOrientation(qGyro, qAccel);

    // horizontal positions: left cube (x=70), right cube (x=170)
    int xCenter = (i == 0) ? 70 : 170;
    int yCenter = 120;
    uint16_t color = (i == 0) ? TFT_ORANGE : TFT_CYAN;
    drawCube(xCenter, yCenter, *qptr, color);

    // optional debug text
    float roll  = atan2(ayf, azf) * (180.0f / PI);
    float pitch = atan2(-axf, sqrt(ayf*ayf + azf*azf)) * (180.0f / PI);
    char buftxt[32];
    snprintf(buftxt, sizeof(buftxt), "S%d R:%5.1f P:%5.1f", i+1, roll, pitch);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(buftxt, (i == 0) ? 8 : 128, 210, 2);

    Serial.printf("S%d ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f q=(%.3f,%.3f,%.3f,%.3f)\n",
                  i+1, axf, ayf, azf, gxf, gyf, gzf, qptr->w, qptr->x, qptr->y, qptr->z);
  }

  delay(25);
}
