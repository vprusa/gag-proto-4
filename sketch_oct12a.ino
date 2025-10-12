/*
   ESP32 TTGO + Display + MPU6050 (GY-25 style)
   Quaternion visualization with smooth 3D cube using TFT_eSPI sprites
   SDA = 21, SCL = 22
   Display: configure your TFT_eSPI User_Setup.h for TTGO (e.g. ST7789, ILI9341)
*/

#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;

const int SCREEN_W = 240;  // adjust to your TTGO screen
const int SCREEN_H = 135;
int cx = SCREEN_W / 2;
int cy = SCREEN_H / 2;
float scale = 60.0f;       // scale for cube

// Cube vertices (8 corners)
struct Vec3 { float x, y, z; };
Vec3 cubeVerts[8] = {
  {-0.5, -0.5, -0.5},
  { 0.5, -0.5, -0.5},
  { 0.5,  0.5, -0.5},
  {-0.5,  0.5, -0.5},
  {-0.5, -0.5,  0.5},
  { 0.5, -0.5,  0.5},
  { 0.5,  0.5,  0.5},
  {-0.5,  0.5,  0.5}
};

// edges between vertices (pairs)
uint8_t edges[12][2] = {
  {0,1},{1,2},{2,3},{3,0},
  {4,5},{5,6},{6,7},{7,4},
  {0,4},{1,5},{2,6},{3,7}
};

// face colors for shading (RGB565)
uint16_t faceColor = TFT_BLUE;
uint16_t edgeColor = TFT_WHITE;
uint16_t bgColor   = TFT_BLACK;

void setup() {
  Serial.begin(115200);
  delay(100);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(bgColor);

  spr.createSprite(SCREEN_W, SCREEN_H);
  spr.setColorDepth(16);
  spr.fillSprite(bgColor);

  Wire.begin(21, 22);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    tft.println("MPU6050 connection failed!");
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  } else {
    tft.println("DMP init failed!");
    while (1);
  }
}

void loop() {
  if (!dmpReady) return;

  if (mpu.getIntStatus() & 0x10) {
    mpu.resetFIFO();
    return;
  }

  while (mpu.getFIFOCount() < packetSize) delay(1);

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // Convert quaternion to rotation matrix
  float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
  float r00 = 1 - 2*(qy*qy + qz*qz);
  float r01 = 2*(qx*qy - qz*qw);
  float r02 = 2*(qx*qz + qy*qw);
  float r10 = 2*(qx*qy + qz*qw);
  float r11 = 1 - 2*(qx*qx + qz*qz);
  float r12 = 2*(qy*qz - qx*qw);
  float r20 = 2*(qx*qz - qy*qw);
  float r21 = 2*(qy*qz + qx*qw);
  float r22 = 1 - 2*(qx*qx + qy*qy);

  // project all vertices
  int px[8], py[8];
  float zoffset = 2.5;
  for (int i = 0; i < 8; i++) {
    float x = cubeVerts[i].x;
    float y = cubeVerts[i].y;
    float z = cubeVerts[i].z;
    float rx = r00*x + r01*y + r02*z;
    float ry = r10*x + r11*y + r12*z;
    float rz = r20*x + r21*y + r22*z;
    float invz = 1.0 / (rz + zoffset);
    px[i] = cx + rx * invz * scale * 2;
    py[i] = cy - ry * invz * scale * 2;
  }

  // clear sprite
  spr.fillSprite(bgColor);

  // draw edges
  for (int i = 0; i < 12; i++) {
    int a = edges[i][0];
    int b = edges[i][1];
    spr.drawLine(px[a], py[a], px[b], py[b], edgeColor);
  }

  // draw axes
  Vec3 ax = {r00, r10, r20};
  Vec3 ay = {r01, r11, r21};
  Vec3 az = {r02, r12, r22};
  int len = 40;
  spr.drawLine(cx, cy, cx + ax.x*len, cy - ax.y*len, TFT_RED);
  spr.drawLine(cx, cy, cx + ay.x*len, cy - ay.y*len, TFT_GREEN);
  spr.drawLine(cx, cy, cx + az.x*len, cy - az.y*len, TFT_BLUE);

  // display roll/pitch/yaw text
  float ysqr = qy * qy;
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + ysqr);
  float roll = atan2f(sinr_cosp, cosr_cosp);
  float sinp = 2.0f * (qw * qy - qz * qx);
  float pitch = fabs(sinp) >= 1 ? copysignf(M_PI/2, sinp) : asinf(sinp);
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (ysqr + qz * qz);
  float yaw = atan2f(siny_cosp, cosy_cosp);

  char buf[64];
  snprintf(buf, sizeof(buf), "R:%.1f P:%.1f Y:%.1f",
           roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);
  spr.setTextColor(TFT_YELLOW, bgColor);
  spr.setTextSize(1);
  spr.drawString(buf, 5, 5);

  // push to screen
  spr.pushSprite(0, 0);
}
