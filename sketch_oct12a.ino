/*
   ESP32 TTGO + Display + 2x GY25 (MPU6050 DMP)
   --------------------------------------------
   - IMU1: SDA1 = 21, SCL = 22
   - IMU2: SDA2 = 17, SCL = 22
   Each IMU read via its own I2C bus (Wire & Wire1)
   Two rotating cubes displayed side-by-side.
*/

#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

// IMU 1 (on Wire)
MPU6050 mpu1;
bool dmpReady1 = false;
uint16_t packetSize1;
uint8_t fifoBuffer1[64];
Quaternion q1;

// IMU 2 (on Wire1)
MPU6050 mpu2;
bool dmpReady2 = false;
uint16_t packetSize2;
uint8_t fifoBuffer2[64];
Quaternion q2;

const int SCREEN_W = 240;
const int SCREEN_H = 135;
float scale = 50.0f;
float zoffset = 2.5f;

// cube geometry
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
uint8_t edges[12][2] = {
  {0,1},{1,2},{2,3},{3,0},
  {4,5},{5,6},{6,7},{7,4},
  {0,4},{1,5},{2,6},{3,7}
};

// helper: draw one cube given quaternion and position offset
void drawCube(Quaternion &q, int offsetX, uint16_t colorEdge) {
  float qw=q.w, qx=q.x, qy=q.y, qz=q.z;
  float r00=1-2*(qy*qy+qz*qz);
  float r01=2*(qx*qy-qz*qw);
  float r02=2*(qx*qz+qy*qw);
  float r10=2*(qx*qy+qz*qw);
  float r11=1-2*(qx*qx+qz*qz);
  float r12=2*(qy*qz-qx*qw);
  float r20=2*(qx*qz-qy*qw);
  float r21=2*(qy*qz+qx*qw);
  float r22=1-2*(qx*qx+qy*qy);

  int cx = offsetX;
  int cy = SCREEN_H/2;
  int px[8], py[8];

  for (int i=0;i<8;i++) {
    float x=cubeVerts[i].x, y=cubeVerts[i].y, z=cubeVerts[i].z;
    float rx=r00*x+r01*y+r02*z;
    float ry=r10*x+r11*y+r12*z;
    float rz=r20*x+r21*y+r22*z;
    float invz=1.0f/(rz+zoffset);
    px[i]=cx+rx*invz*scale*2;
    py[i]=cy-ry*invz*scale*2;
  }

  for (int i=0;i<12;i++){
    int a=edges[i][0], b=edges[i][1];
    spr.drawLine(px[a], py[a], px[b], py[b], colorEdge);
  }

  // draw axes
  Vec3 ax={r00,r10,r20};
  Vec3 ay={r01,r11,r21};
  Vec3 az={r02,r12,r22};
  int len=30;
  spr.drawLine(cx,cy,cx+ax.x*len,cy-ax.y*len,TFT_RED);
  spr.drawLine(cx,cy,cx+ay.x*len,cy-ax.y*len,TFT_GREEN);
  spr.drawLine(cx,cy,cx+az.x*len,cy-az.y*len,TFT_BLUE);
}

void setupIMU(MPU6050 &mpu, TwoWire &wireBus, bool &readyFlag, uint16_t &packetSize) {
  // mpu.initialize(&wireBus);
  // mpu.setWire(&wireBus);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU connection failed!");
    return;
  }
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    readyFlag = true;
  } else {
    Serial.printf("DMP init failed (code %d)\n", devStatus);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  spr.createSprite(SCREEN_W, SCREEN_H);
  spr.setColorDepth(16);

  // Initialize both I2C buses
  Wire.begin(21,22);
  Wire1.begin(17,22);

  // Init both IMUs
  setupIMU(mpu1, Wire, dmpReady1, packetSize1);
  setupIMU(mpu2, Wire1, dmpReady2, packetSize2);
}

void loop() {
  if (!dmpReady1 && !dmpReady2) return;
  spr.fillSprite(TFT_BLACK);

  // ---- IMU 1 ----
  if (mpu1.getIntStatus() & 0x10) mpu1.resetFIFO();
  while (mpu1.getFIFOCount() < packetSize1) delay(1);
  mpu1.getFIFOBytes(fifoBuffer1, packetSize1);
  mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
  drawCube(q1, SCREEN_W/4, TFT_WHITE);

  // ---- IMU 2 ----
  if (mpu2.getIntStatus() & 0x10) mpu2.resetFIFO();
  while (mpu2.getFIFOCount() < packetSize2) delay(1);
  mpu2.getFIFOBytes(fifoBuffer2, packetSize2);
  mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
  drawCube(q2, 3*SCREEN_W/4, TFT_YELLOW);

  spr.drawString("IMU1", SCREEN_W/4-20, 10);
  spr.drawString("IMU2", 3*SCREEN_W/4-20, 10);
  spr.pushSprite(0,0);
}
