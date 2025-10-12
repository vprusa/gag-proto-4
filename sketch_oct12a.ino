/*
  ESP32 TTGO + Display + MPU6050 DMP quaternion visualization
  - I2C pins: SDA = 21, SCL = 22 (Wire defaults used)
  - Requires:
      * MPU6050 library by Jeff Rowberg (MotionApps20 / DMP)
      * TFT_eSPI by Bodmer
  - Configure TFT_eSPI (User_Setup.h) for your TTGO display driver & pins.
  - Draws a 3D square (wireframe) rotated by the quaternion read from the sensor.
*/

#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h" // Jeff Rowberg's library (MotionApps)
#include <TFT_eSPI.h>                  // TFT library (config via User_Setup.h)
#include <SPI.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64]; // FIFO buffer

// quaternion container from the library
Quaternion q;

const int SCREEN_W = 240; // adjust in case your display is different
const int SCREEN_H = 135; // adjust accordingly

// 3D square model (local coordinates centered at origin)
struct Vec3 { float x, y, z; };
Vec3 cubePts[4]; // 4 vertices of a square (z=0 plane in model space)

// screen projection center and scale
float scale = 60.0f; // scale of model to screen
int cx = SCREEN_W / 2;
int cy = SCREEN_H / 2;

// previous transformed points used to erase old draw (simple doublebuffer-ish)
int prevX[4], prevY[4];

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("Initializing...");

  // Initialize display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);

  // Initialize I2C
  Wire.begin(21, 22); // SDA=21, SCL=22
  Serial.println("Wire started on SDA=21, SCL=22");

  // MPU6050 init
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    tft.setCursor(2, 2);
    tft.println("MPU6050 not found!");
    while (1) delay(1000);
  }

  // load and enable DMP
  Serial.println("Initializing DMP...");
  uint8_t devStatus = mpu.dmpInitialize();

  // supply offsets if you have them (recommended to calibrate for best results)
  // mpu.setXAccelOffset(...); mpu.setYAccelOffset(...); mpu.setZAccelOffset(...);
  // mpu.setXGyroOffset(...); mpu.setYGyroOffset(...); mpu.setZGyroOffset(...);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready.");
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    tft.setCursor(2, 2);
    tft.println("DMP init failed!");
  }

  // define a square of size 1 centered at origin in XY
  float half = 0.5f;
  cubePts[0] = { -half, -half, 0.0f };
  cubePts[1] = {  half, -half, 0.0f };
  cubePts[2] = {  half,  half, 0.0f };
  cubePts[3] = { -half,  half, 0.0f };

  // initialize prev points to offscreen
  for (int i = 0; i < 4; i++) { prevX[i] = prevY[i] = -9999; }

  tft.setTextSize(1);
  tft.setCursor(0,0);
  tft.println("MPU6050 DMP Orientation Visualizer");
  delay(800);
  tft.fillRect(0, 0, SCREEN_W, 16, TFT_BLACK); // clear top text area
}

void loop() {
  if (!dmpReady) {
    delay(100);
    return;
  }

  // check for new DMP data
  if (mpu.getIntStatus() & 0x10) {
    // FIFO overflow: reset
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  // wait for a packet
  while (mpu.getFIFOCount() < packetSize) {
    delay(1);
  }

  // read packet
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // retrieve quaternion (floating point)
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  // q.w, q.x, q.y, q.z are floats in the library's Quaternion struct

  // Build rotation matrix from quaternion
  float qw = q.w;
  float qx = q.x;
  float qy = q.y;
  float qz = q.z;

  // rotation matrix elements (row-major)
  float r00 = 1 - 2*(qy*qy + qz*qz);
  float r01 = 2*(qx*qy - qz*qw);
  float r02 = 2*(qx*qz + qy*qw);
  float r10 = 2*(qx*qy + qz*qw);
  float r11 = 1 - 2*(qx*qx + qz*qz);
  float r12 = 2*(qy*qz - qx*qw);
  float r20 = 2*(qx*qz - qy*qw);
  float r21 = 2*(qy*qz + qx*qw);
  float r22 = 1 - 2*(qx*qx + qy*qy);

  // Transform and project each vertex
  int screenX[4], screenY[4];
  for (int i = 0; i < 4; i++) {
    // model point
    float mx = cubePts[i].x;
    float my = cubePts[i].y;
    float mz = cubePts[i].z; // zero for square plane

    // rotate: v' = R * v
    float rx = r00*mx + r01*my + r02*mz;
    float ry = r10*mx + r11*my + r12*mz;
    float rz = r20*mx + r21*my + r22*mz;

    // simple perspective projection (small depth effect)
    // move the object away from camera by adding a z offset so rz + zoffset > 0
    float zoffset = 2.5f;
    float perspective = 1.0f / (rz + zoffset); // avoid divide by zero (zoffset chosen so denom>0)
    float px = rx * perspective * scale;
    float py = ry * perspective * scale;

    screenX[i] = cx + (int)round(px);
    screenY[i] = cy - (int)round(py); // invert y for screen coordinates
  }

  // erase previous polygon by redrawing lines in background color
  for (int i = 0; i < 4; i++) {
    int j = (i+1)%4;
    if (prevX[i] >= 0 && prevY[i] >= 0 && prevX[j] >= 0 && prevY[j] >= 0) {
      tft.drawLine(prevX[i], prevY[i], prevX[j], prevY[j], TFT_BLACK);
    }
  }

  // draw new polygon (wireframe) in white
  for (int i = 0; i < 4; i++) {
    int j = (i+1)%4;
    tft.drawLine(screenX[i], screenY[i], screenX[j], screenY[j], TFT_WHITE);
  }

  // draw axes as small colored lines on the center to show orientation
  // compute rotated axes vectors
  Vec3 axisX = { r00, r10, r20 }; // column 0 of rotation matrix
  Vec3 axisY = { r01, r11, r21 }; // column 1
  Vec3 axisZ = { r02, r12, r22 }; // column 2

  int ax = cx + (int)round(axisX.x * scale * 0.8f);
  int ay = cy - (int)round(axisX.y * scale * 0.8f);
  int bx = cx + (int)round(axisY.x * scale * 0.8f);
  int by = cy - (int)round(axisY.y * scale * 0.8f);
  int cxA = cx + (int)round(axisZ.x * scale * 0.8f);
  int cyA = cy - (int)round(axisZ.y * scale * 0.8f);

  // erase previous center axes (we do a small rectangle to clear area)
  tft.fillRect(cx - 30, cy - 30, 60, 60, TFT_BLACK);

  // draw axes
  tft.drawLine(cx, cy, ax, ay, TFT_RED);    // X axis - red
  tft.drawLine(cx, cy, bx, by, TFT_GREEN);  // Y axis - green
  tft.drawLine(cx, cy, cxA, cyA, TFT_BLUE); // Z axis - blue

  // show Euler angles (converted from quaternion) in text area
  float ysqr = qy * qy;

  // roll (x-axis rotation)
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + ysqr);
  float roll = atan2f(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = 2.0f * (qw * qy - qz * qx);
  float pitch;
  if (fabs(sinp) >= 1)
    pitch = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asinf(sinp);

  // yaw (z-axis rotation)
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (ysqr + qz * qz);
  float yaw = atan2f(siny_cosp, cosy_cosp);

  // convert to degrees
  float roll_d = roll * 180.0f / M_PI;
  float pitch_d = pitch * 180.0f / M_PI;
  float yaw_d = yaw * 180.0f / M_PI;

  // small text area on top-left
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  char buf[64];
  snprintf(buf, sizeof(buf), "R: %6.1f P: %6.1f Y: %6.1f", roll_d, pitch_d, yaw_d);
  tft.setCursor(2, 2);
  tft.print(buf);

  // Save current points to prev arrays
  for (int i = 0; i < 4; i++) {
    prevX[i] = screenX[i];
    prevY[i] = screenY[i];
  }

  // small delay to control frame rate
  delay(20);
}
