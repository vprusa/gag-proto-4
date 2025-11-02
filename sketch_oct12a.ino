/**
 * ESP32 (LilyGO T-Display) — Hand Skeleton from MPU9250 (wrist) + MPU6050 (fingers) via PCA9548A
 *
 * Features
 *  - Wrist sensor: MPU9250 on mux channel 0 (direct I2C regs)
 *  - Finger sensors: MPU6050 on mux channels 1..6 (Jeff Rowberg/Electronic Cats lib)
 *  - Simple hand skeleton: wrist pivot (red), five guide rays (grey) to finger pivots (orange),
 *    and finger lines (yellow) that rotate around each finger pivot.
 *  - Entire skeleton rotates around wrist yaw; finger lines are offset back by wrist yaw
 *    when WRIST_RELATIVE is true.
 *  - Wrist data (roll/pitch/yaw) logged to Serial each frame.
 *
 * Wiring (from your notes)
 *  - ESP32 I2C: SCL=21, SDA=22
 *  - PCA9548A RESET=33, A0=25, A1=26, A2=27 (address pins optional; default address 0x70)
 *  - ACTIVE_CHANNELS lists connected mux ports (0..6 by default)
 *
 * Libraries (install via Library Manager)
 *  - "MPU6050" by Electronic Cats (Jeff Rowberg fork)
 *  - "TFT_eSPI" (configure for LilyGO T-Display ST7789 135x240)
 */

#include <Wire.h>
#include <MPU6050.h>   // for finger sensors
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

// ================= Feature flags =================
#define WRIST_RELATIVE   true     // when true, finger yaw is (finger - wrist)

// ================= Pins & PCA9548A =================
#define PIN_SCL          21
#define PIN_SDA          22
#define PIN_PCA_RST      33
#define PIN_PCA_A0       25
#define PIN_PCA_A1       26
#define PIN_PCA_A2       27
#define DRIVE_PCA_ADDR_PINS  false

#define PCA9548A_BASE_ADDR  0x70
static uint8_t pca_addr = PCA9548A_BASE_ADDR;

// ================= Active sensors =================
// Channel 0 = wrist (MPU9250), channels 1..6 = fingers (MPU6050)
const uint8_t ACTIVE_CHANNELS[] = {0,1,2,3,4,5,6};
const uint8_t NUM_SENSORS = sizeof(ACTIVE_CHANNELS)/sizeof(ACTIVE_CHANNELS[0]);

// ================= IMU addresses =================
#define MPU9250_ADDR   0x68   // change to 0x69 if AD0 is HIGH on your board

// ================= Globals =================
TFT_eSPI tft;

MPU6050 mpu[NUM_SENSORS];        // used for finger sensors (idx>0)
float roll_[NUM_SENSORS]  = {0};
float pitch_[NUM_SENSORS] = {0};
float yaw_[NUM_SENSORS]   = {0};
unsigned long lastT[NUM_SENSORS] = {0};
const float alpha = 0.98f;       // complementary filter gyro weight

// Colors
static const uint16_t COL_BG     = TFT_BLACK;
static const uint16_t COL_WRIST  = TFT_RED;
static const uint16_t COL_PIVOT  = 0xFD20;   // orange
static const uint16_t COL_GUIDE  = 0xC618;   // light gray
static const uint16_t COL_FINGER = TFT_YELLOW;
static const uint16_t COL_TEXT   = TFT_WHITE;

// ================= Hand parts & mapping =================
enum HandPart { WRIST=0, THUMB=1, INDEXF=2, MIDDLE=3, RING=4, LITTLE=5 };
static uint8_t PART_TO_SENSOR[6];

static void buildPartMapping() {
  PART_TO_SENSOR[WRIST] = 0;     // channel 0 -> wrist

  if (NUM_SENSORS <= 1) {
    // Fallback: everything uses the wrist sensor
    PART_TO_SENSOR[THUMB]  = 0;
    PART_TO_SENSOR[INDEXF] = 0;
    PART_TO_SENSOR[MIDDLE] = 0;
    PART_TO_SENSOR[RING]   = 0;
    PART_TO_SENSOR[LITTLE] = 0;
    return;
  }

  // Assign channels 1.. to fingers in order; wrap if fewer than five finger sensors
  const uint8_t avail = NUM_SENSORS - 1; // finger sensors count
  const uint8_t order[5] = { THUMB, INDEXF, MIDDLE, RING, LITTLE };
  for (uint8_t i=0; i<5; ++i) {
    PART_TO_SENSOR[order[i]] = 1 + (i % avail);
  }
}

// ================= PCA9548A helpers =================
static void pcaReset() {
  pinMode(PIN_PCA_RST, OUTPUT);
  digitalWrite(PIN_PCA_RST, LOW);
  delay(2);
  digitalWrite(PIN_PCA_RST, HIGH);
  delay(2);
}

static void pcaSelectChannel(uint8_t ch) {
  Wire.beginTransmission(pca_addr);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}

// ================= Math helpers =================
static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }

static void drawRayPolar(int x0, int y0, float angleDeg, float length, uint16_t color) {
  const float a = deg2rad(angleDeg);
  const int x1 = x0 + (int)(cosf(a) * length);
  const int y1 = y0 - (int)(sinf(a) * length);
  tft.drawLine(x0, y0, x1, y1, color);
}

// ================= Minimal I2C helpers (wrist MPU9250) =================
static void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t n, uint8_t* dst) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)n);
  for (uint8_t i=0; i<n && Wire.available(); ++i) dst[i] = Wire.read();
}

// ================= Wrist (MPU9250) =================
static bool initMPU9250_Wrist() {
  pcaSelectChannel(ACTIVE_CHANNELS[0]);

  // Reset and basic config
  i2cWriteByte(MPU9250_ADDR, 0x6B, 0x80);  // PWR_MGMT_1 reset
  delay(100);
  i2cWriteByte(MPU9250_ADDR, 0x6B, 0x01);  // clock = PLL, wake
  i2cWriteByte(MPU9250_ADDR, 0x6C, 0x00);  // enable all axes
  i2cWriteByte(MPU9250_ADDR, 0x1B, 0x00);  // gyro ±250 dps
  i2cWriteByte(MPU9250_ADDR, 0x1C, 0x00);  // accel ±2 g
  i2cWriteByte(MPU9250_ADDR, 0x1D, 0x03);  // DLPF ~44 Hz
  i2cWriteByte(MPU9250_ADDR, 0x19, 0x07);  // sample rate 1kHz/(1+7)=125 Hz

  // Prime orientation
  uint8_t buf[14];
  i2cReadBytes(MPU9250_ADDR, 0x3B, 14, buf);
  const int16_t ax = (buf[0]<<8) | buf[1];
  const int16_t ay = (buf[2]<<8) | buf[3];
  const int16_t az = (buf[4]<<8) | buf[5];

  const float axg = ax / 16384.0f;
  const float ayg = ay / 16384.0f;
  const float azg = az / 16384.0f;

  roll_[0]  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  pitch_[0] = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / (float)M_PI;
  yaw_[0]   = 0.0f;  // yaw will drift without magnetometer fusion

  lastT[0]  = millis();

  Serial.println("MPU9250 wrist initialized");
  return true; // (Optional: add WHO_AM_I check 0x75==0x71)
}

static void readMPU9250_Wrist(int16_t &ax, int16_t &ay, int16_t &az,
                              int16_t &gx, int16_t &gy, int16_t &gz) {
  pcaSelectChannel(ACTIVE_CHANNELS[0]);
  uint8_t buf[14];
  i2cReadBytes(MPU9250_ADDR, 0x3B, 14, buf);
  ax = (buf[0]<<8) | buf[1];
  ay = (buf[2]<<8) | buf[3];
  az = (buf[4]<<8) | buf[5];
  gx = (buf[8]<<8) | buf[9];
  gy = (buf[10]<<8) | buf[11];
  gz = (buf[12]<<8) | buf[13];
}

// ================= Generic init/update =================
static bool initOneIMU(uint8_t idx) {
  if (idx == 0) {
    return initMPU9250_Wrist();
  }

  // Finger sensor (MPU6050)
  pcaSelectChannel(ACTIVE_CHANNELS[idx]);
  mpu[idx].initialize();
  if (!mpu[idx].testConnection()) return false;

  lastT[idx] = millis();

  int16_t ax,ay,az,gx,gy,gz;
  mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  const float axg = ax / 16384.0f;
  const float ayg = ay / 16384.0f;
  const float azg = az / 16384.0f;
  roll_[idx]  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  pitch_[idx] = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / (float)M_PI;
  yaw_[idx]   = 0.0f;
  return true;
}

static void updateOneIMU(uint8_t idx) {
  int16_t ax,ay,az,gx,gy,gz;

  if (idx == 0) {
    readMPU9250_Wrist(ax,ay,az,gx,gy,gz);
  } else {
    pcaSelectChannel(ACTIVE_CHANNELS[idx]);
    mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  }

  const unsigned long now = millis();
  float dt = (now - lastT[idx]) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  lastT[idx] = now;

  const float axg = ax / 16384.0f;
  const float ayg = ay / 16384.0f;
  const float azg = az / 16384.0f;

  const float gxds = gx / 131.0f;
  const float gyds = gy / 131.0f;
  const float gzds = gz / 131.0f;

  const float roll_gyro  = roll_[idx]  + gxds * dt;
  const float pitch_gyro = pitch_[idx] + gyds * dt;
  const float yaw_gyro   = yaw_[idx]   + gzds * dt;

  const float roll_acc  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  const float pitch_acc = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / (float)M_PI;

  roll_[idx]  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
  pitch_[idx] = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
  yaw_[idx]   = yaw_gyro;  // gyro-only yaw

  if (yaw_[idx] > 180.0f)  yaw_[idx] -= 360.0f;
  if (yaw_[idx] < -180.0f) yaw_[idx] += 360.0f;

  if (idx == 0) {
    Serial.printf("Wrist data -> Roll:%6.1f  Pitch:%6.1f  Yaw:%6.1f", roll_[0], pitch_[0], yaw_[0]);
  }
}

static void drawHandPivotModel() {
  const int W = tft.width();
  const int H = tft.height();

  tft.fillScreen(COL_BG);

  // Wrist pivot (center-ish), red dot
  const int wristX = W/2 - 10;
  const int wristY = H/2 + 0;
  tft.fillCircle(wristX, wristY, 4, COL_WRIST);

  // Wrist yaw rotates the entire skeleton (guides + pivots) around the wrist
  const float yawW = yaw_[PART_TO_SENSOR[WRIST]];

  // Layout (local hand frame)
  const float baseAngle     = -40.0f; // start of finger fan (deg)
  const float stepAngle     =  20.0f; // spacing between fingers (deg)
  const float guideLen      =  50.0f; // wrist -> finger pivot (px)
  const float fingerLen     =  40.0f; // finger line length (px)
  const float fingerBaseDir = -90.0f; // default finger direction (up)

  // helper to wrap to [-180, 180]
  auto norm180 = [](float a)->float {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  };

  for (int i = 0; i < 5; ++i) {
    // 1) Skeleton (guide) rotated by wrist yaw
    const float localGuide = baseAngle + i * stepAngle;
    const float worldGuide = localGuide + yawW;
    drawRayPolar(wristX, wristY, worldGuide, guideLen, COL_GUIDE);

    // Finger pivot position (end of the guide)
    const float a = deg2rad(worldGuide);
    const int px = wristX + (int)(cosf(a) * guideLen);
    const int py = wristY - (int)(sinf(a) * guideLen);
    tft.fillCircle(px, py, 3, COL_PIVOT);

    // 2) Finger line: DO NOT offset by wrist rotation — use absolute finger yaw
    const uint8_t sIdx = PART_TO_SENSOR[i + 1]; // THUMB..LITTLE
    const float fingerYawAbs = norm180(yaw_[sIdx]);

    // World finger angle = skeleton rotation + base finger dir + absolute finger yaw
    const float worldFinger = yawW + fingerBaseDir + fingerYawAbs;
    drawRayPolar(px, py, worldFinger, fingerLen, COL_FINGER);
  }

  // HUD
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(1);
  tft.drawString("Skeleton: wrist-rot, fingers abs", W - 5, 10, 1);
}


// ================= Arduino setup/loop =================
void setup() {
  Serial.begin(115200);

  if (DRIVE_PCA_ADDR_PINS) {
    pinMode(PIN_PCA_A0, OUTPUT);
    pinMode(PIN_PCA_A1, OUTPUT);
    pinMode(PIN_PCA_A2, OUTPUT);
    digitalWrite(PIN_PCA_A0, LOW);
    digitalWrite(PIN_PCA_A1, LOW);
    digitalWrite(PIN_PCA_A2, LOW);
    // If you change these, update pca_addr = 0x70..0x77 accordingly
  }

  Wire.begin(PIN_SDA, PIN_SCL, 400000); // 400kHz
  pcaReset();
  buildPartMapping();

  tft.init();
  tft.setRotation(1); // 240x135 landscape
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(2);
  tft.drawString("MPU Hand Pivot", 120, 10);

  // Initialize all sensors
  for (uint8_t i=0; i<NUM_SENSORS; ++i) {
    if (!initOneIMU(i)) {
      tft.fillRect(0, 30 + i*16, tft.width(), 14, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.drawString("MPU FAIL CH" + String(ACTIVE_CHANNELS[i]), 5, 37 + i*16);
      tft.setTextColor(COL_TEXT, COL_BG);
    }
  }
}

void loop() {
  for (uint8_t i=0; i<NUM_SENSORS; ++i) updateOneIMU(i);
  drawHandPivotModel();
  delay(8);
}
