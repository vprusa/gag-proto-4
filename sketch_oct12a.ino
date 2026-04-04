#pragma once
/*
 * gesture_glove_ttgo_merged_fixed.ino (header implementation)
 *
 * Merged TTGO / ESP32 glove sketch.
 *
 * What this version keeps from the newer TTGO sketch:
 *   - ESP32 TTGO / T-Display target
 *   - PCA9548A/TCA9548A mux
 *   - wrist MPU9250 + 5 finger MPU6050-class sensors
 *   - wrist auxiliary GY-511 (LSM303DLHC accel+mag)
 *
 * What this version ports from the older firmware:
 *   - gesture recognition pipeline
 *   - command / gesture history log on the display
 *   - visualization modes with circular switching
 *   - per-gesture blink feedback on the underlying sensor visualization
 *   - optional BLE mouse actions
 *   - optional per-sensor vibration actions
 *   - hardware and software offsets in a dedicated store
 *
 * Assumptions used in this migration:
 *   - sensor index 0 = wrist MPU9250
 *   - sensor index 1..5 = thumb, index, middle, ring, little
 *   - sensor index 6 = wrist auxiliary GY-511
 *   - software offsets are used to neutralize the current pose at boot by default
 *   - hardware offsets are stored in the same ax/ay/az/gx/gy/gz format as the old code
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MPU6050.h>
#include <math.h>

#include "GagRecogMerged.h"
#include "GagOffsetsMerged.h"
#include "GagTtgoVizMerged.h"

#ifndef TFT_ORANGE
#define TFT_ORANGE 0xFDA0
#endif

#ifndef MOUSE_LEFT
#define MOUSE_LEFT 0x01
#endif
#ifndef MOUSE_RIGHT
#define MOUSE_RIGHT 0x02
#endif
#ifndef MOUSE_MIDDLE
#define MOUSE_MIDDLE 0x04
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if __has_include(<BleMouse.h>)
  #include <BleMouse.h>
  #define GAG_HAVE_BLE_MOUSE 1
#else
  #define GAG_HAVE_BLE_MOUSE 0
#endif

#ifndef GAG_ENABLE_BLE_MOUSE
#define GAG_ENABLE_BLE_MOUSE GAG_HAVE_BLE_MOUSE
#endif

#ifndef GAG_ENABLE_VIBRATION
#define GAG_ENABLE_VIBRATION 1
#endif

#ifndef GAG_AUTO_CAPTURE_SW_NEUTRAL
#define GAG_AUTO_CAPTURE_SW_NEUTRAL 1
#endif

#ifndef GAG_MEASURE_HW_OFFSETS_AT_BOOT
#define GAG_MEASURE_HW_OFFSETS_AT_BOOT 1
#endif

#ifndef GAG_TFT_ROTATION
#define GAG_TFT_ROTATION 0  // Previous build used 1; 0 rotates the whole UI 90° CCW.
#endif

#define GAG_WRIST_QUAT_SOURCE_MPU9250 0
#define GAG_WRIST_QUAT_SOURCE_GY511   1
#ifndef GAG_WRIST_QUAT_SOURCE
#define GAG_WRIST_QUAT_SOURCE GAG_WRIST_QUAT_SOURCE_GY511
#endif

#if (GAG_WRIST_QUAT_SOURCE != GAG_WRIST_QUAT_SOURCE_MPU9250) &&     (GAG_WRIST_QUAT_SOURCE != GAG_WRIST_QUAT_SOURCE_GY511)
#error "GAG_WRIST_QUAT_SOURCE must be GAG_WRIST_QUAT_SOURCE_MPU9250 or GAG_WRIST_QUAT_SOURCE_GY511"
#endif

// =====================
// Pins & PCA9548A / TCA9548A
// =====================
#define PIN_I2C_SDA   22
#define PIN_I2C_SCL   21
#define PIN_PCA_RST   33
#define PIN_PCA_A0    25
#define PIN_PCA_A1    26
#define PIN_PCA_A2    27
#define DRIVE_PCA_ADDR_PINS  false
#define PCA9548A_BASE_ADDR   0x70

static uint8_t pca_addr = PCA9548A_BASE_ADDR;

// =====================
// Sensor topology
// =====================
enum SensorIndex : uint8_t {
  SENSOR_WRIST = 0,
  SENSOR_THUMB = 1,
  SENSOR_INDEX = 2,
  SENSOR_MIDDLE = 3,
  SENSOR_RING = 4,
  SENSOR_LITTLE = 5,
  SENSOR_WRIST_AUX = 6,
  SENSOR_COUNT_ALL = 7,
  SENSOR_COUNT_HAND = 6,
};

// [0] wrist MPU9250, [1..5] fingers
// const uint8_t ACTIVE_CHANNELS[] = {1, 0, 7, 3, 4, 5};
// const uint8_t ACTIVE_CHANNELS[] = {1, 7, 0, 3, 4, 5};
// const uint8_t ACTIVE_CHANNELS[] = {0, 3, 4, 7, 1, 5};
const uint8_t ACTIVE_CHANNELS[] = {2, 0, 3, 4, 7, 1, 5};
// const uint8_t ACTIVE_CHANNELS[] = {2, 0, 3, 4};
const uint8_t NUM_ACTIVE_IMUS = sizeof(ACTIVE_CHANNELS) / sizeof(ACTIVE_CHANNELS[0]);
static const uint8_t CH_GY511 = 2;

// Indexes into ACTIVE_CHANNELS / roll_ / pitch_ / yaw_ arrays.
static const uint8_t FINGER_MAP[5] = {SENSOR_THUMB, SENSOR_INDEX, SENSOR_MIDDLE, SENSOR_RING, SENSOR_LITTLE};

// =====================
// Optional vibration motors
// =====================
#if GAG_ENABLE_VIBRATION
static const int8_t MOTOR_PINS[SENSOR_COUNT_ALL] = {17, 2, 15, 13, 25, 26, 27};
static const bool MOTOR_ACTIVE_HIGH = true;
struct MotorState { bool active = false; uint32_t until_ms = 0; };
static MotorState g_motorState[SENSOR_COUNT_ALL];
#endif

// =====================
// Display
// =====================
TFT_eSPI tft;
static gag::viz::TtgoDisplayViz g_viz;

// =====================
// Colors per sensor
// =====================
static const uint16_t SENSOR_COLORS[SENSOR_COUNT_ALL] = {
  TFT_RED,      // wrist MPU9250
  TFT_YELLOW,   // thumb
  TFT_GREEN,    // index
  TFT_CYAN,     // middle
  TFT_MAGENTA,  // ring
  TFT_ORANGE,   // little
  TFT_BLUE      // wrist aux GY-511
};

// =====================
// Recognition / actions
// =====================
static gag::Recognizer g_recognizer;
static gag::offsets::OffsetStore g_offsets;

#if GAG_ENABLE_BLE_MOUSE && defined(MASTER_HAND)
static BleMouse g_bleMouse("GAG Mouse", "OpenAI", 100);
#elif GAG_ENABLE_BLE_MOUSE
static BleMouse g_bleMouse("GAG Mouse", "OpenAI", 100);
#endif

// =====================
// Minimal vector helpers
// =====================
struct Vec3 {
  float x, y, z;
};

static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r){ return r * 180.0f / (float)M_PI; }
static inline float wrap180(float a){ while(a > 180.0f) a -= 360.0f; while(a < -180.0f) a += 360.0f; return a; }
static inline float deltaAngleDeg(float a, float b){ return wrap180(a - b); }

// =====================
// IMU state
// =====================
MPU6050 mpu[NUM_ACTIVE_IMUS];
float roll_[NUM_ACTIVE_IMUS]  = {0};
float pitch_[NUM_ACTIVE_IMUS] = {0};
float yaw_[NUM_ACTIVE_IMUS]   = {0};
unsigned long lastT[NUM_ACTIVE_IMUS] = {0};
const float alpha = 0.98f;

bool wristMagOk = false;
float yawMagWristDeg = 0.0f;
Vec3 wristMagRaw{0,0,0};

bool gy511Ok = false;
Vec3 gy511Accel_g{0,0,0};
Vec3 gy511MagRaw{0,0,0};
float gy511RollDeg = 0.0f;
float gy511PitchDeg = 0.0f;
float gy511YawMagDeg = 0.0f;
unsigned long gy511LastT = 0;

// =====================
// Default offsets
// =====================
// Old 6-component layout: ax, ay, az, gx, gy, gz.
// Edit these after calibration. Left at 0 by default in the merged TTGO build.
static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
  { 0, 0, 0, 0, 0, 0 }, // wrist MPU9250
  { 0, 0, 0, 0, 0, 0 }, // thumb
  { 0, 0, 0, 0, 0, 0 }, // index
  { 0, 0, 0, 0, 0, 0 }, // middle
  { 0, 0, 0, 0, 0, 0 }, // ring
  { 0, 0, 0, 0, 0, 0 }, // little
  { 0, 0, 0, 0, 0, 0 }, // wrist aux (accel-only used in this sketch)
};

// Default per-sensor mounting compensation applied before neutral offsets.
// Fingers use a 90 deg clockwise compensation around Z because the GY25 boards
// are mounted 90 deg counterclockwise. The wrist GY-511 uses the inverse of
// its physical mounting: undo the 90 deg X rotation, then undo the 180 deg Z
// rotation.
static const gag::Quaternion DEFAULT_SENSOR_ROTATION[SENSOR_COUNT_ALL] = {
  gag::Quaternion(),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 180.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)
  ),
};

// =====================
// MPU9250 / AK8963 registers
// =====================
#define MPU9250_ADDR   0x68
#define AK8963_ADDR    0x0C
#define REG_PWR_MGMT_1     0x6B
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_USER_CTRL      0x6A
#define REG_INT_PIN_CFG    0x37
#define AK8963_WHO_AM_I    0x00
#define AK8963_ST1         0x02
#define AK8963_HXL         0x03
#define AK8963_CNTL1       0x0A
#define AK8963_ASAX        0x10

// =====================
// LSM303DLHC / GY-511
// =====================
#define LSM_ACC_ADDR       0x19
#define LSM_MAG_ADDR       0x1E
#define LSM_CTRL_REG1_A    0x20
#define LSM_CTRL_REG4_A    0x23
#define LSM_OUT_X_L_A      0x28
#define LSM_CRA_REG_M      0x00
#define LSM_CRB_REG_M      0x01
#define LSM_MR_REG_M       0x02
#define LSM_OUT_X_H_M      0x03

// =====================
// I2C helpers
// =====================
static void pcaReset(){
  if (DRIVE_PCA_ADDR_PINS){
    pinMode(PIN_PCA_A0, OUTPUT); pinMode(PIN_PCA_A1, OUTPUT); pinMode(PIN_PCA_A2, OUTPUT);
    digitalWrite(PIN_PCA_A0, LOW); digitalWrite(PIN_PCA_A1, LOW); digitalWrite(PIN_PCA_A2, LOW);
  }
  pinMode(PIN_PCA_RST, OUTPUT);
  digitalWrite(PIN_PCA_RST, LOW); delay(2);
  digitalWrite(PIN_PCA_RST, HIGH); delay(2);
}

static void pcaSelect(uint8_t ch){
  Wire.beginTransmission(pca_addr);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}

static void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t i2cReadByte(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

static void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t*buf, uint8_t len){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)len);
  for (uint8_t i = 0; i < len && Wire.available(); ++i) buf[i] = Wire.read();
}

// =====================
// Magnetometer helpers
// =====================
static float yawFromMagTiltComp(const Vec3& m, float rollDeg, float pitchDeg){
  float roll = deg2rad(rollDeg);
  float pitch = deg2rad(pitchDeg);
  float cr = cosf(roll), sr = sinf(roll);
  float cp = cosf(pitch), sp = sinf(pitch);
  float mx2 = m.x * cp + m.z * sp;
  float my2 = m.x * sr * sp + m.y * cr - m.z * sr * cp;
  return wrap180(rad2deg(atan2f(-my2, mx2)));
}

// =====================
// Optional vibration
// =====================
#if GAG_ENABLE_VIBRATION
static void setMotorOutput(uint8_t sensorIdx, bool on) {
  if (sensorIdx >= SENSOR_COUNT_ALL) return;
  const int8_t pin = MOTOR_PINS[sensorIdx];
  if (pin < 0) return;
  digitalWrite((uint8_t)pin, (on == MOTOR_ACTIVE_HIGH) ? HIGH : LOW);
}

static void initMotors() {
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    const int8_t pin = MOTOR_PINS[i];
    if (pin < 0) continue;
    pinMode((uint8_t)pin, OUTPUT);
    setMotorOutput(i, false);
    g_motorState[i] = MotorState();
  }
}

static void scheduleVibration(uint8_t sensorMask, uint16_t durationMs) {
  const uint32_t until = millis() + durationMs;
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    if (!(sensorMask & (1u << i))) continue;
    g_motorState[i].active = true;
    g_motorState[i].until_ms = until;
    setMotorOutput(i, true);
  }
}

static void updateVibrations() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    if (!g_motorState[i].active) continue;
    if ((int32_t)(now - g_motorState[i].until_ms) >= 0) {
      g_motorState[i].active = false;
      setMotorOutput(i, false);
    }
  }
}
#else
static void initMotors() {}
static void scheduleVibration(uint8_t, uint16_t) {}
static void updateVibrations() {}
#endif

// =====================
// BLE mouse helpers
// =====================
static void execMouseAction(const gag::MouseAction& mouse) {
#if GAG_ENABLE_BLE_MOUSE
  if (!g_bleMouse.isConnected()) return;
  switch (mouse.type) {
    case gag::MouseActionType::MOVE:
      g_bleMouse.move(mouse.dx, mouse.dy, 0, 0);
      break;
    case gag::MouseActionType::CLICK:
      g_bleMouse.click(mouse.button);
      break;
    case gag::MouseActionType::PRESS:
      g_bleMouse.press(mouse.button);
      break;
    case gag::MouseActionType::RELEASE:
      g_bleMouse.release(mouse.button);
      break;
    case gag::MouseActionType::SCROLL:
      g_bleMouse.move(0, 0, mouse.wheel, mouse.hWheel);
      break;
    default:
      break;
  }
#else
  (void)mouse;
#endif
}

// =====================
// Sensor mapping / conversions
// =====================
static inline gag::Sensor mapToRecognizerSensor(uint8_t sensorIdx) {
  switch (sensorIdx) {
    case SENSOR_WRIST: return gag::Sensor::WRIST;
    case SENSOR_THUMB: return gag::Sensor::THUMB;
    case SENSOR_INDEX: return gag::Sensor::INDEX;
    case SENSOR_MIDDLE: return gag::Sensor::MIDDLE;
    case SENSOR_RING: return gag::Sensor::RING;
    case SENSOR_LITTLE: return gag::Sensor::LITTLE;
    default: return gag::Sensor::WRIST;
  }
}

static gag::Quaternion eulerSensorToQuat(float rollDeg, float pitchDeg, float yawDeg) {
  return gag::Quaternion::fromEulerZyxDeg(yawDeg, pitchDeg, rollDeg);
}

static gag::Quaternion rawQuaternionForPhysicalSensor(uint8_t sensorIdx) {
  if (sensorIdx == SENSOR_WRIST_AUX) {
    return eulerSensorToQuat(gy511RollDeg, gy511PitchDeg, gy511YawMagDeg);
  }
  if (sensorIdx == SENSOR_WRIST) {
    const float yawDeg = wristMagOk ? yawMagWristDeg : yaw_[SENSOR_WRIST];
    return eulerSensorToQuat(roll_[SENSOR_WRIST], pitch_[SENSOR_WRIST], yawDeg);
  }
  return eulerSensorToQuat(roll_[sensorIdx], pitch_[sensorIdx], yaw_[sensorIdx]);
}

static gag::Quaternion applyDefaultSensorRotation(uint8_t sensorIdx, const gag::Quaternion& rawIn) {
  gag::Quaternion raw = rawIn;
  raw.normalizeInPlace();
  if (sensorIdx >= SENSOR_COUNT_ALL) return raw;
  gag::Quaternion out = gag::Quaternion::mul(DEFAULT_SENSOR_ROTATION[sensorIdx], raw);
  out.normalizeInPlace();
  return out;
}

static bool physicalSensorQuaternionAvailable(uint8_t sensorIdx) {
  if (sensorIdx == SENSOR_WRIST_AUX) {
    return gy511Ok;
  }
  return true;
}

static gag::Quaternion correctedQuaternionForPhysicalSensor(uint8_t sensorIdx) {
  return g_offsets.applySoftwareOffset(sensorIdx, applyDefaultSensorRotation(sensorIdx, rawQuaternionForPhysicalSensor(sensorIdx)));
}

static uint8_t selectedWristQuaternionPhysicalSensor() {
#if GAG_WRIST_QUAT_SOURCE == GAG_WRIST_QUAT_SOURCE_GY511
  return gy511Ok ? SENSOR_WRIST_AUX : SENSOR_WRIST;
#else
  return SENSOR_WRIST;
#endif
}

static bool selectedWristQuaternionAvailable() {
  return physicalSensorQuaternionAvailable(selectedWristQuaternionPhysicalSensor());
}

static gag::Quaternion correctedLogicalWristQuaternion() {
  return correctedQuaternionForPhysicalSensor(selectedWristQuaternionPhysicalSensor());
}

static uint16_t selectedLogicalWristColor() {
  return SENSOR_COLORS[selectedWristQuaternionPhysicalSensor()];
}

// =====================
// GY-511
// =====================
static bool initGY511(){
  pcaSelect(CH_GY511);
  i2cWriteByte(LSM_ACC_ADDR, LSM_CTRL_REG1_A, 0x57); // 100Hz, XYZ enable
  i2cWriteByte(LSM_ACC_ADDR, LSM_CTRL_REG4_A, 0x00); // ±2g
  i2cWriteByte(LSM_MAG_ADDR, LSM_CRA_REG_M,  0x14); // 30Hz
  i2cWriteByte(LSM_MAG_ADDR, LSM_CRB_REG_M,  0x20); // +/-1.3 gauss
  i2cWriteByte(LSM_MAG_ADDR, LSM_MR_REG_M,   0x00); // continuous
  delay(20);
  uint8_t who = i2cReadByte(LSM_MAG_ADDR, 0x0A);
  gy511LastT = millis();
  return (who == 'H');
}

static bool readGY511Accel(Vec3 &accel_g){
  pcaSelect(CH_GY511);
  uint8_t buf[6] = {0};
  i2cReadBytes(LSM_ACC_ADDR, (LSM_OUT_X_L_A | 0x80), buf, 6);
  int16_t ax = (int16_t)((buf[1]<<8) | buf[0]);
  int16_t ay = (int16_t)((buf[3]<<8) | buf[2]);
  int16_t az = (int16_t)((buf[5]<<8) | buf[4]);
  const gag::offsets::HwOffset6 hw = g_offsets.hardware(SENSOR_WRIST_AUX);
  ax -= hw.ax; ay -= hw.ay; az -= hw.az;
  accel_g.x = (float)ax / 16384.0f;
  accel_g.y = (float)ay / 16384.0f;
  accel_g.z = (float)az / 16384.0f;
  return true;
}

static bool readGY511Mag(Vec3 &magRaw){
  pcaSelect(CH_GY511);
  uint8_t buf[6] = {0};
  i2cReadBytes(LSM_MAG_ADDR, LSM_OUT_X_H_M, buf, 6);
  int16_t mx = (int16_t)((buf[0]<<8) | buf[1]);
  int16_t mz = (int16_t)((buf[2]<<8) | buf[3]);
  int16_t my = (int16_t)((buf[4]<<8) | buf[5]);
  magRaw.x = (float)mx;
  magRaw.y = (float)my;
  magRaw.z = (float)mz;
  return true;
}

static void updateGY511(){
  if (!gy511Ok) return;
  Vec3 a, m;
  if (!readGY511Accel(a)) return;
  if (!readGY511Mag(m)) return;
  gy511Accel_g = a;
  gy511MagRaw = m;
  gy511RollDeg  = rad2deg(atan2f(a.y, a.z));
  gy511PitchDeg = rad2deg(atan2f(-a.x, sqrtf(a.y*a.y + a.z*a.z)));
  gy511YawMagDeg = yawFromMagTiltComp(m, gy511RollDeg, gy511PitchDeg);
  gy511LastT = millis();
}

// =====================
// Wrist AK8963
// =====================
static bool initWristMagAK8963(){
  pcaSelect(ACTIVE_CHANNELS[SENSOR_WRIST]);
  i2cWriteByte(MPU9250_ADDR, REG_INT_PIN_CFG, 0x02); // bypass enable
  delay(10);
  uint8_t who = i2cReadByte(AK8963_ADDR, AK8963_WHO_AM_I);
  if (who != 0x48) return false;
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x00); delay(10);
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x16); delay(10); // continuous 2, 16-bit
  return true;
}

static bool readWristMag(Vec3 &magOut){
  pcaSelect(ACTIVE_CHANNELS[SENSOR_WRIST]);
  uint8_t st1 = i2cReadByte(AK8963_ADDR, AK8963_ST1);
  if (!(st1 & 0x01)) return false;
  uint8_t buf[7] = {0};
  i2cReadBytes(AK8963_ADDR, AK8963_HXL, buf, 7);
  int16_t mx = (int16_t)((buf[1]<<8) | buf[0]);
  int16_t my = (int16_t)((buf[3]<<8) | buf[2]);
  int16_t mz = (int16_t)((buf[5]<<8) | buf[4]);
  magOut.x = (float)mx;
  magOut.y = (float)my;
  magOut.z = (float)mz;
  return true;
}

static void updateWristMagYaw(){
  if (!wristMagOk) return;
  Vec3 m;
  if (readWristMag(m)) {
    wristMagRaw = m;
    yawMagWristDeg = yawFromMagTiltComp(m, roll_[SENSOR_WRIST], pitch_[SENSOR_WRIST]);
  }
}

// =====================
// IMU init/update
// =====================
static bool initOneIMU(uint8_t idx){
  const uint8_t ch = ACTIVE_CHANNELS[idx];
  pcaSelect(ch);

  if (idx == SENSOR_WRIST) {
    i2cWriteByte(MPU9250_ADDR, REG_PWR_MGMT_1, 0x00); delay(10);
    i2cWriteByte(MPU9250_ADDR, REG_GYRO_CONFIG, 0x00);
    i2cWriteByte(MPU9250_ADDR, REG_ACCEL_CONFIG, 0x00);
    return true;
  }

  mpu[idx].initialize();
  bool ok = mpu[idx].testConnection();
  if (ok) {
    mpu[idx].setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu[idx].setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    const gag::offsets::HwOffset6 hw = g_offsets.hardware(idx);
    mpu[idx].setXAccelOffset(hw.ax);
    mpu[idx].setYAccelOffset(hw.ay);
    mpu[idx].setZAccelOffset(hw.az);
    mpu[idx].setXGyroOffset(hw.gx);
    mpu[idx].setYGyroOffset(hw.gy);
    mpu[idx].setZGyroOffset(hw.gz);
  }
  return ok;
}

static void updateOneIMU(uint8_t idx){
  int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  pcaSelect(ACTIVE_CHANNELS[idx]);

  if (idx == SENSOR_WRIST) {
    uint8_t buf[14] = {0};
    i2cReadBytes(MPU9250_ADDR, REG_ACCEL_XOUT_H, buf, 14);
    ax = (int16_t)((buf[0]<<8)  | buf[1]);
    ay = (int16_t)((buf[2]<<8)  | buf[3]);
    az = (int16_t)((buf[4]<<8)  | buf[5]);
    gx = (int16_t)((buf[8]<<8)  | buf[9]);
    gy = (int16_t)((buf[10]<<8) | buf[11]);
    gz = (int16_t)((buf[12]<<8) | buf[13]);

    // Apply the stored 6-component offsets as pre-fusion biases on the raw wrist path.
    const gag::offsets::HwOffset6 hw = g_offsets.hardware(SENSOR_WRIST);
    ax -= hw.ax; ay -= hw.ay; az -= hw.az;
    gx -= hw.gx; gy -= hw.gy; gz -= hw.gz;
  } else {
    mpu[idx].getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }

  unsigned long now = millis();
  float dt = (now - lastT[idx]) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastT[idx] = now;

  float axg = ax / 16384.0f;
  float ayg = ay / 16384.0f;
  float azg = az / 16384.0f;
  float gxds = gx / 131.0f;
  float gyds = gy / 131.0f;
  float gzds = gz / 131.0f;

  float rollGyro  = roll_[idx]  + gxds * dt;
  float pitchGyro = pitch_[idx] + gyds * dt;
  float yawGyro   = yaw_[idx]   + gzds * dt;

  float rollAcc  = rad2deg(atan2f(ayg, azg));
  float pitchAcc = rad2deg(atan2f(-axg, sqrtf(ayg*ayg + azg*azg)));

  roll_[idx]  = alpha * rollGyro  + (1.0f - alpha) * rollAcc;
  pitch_[idx] = alpha * pitchGyro + (1.0f - alpha) * pitchAcc;
  yaw_[idx]   = wrap180(yawGyro);
}

// =====================
// Optional boot-time offset measurement
// =====================
static bool readRawSampleForOffset(uint8_t sensorIdx, gag::offsets::RawImuSample& out) {
  if (sensorIdx == SENSOR_WRIST_AUX) {
    Vec3 a;
    if (!readGY511Accel(a)) return false;
    out.ax = (int16_t)(a.x * 16384.0f);
    out.ay = (int16_t)(a.y * 16384.0f);
    out.az = (int16_t)(a.z * 16384.0f);
    out.gx = out.gy = out.gz = 0;
    return true;
  }

  pcaSelect(ACTIVE_CHANNELS[sensorIdx]);
  if (sensorIdx == SENSOR_WRIST) {
    uint8_t buf[14] = {0};
    i2cReadBytes(MPU9250_ADDR, REG_ACCEL_XOUT_H, buf, 14);
    out.ax = (int16_t)((buf[0]<<8)  | buf[1]);
    out.ay = (int16_t)((buf[2]<<8)  | buf[3]);
    out.az = (int16_t)((buf[4]<<8)  | buf[5]);
    out.gx = (int16_t)((buf[8]<<8)  | buf[9]);
    out.gy = (int16_t)((buf[10]<<8) | buf[11]);
    out.gz = (int16_t)((buf[12]<<8) | buf[13]);
    return true;
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpu[sensorIdx].getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  out.ax = ax; out.ay = ay; out.az = az; out.gx = gx; out.gy = gy; out.gz = gz;
  return true;
}

static void measureHardwareOffsetsAtBoot() {
#if GAG_MEASURE_HW_OFFSETS_AT_BOOT
  const gag::offsets::StableWindowConfig cfg;
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    gag::offsets::StablePoseAccumulator acc;
    uint32_t deadline = millis() + 4000;
    while ((int32_t)(millis() - deadline) < 0 && !acc.isStable(cfg)) {
      gag::offsets::RawImuSample sample;
      if (readRawSampleForOffset(s, sample)) {
        acc.push(sample);
      }
      delay(5);
    }
    gag::offsets::SampleStats stats = acc.stats();
    if (stats.valid && acc.isStable(cfg)) {
      g_offsets.setHardware(s, gag::offsets::computeHardwareOffsetsFromStablePose(stats));
    }
  }
#endif
}

// =====================
// Boot-time software offset neutral capture
// =====================
static gag::Quaternion averageCurrentSensorQuat(uint8_t sensorIdx, uint8_t samples = 16) {
  gag::Quaternion ref(1,0,0,0);
  gag::Quaternion sum(0,0,0,0);
  bool haveRef = false;

  for (uint8_t i = 0; i < samples; ++i) {
    if (sensorIdx == SENSOR_WRIST_AUX) {
      updateGY511();
    } else {
      updateOneIMU(sensorIdx);
      if (sensorIdx == SENSOR_WRIST) updateWristMagYaw();
    }

    gag::Quaternion q = applyDefaultSensorRotation(sensorIdx, rawQuaternionForPhysicalSensor(sensorIdx));
    q.normalizeInPlace();

    if (!haveRef) { ref = q; haveRef = true; }
    if (gag::Quaternion::dot(ref, q) < 0.0f) {
      q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
    }
    sum.w += q.w; sum.x += q.x; sum.y += q.y; sum.z += q.z;
    delay(10);
  }

  sum.normalizeInPlace();
  return sum;
}

static void autoCaptureSoftwareNeutralOffsets() {
#if GAG_AUTO_CAPTURE_SW_NEUTRAL
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    const gag::Quaternion qAvg = averageCurrentSensorQuat(s, 16);
    const gag::Quaternion off = gag::offsets::OffsetStore::computeNeutralizingSoftwareOffset(qAvg, gag::Quaternion());
    g_offsets.setSoftwareQuaternion(s, off);
  }
#endif
}

// =====================
// Gesture definitions
// =====================
static void addPoseGesture(const char* name,
                           const char* command,
                           const char* label,
                           gag::Sensor sensor,
                           const gag::Quaternion& target,
                           float thresholdDeg,
                           uint32_t cooldownMs,
                           uint32_t maxTimeMs,
                           const gag::GestureAction& action) {
  gag::GestureDef g;
  strncpy(g.name, name, sizeof(g.name)-1);
  strncpy(g.command, command, sizeof(g.command)-1);
  strncpy(g.label, label, sizeof(g.label)-1);
  g.threshold_rad = deg2rad(thresholdDeg);
  g.recognition_delay_ms = cooldownMs;
  g.max_time_ms = maxTimeMs;
  g.relative = false;
  g.active = true;
  g.action = action;
  g.perSensor[(uint8_t)sensor].len = 2;
  g.perSensor[(uint8_t)sensor].q[0] = target;
  g.perSensor[(uint8_t)sensor].q[1] = target;
  g_recognizer.addGesture(g);
}

static void installDefaultGestures() {
  using gag::MouseActionType;

  // Little finger up -> visualization mode cycle + blink + vibration.
  {
    gag::GestureAction a;
    a.switch_visualization_mode = true;
    a.blink_visualization = true;
    a.blink_color565 = TFT_ORANGE;
    a.vibrate = true;
    a.vibrate_sensor_mask = (1u << SENSOR_LITTLE);
    a.vibrate_duration_ms = 220;
    addPoseGesture("little_up_mode", "GAG_CYCLE_MODE", "MODE",
                   gag::Sensor::LITTLE,
                   gag::Quaternion::fromAxisAngleDeg(1,0,0,-28.0f),
                   18.0f, 700, 1200, a);
  }

  // Index bend -> left click.
  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_GREEN;
    a.mouse.type = MouseActionType::CLICK;
    a.mouse.button = MOUSE_LEFT;
    a.vibrate = true;
    a.vibrate_sensor_mask = (1u << SENSOR_INDEX);
    a.vibrate_duration_ms = 140;
    addPoseGesture("index_left_click", "MOUSE_LEFT_CLICK", "LCLK",
                   gag::Sensor::INDEX,
                   gag::Quaternion::fromAxisAngleDeg(1,0,0,-30.0f),
                   18.0f, 320, 900, a);
  }

  // Ring bend -> right click.
  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_MAGENTA;
    a.mouse.type = MouseActionType::CLICK;
    a.mouse.button = MOUSE_RIGHT;
    addPoseGesture("ring_right_click", "MOUSE_RIGHT_CLICK", "RCLK",
                   gag::Sensor::RING,
                   gag::Quaternion::fromAxisAngleDeg(1,0,0,-30.0f),
                   18.0f, 320, 900, a);
  }

  // Thumb mouse movement. These are intentionally simple and depend on the
  // software neutral offsets having zeroed the current thumb pose.
  {
    gag::GestureAction a; a.blink_visualization = true; a.blink_color565 = TFT_YELLOW;
    a.mouse.type = MouseActionType::MOVE; a.mouse.dx = +12; a.mouse.dy = 0;
    addPoseGesture("thumb_move_right", "MOUSE_MOVE_RIGHT", "MR",
                   gag::Sensor::THUMB,
                   gag::Quaternion::fromAxisAngleDeg(0,0,1,-22.0f),
                   16.0f, 180, 900, a);
  }
  {
    gag::GestureAction a; a.blink_visualization = true; a.blink_color565 = TFT_YELLOW;
    a.mouse.type = gag::MouseActionType::MOVE; a.mouse.dx = -12; a.mouse.dy = 0;
    addPoseGesture("thumb_move_left", "MOUSE_MOVE_LEFT", "ML",
                   gag::Sensor::THUMB,
                   gag::Quaternion::fromAxisAngleDeg(0,0,1,+22.0f),
                   16.0f, 180, 900, a);
  }
  {
    gag::GestureAction a; a.blink_visualization = true; a.blink_color565 = TFT_YELLOW;
    a.mouse.type = gag::MouseActionType::MOVE; a.mouse.dx = 0; a.mouse.dy = -12;
    addPoseGesture("thumb_move_up", "MOUSE_MOVE_UP", "MU",
                   gag::Sensor::THUMB,
                   gag::Quaternion::fromAxisAngleDeg(1,0,0,-22.0f),
                   16.0f, 180, 900, a);
  }
  {
    gag::GestureAction a; a.blink_visualization = true; a.blink_color565 = TFT_YELLOW;
    a.mouse.type = gag::MouseActionType::MOVE; a.mouse.dx = 0; a.mouse.dy = +12;
    addPoseGesture("thumb_move_down", "MOUSE_MOVE_DOWN", "MD",
                   gag::Sensor::THUMB,
                   gag::Quaternion::fromAxisAngleDeg(1,0,0,+22.0f),
                   16.0f, 180, 900, a);
  }
}

// =====================
// Gesture callback
// =====================
static void onGestureRecognized(const gag::RecognizedGesture& gr) {
  const char* label = (gr.label && gr.label[0]) ? gr.label : ((gr.name && gr.name[0]) ? gr.name : "GEST");
  char logLine[28];
  snprintf(logLine, sizeof(logLine), "%lu %s", (unsigned long)(millis() % 10000UL), label);
  if (!gr.action || gr.action->log_to_history) {
    g_viz.pushLog(logLine);
  }

  if (gr.action) {
    if (gr.action->switch_visualization_mode) {
      g_viz.nextMode();
    }
    if (gr.action->blink_visualization) {
      g_viz.flash(gr.sensor_mask, gr.action->blink_color565, 180);
    }
    if (gr.action->vibrate) {
      scheduleVibration(gr.action->vibrate_sensor_mask, gr.action->vibrate_duration_ms);
    }
    execMouseAction(gr.action->mouse);
  }
}

// =====================
// Recognizer feed
// =====================
static void feedRecognizerFromCurrentPose() {
  const uint32_t now = millis();
  for (uint8_t s = 0; s < SENSOR_COUNT_HAND; ++s) {
    gag::Quaternion qCorr = (s == SENSOR_WRIST)
      ? correctedLogicalWristQuaternion()
      : correctedQuaternionForPhysicalSensor(s);
    g_recognizer.processSample(mapToRecognizerSensor(s), qCorr, now);
  }

  // Wrist accel can be used later for acceleration-driven gestures.
  pcaSelect(ACTIVE_CHANNELS[SENSOR_WRIST]);
  uint8_t buf[6] = {0};
  i2cReadBytes(MPU9250_ADDR, REG_ACCEL_XOUT_H, buf, 6);
  int16_t ax = (int16_t)((buf[0]<<8) | buf[1]);
  int16_t ay = (int16_t)((buf[2]<<8) | buf[3]);
  int16_t az = (int16_t)((buf[4]<<8) | buf[5]);
  const gag::offsets::HwOffset6 hw = g_offsets.hardware(SENSOR_WRIST);
  gag::AccelData a((float)(ax - hw.ax) / 16384.0f,
                   (float)(ay - hw.ay) / 16384.0f,
                   (float)(az - hw.az) / 16384.0f);
  g_recognizer.processSample(gag::Sensor::WRIST, gag::RecogData::fromAccel(a), now);
}

// =====================
// Visualization input
// =====================
static gag::viz::FrameInput buildVizFrame() {
  gag::viz::FrameInput frame;
  frame.sensor_count = SENSOR_COUNT_ALL;
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    frame.base_color[i] = SENSOR_COLORS[i];
    frame.present[i] = true;
  }

  for (uint8_t s = 0; s < SENSOR_COUNT_HAND; ++s) {
    frame.sensor_q[s] = correctedQuaternionForPhysicalSensor(s);
  }

  frame.sensor_q[SENSOR_WRIST_AUX] = correctedQuaternionForPhysicalSensor(SENSOR_WRIST_AUX);
  frame.present[SENSOR_WRIST_AUX] = gy511Ok;
  frame.hand_wrist_q = correctedLogicalWristQuaternion();
  frame.hand_wrist_present = selectedWristQuaternionAvailable();
  frame.hand_wrist_color = selectedLogicalWristColor();

  return frame;
}

// =====================
// Setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(100);

  tft.init();
  tft.setRotation(GAG_TFT_ROTATION); // TFT_eSPI rotates the whole UI, including text primitives.
  tft.fillScreen(TFT_BLACK);
  g_viz.begin(tft, TFT_BLACK);
  g_viz.pushLog("BOOT");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  pcaReset();

  // Load default offsets.
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    g_offsets.setHardware(i, DEFAULT_HW_OFFSETS[i]);
    g_offsets.setSoftwareQuaternion(i, gag::Quaternion());
  }

  // Optional hardware offset measurement before IMU init.
  measureHardwareOffsetsAtBoot();

  for (uint8_t i = 0; i < NUM_ACTIVE_IMUS; ++i) {
    bool ok = initOneIMU(i);
    lastT[i] = millis();
    if (!ok) {
      Serial.printf("IMU init failed idx=%u ch=%u\n", i, ACTIVE_CHANNELS[i]);
    }
    delay(10);
  }

  wristMagOk = initWristMagAK8963();
  gy511Ok = initGY511();

  // Let filters settle.
  for (uint8_t warm = 0; warm < 20; ++warm) {
    for (uint8_t i = 0; i < NUM_ACTIVE_IMUS; ++i) updateOneIMU(i);
    updateWristMagYaw();
    updateGY511();
    delay(10);
  }

  autoCaptureSoftwareNeutralOffsets();

#if GAG_ENABLE_BLE_MOUSE
  g_bleMouse.begin();
#endif
  initMotors();

  g_recognizer.begin(Serial);
  g_recognizer.setOnRecognized(onGestureRecognized);
  installDefaultGestures();

  g_viz.pushLog("READY");
}

void loop() {
  for (uint8_t i = 0; i < NUM_ACTIVE_IMUS; ++i) {
    updateOneIMU(i);
  }
  updateWristMagYaw();
  updateGY511();

  feedRecognizerFromCurrentPose();
  updateVibrations();

  gag::viz::FrameInput frame = buildVizFrame();
  g_viz.draw(frame);

  delay(10);
}
