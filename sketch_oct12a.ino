#pragma once
/*
 * gesture_glove_ttgo_merged_fixed.ino (header implementation)
 *
 * Merged TTGO / ESP32 glove sketch.
 *
 * What this version keeps from the newer TTGO sketch:
 *   - ESP32 TTGO / T-Display target
 *   - PCA9548A/TCA9548A mux
 *   - wrist GY25 + wrist MPU9250 + 5 finger MPU6050-class sensors
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
 *   - sensor index 0..4 = thumb, index, middle, ring, little
 *   - sensor index 5 = wrist GY25
 *   - sensor index 6 = wrist MPU9250
 *   - sensor index 7 = wrist auxiliary GY-511
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

struct Vec3;

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
#include <BleMouse.h>
#define GAG_HAVE_BLE_MOUSE 1

#include "config.h"

#if (GAG_PRIMARY_WRIST_SENSOR != GAG_PRIMARY_WRIST_SENSOR_GY25) && (GAG_PRIMARY_WRIST_SENSOR != GAG_PRIMARY_WRIST_SENSOR_MPU9250) && (GAG_PRIMARY_WRIST_SENSOR != GAG_PRIMARY_WRIST_SENSOR_GY511)
#error "GAG_PRIMARY_WRIST_SENSOR must be GAG_PRIMARY_WRIST_SENSOR_GY25, GAG_PRIMARY_WRIST_SENSOR_MPU9250, or GAG_PRIMARY_WRIST_SENSOR_GY511"
#endif

// =====================
// Pins & PCA9548A / TCA9548A
// =====================
#define PIN_I2C_SDA 22
#define PIN_I2C_SCL 21
#define PIN_PCA_RST 33
#define PIN_PCA_A0 25
#define PIN_PCA_A1 26
#define PIN_PCA_A2 27
#define DRIVE_PCA_ADDR_PINS false
#define PCA9548A_BASE_ADDR 0x70
#define MPU9250_ADDR_DEFAULT 0x68

static uint8_t pca_addr = PCA9548A_BASE_ADDR;
static uint8_t g_wristMpuAddr = MPU9250_ADDR_DEFAULT;

static bool readTtgoRightButtonPressed() {
  const int raw = digitalRead(GAG_TTGO_RIGHT_BUTTON_PIN);
  return GAG_TTGO_BUTTON_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

static bool readTtgoLeftButtonPressed() {
  const int raw = digitalRead(GAG_PAIR_CONFIRM_BUTTON_PIN);
  return GAG_PAIR_CONFIRM_BUTTON_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}


// =====================
// Sensor topology
// =====================
enum SensorIndex : uint8_t {
  SENSOR_THUMB = 0,
  SENSOR_INDEX = 1,
  SENSOR_MIDDLE = 2,
  SENSOR_RING = 3,
  SENSOR_LITTLE = 4,
  SENSOR_WRIST_GY25 = 5,
  SENSOR_WRIST_MPU9250 = 6,
  SENSOR_WRIST_GY511 = 7,
  SENSOR_COUNT_ALL = 8,
  SENSOR_COUNT_FINGERS = 5,
  SENSOR_COUNT_HAND = 6,
};

static const uint8_t CH_GY25 = 5;
static const uint8_t CH_MPU9250 = 1;
static const uint8_t CH_GY511 = 2;

// Per-sensor PCA9548A channel map in logical sensor order:
// thumb, index, middle, ring, little, wrist GY25, wrist MPU9250, wrist GY-511.
static const uint8_t ACTIVE_CHANNELS[SENSOR_COUNT_ALL] = { 0, 3, 4, 6, 7, CH_GY25, CH_MPU9250, CH_GY511 };
// Enable only the sensors that are physically connected in the current glove.
static const bool SENSOR_ENABLED[SENSOR_COUNT_ALL] = {
  true,   // thumb
  true,   // index
  true,   // middle
  true,   // ring
  true,   // little
  true,   // wrist GY25
  true,   // wrist MPU9250
  true,   // wrist GY-511
};

static const uint8_t FINGER_MAP[5] = { SENSOR_THUMB, SENSOR_INDEX, SENSOR_MIDDLE, SENSOR_RING, SENSOR_LITTLE };

static inline uint8_t sensorBitMask(uint8_t sensorIdx) {
  return (sensorIdx < 8u) ? (uint8_t)(1u << sensorIdx) : 0u;
}

static inline bool isWristPhysicalSensor(uint8_t sensorIdx) {
  return sensorIdx == SENSOR_WRIST_GY25 || sensorIdx == SENSOR_WRIST_MPU9250 || sensorIdx == SENSOR_WRIST_GY511;
}

static inline uint8_t simultaneousDriftResetCompileTimeMask() {
  return (uint8_t)((GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_THUMB ? sensorBitMask(SENSOR_THUMB) : 0u) |
                   (GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_INDEX ? sensorBitMask(SENSOR_INDEX) : 0u) |
                   (GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_MIDDLE ? sensorBitMask(SENSOR_MIDDLE) : 0u) |
                   (GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_RING ? sensorBitMask(SENSOR_RING) : 0u) |
                   (GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_LITTLE ? sensorBitMask(SENSOR_LITTLE) : 0u) |
                   (GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_GY25 ? sensorBitMask(SENSOR_WRIST_GY25) : 0u) |
                   (GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_MPU9250 ? sensorBitMask(SENSOR_WRIST_MPU9250) : 0u) |
                   (GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_SENSOR_WRIST_GY511 ? sensorBitMask(SENSOR_WRIST_GY511) : 0u));
}

// =====================
// Minimal vector helpers
// =====================
struct Vec3 {
  float x, y, z;
};

// Choose which physical sensors emit corrected quaternions on Serial.
// Example:
//   sensorBitMask(SENSOR_THUMB) | sensorBitMask(SENSOR_INDEX) | sensorBitMask(SENSOR_WRIST_MPU9250)
static uint8_t g_serialQuatLogSensorMask = 0;
static uint32_t g_lastSerialQuatLogMs = 0;
static uint32_t g_lastMinorRotationOffsetPrintMs = 0;
static uint32_t g_rightButtonLastTriggerMs = 0;
static bool g_leftButtonPrevPressed = false;
static uint32_t g_leftButtonLastTriggerMs = 0;
static bool g_bleMouseSendEnabled = false;
static bool g_isMouseMoving = false;
static bool g_wristMouseEmulationEnabled = true;
static bool g_pendingLeftClick = false;
static uint32_t g_pendingLeftClickDueMs = 0;
static uint32_t g_ignoreSingleLeftClickUntilMs = 0;
static bool g_pendingWristMouseToggleSoftReset = false;
static uint32_t g_pendingWristMouseToggleSoftResetDueMs = 0;
static uint8_t g_pendingGestureSoftResetMask[GAG_MAX_PENDING_GESTURE_SOFT_RESETS] = { 0 };
static uint32_t g_pendingGestureSoftResetDueMs[GAG_MAX_PENDING_GESTURE_SOFT_RESETS] = { 0 };
static uint32_t g_softResetOperationIndex = 0;
static float g_thumbMouseFilteredDx = 0.0f;
static float g_thumbMouseFilteredDy = 0.0f;
static float g_thumbMouseResidualDx = 0.0f;
static float g_thumbMouseResidualDy = 0.0f;
static bool g_rightButtonPrevPressed = false;
static float g_thumbMouseVizDx = 0.0f;
static float g_thumbMouseVizDy = 0.0f;
static Vec3 g_lastAccelBody[SENSOR_COUNT_ALL];
static bool g_lastAccelBodyValid[SENSOR_COUNT_ALL] = { false };
static float g_wristGy25RuntimeBiasDegX = 0.0f;
static float g_wristGy25RuntimeBiasDegY = 0.0f;
static float g_wristGy25RuntimeBiasDegZ = 0.0f;
static uint32_t g_wristGy25StillSinceMs = 0;
static uint32_t g_lastWristGy25BiasLogMs = 0;
static uint32_t g_lastSoftSensorResetMs = 0;
static gag::Quaternion g_driftResetLastPhysicalFixed[SENSOR_COUNT_ALL];
static bool g_driftResetLastPhysicalFixedValid[SENSOR_COUNT_ALL] = { false };
static uint32_t g_driftResetStillSinceMs[SENSOR_COUNT_ALL] = { 0 };
static bool g_driftResetActive[SENSOR_COUNT_ALL] = { false };
static bool g_enableDriftReset[SENSOR_COUNT_ALL] = { true, true, true, true, true, true, true, true };
static uint32_t g_lastSimultaneousDriftResetMs = 0;
static bool g_printQuaternionsOnLeftClick = false;

static void syncDriftResetEnableState() {
  const uint32_t now = millis();
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    g_enableDriftReset[s] = !g_printQuaternionsOnLeftClick;
    if (!g_enableDriftReset[s]) {
      g_driftResetLastPhysicalFixedValid[s] = false;
      g_driftResetStillSinceMs[s] = now;
      g_driftResetActive[s] = false;
    }
  }
}

// =====================
// Optional vibration motors
// =====================
#if GAG_ENABLE_VIBRATION
// static const int8_t MOTOR_PINS[SENSOR_COUNT_ALL] = {2, 15, 13, 25, 26, 27, 17};
// static const int8_t MOTOR_PINS[SENSOR_COUNT_ALL] = {17, 2, 15, 13, 25, 26, 27};
static const int8_t MOTOR_PINS[SENSOR_COUNT_ALL] = { 15, 2, 17, 25, 13, 26, 27, -1 };
static const bool MOTOR_ACTIVE_HIGH = true;
struct MotorState {
  bool active = false;
  uint32_t until_ms = 0;
};
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
  TFT_YELLOW,   // thumb
  TFT_GREEN,    // index
  TFT_CYAN,     // middle
  TFT_MAGENTA,  // ring
  TFT_ORANGE,   // little
  TFT_RED,      // wrist GY25
  TFT_YELLOW,   // wrist MPU9250
  TFT_BLUE      // wrist GY-511
};

// =====================
// Recognition / actions
// =====================
static gag::Recognizer g_recognizer;
static gag::offsets::OffsetStore g_offsets;

#if GAG_ENABLE_BLE_MOUSE && defined(MASTER_HAND)
static BleMouse g_bleMouse("GAG Mouse", "GAG", 100);
#elif GAG_ENABLE_BLE_MOUSE
static BleMouse g_bleMouse("GAG Mouse", "GAG", 100);
#endif

static inline float deg2rad(float d) {
  return d * (float)M_PI / 180.0f;
}
static inline float rad2deg(float r) {
  return r * 180.0f / (float)M_PI;
}
static inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
static inline float deltaAngleDeg(float a, float b) {
  return wrap180(a - b);
}
static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}
static inline float vecDot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 vecCross(const Vec3& a, const Vec3& b) {
  Vec3 out{
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
  return out;
}
static inline float vecNorm(const Vec3& v) {
  return sqrtf(vecDot(v, v));
}
static inline Vec3 vecScale(const Vec3& v, float s) {
  Vec3 out{ v.x * s, v.y * s, v.z * s };
  return out;
}
static inline Vec3 vecSub(const Vec3& a, const Vec3& b) {
  Vec3 out{ a.x - b.x, a.y - b.y, a.z - b.z };
  return out;
}
static inline Vec3 vecNormalize(const Vec3& v) {
  const float n = vecNorm(v);
  if (n <= 1e-6f) return Vec3{ 0.0f, 0.0f, 0.0f };
  return vecScale(v, 1.0f / n);
}

// =====================
// IMU state
// =====================
MPU6050 mpu[SENSOR_COUNT_ALL];
uint8_t g_sensorMpuAddr[SENSOR_COUNT_ALL] = { 0 };
gag::Quaternion g_sensorFusionQuat[SENSOR_COUNT_ALL];
bool g_sensorFusionInitialized[SENSOR_COUNT_ALL] = { false };
bool g_sensorInitOk[SENSOR_COUNT_ALL] = { false };
unsigned long lastT[SENSOR_COUNT_ALL] = { 0 };
unsigned long g_lastFifoResetMs[SENSOR_COUNT_ALL] = { 0 };
const float alpha = 0.98f;
const float kTiltCorrectionGain = 1.0f - alpha;
const float kHeadingCorrectionGain = 0.08f;
const Vec3 kWorldUp{ 0.0f, 0.0f, 1.0f };
const Vec3 kWorldNorth{ 1.0f, 0.0f, 0.0f };

// Per-sensor fused-rotation deadband in deg/s. Small changes below the
// threshold are suppressed to hide residual jitter from imperfect calibration.
// Set an entry to 0 to disable the deadband for that sensor.
static const float SENSOR_ROTATION_IGNORE_DEGS[SENSOR_COUNT_ALL] = {
  9.0f,  // thumb
  9.0f,  // index
  9.0f,  // middle
  10.0f, // ring
  10.0f, // little
  6.0f,  // wrist GY25
  5.0f,  // wrist MPU9250
  7.0f   // wrist GY-511
};

bool wristMagOk = false;
Vec3 wristMagRaw{ 0, 0, 0 };

bool gy511Ok = true;
bool gy511MagOk = true;
Vec3 gy511Accel_g{ 0, 0, 0 };
Vec3 gy511MagRaw{ 0, 0, 0 };
unsigned long gy511LastT = 0;

// =====================
// Default offsets
// =====================
// Old 6-component layout: ax, ay, az, gx, gy, gz.
// Edit these after calibration. Left at 0 by default in the merged TTGO build.
// static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
//   { 0, 0, 0, 0, 0, 0 }, // wrist MPU9250
//   { 0, 0, 0, 0, 0, 0 }, // thumb
//   { 0, 0, 0, 0, 0, 0 }, // index
//   { 0, 0, 0, 0, 0, 0 }, // middle
//   { 0, 0, 0, 0, 0, 0 }, // ring
//   { 0, 0, 0, 0, 0, 0 }, // little
//   { 0, 0, 0, 0, 0, 0 }, // wrist aux (accel-only used in this sketch)
// };
// static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
//   { 0, 0, 0, 0, 0, 0 }, // wrist MPU9250
//   { -1583, -787, 1897, 49, -27, 22 }, // thumb
//   { 1645, -1569, 1669, 146, 16, 21 }, // index
//   { -1119, -613, 1101, 244, 92, 53 }, // middle
//   { 0, 0, 0, 0, 0, 0 }, // ring
//   { 0, 0, 0, 0, 0, 0 }, // little
//   { 270, 2051, 1915, 0, 0, 0 }, // wrist aux (accel-only used in this sketch)
// };

// static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
//   { 60, 171, -184, 1, 0, 0 }, // thumb
//   { -47, 80, -204, -2, 1, 1 }, // index
//   // { 110, 86, -137, 0, 4, 0 }, // middle
//   { -1275, -1079, 1221, 250, 92, 53 }, // middle
//   { 0, 0, 0, 0, 0, 0 }, // ring
//   { 0, 0, 0, 0, 0, 0 }, // little
//   { 297, 2305, 2130, 0, 0, 0 }, // wrist aux (accel-only used in this sketch)
//   // { 0, 0, 0, 0, 0, 0 }, // wrist MPU9250
//   { -2215, -141, 2588, -145, 35, -21 }, // wrist MPU9250
// };

// static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
//   { -1602, -847, 2079, 47, -26, 24 }, // thumb
//   { 1313, -1382, 1813, 142, 21, 20 }, // index
//   { -306, 222, -392, 2, 4, 0 }, // middle
//   { 0, 0, 0, 0, 0, 0 }, // ring
//   { 0, 0, 0, 0, 0, 0 }, // little
//   { 15, 2369, 2700, 0, 0, 0 }, // wrist aux (accel-only used in this sketch)
//   { -2211, -216, 2582, -144, 38, -20 }, // wrist MPU9250
// };

// static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
//   // { 138, 243, -392, 1, 0, 0 }, // thumb
//   // { -117, 252, -486, -1, 1, 1 }, // index
//   { -1602, -847, 2079, 47, -26, 24 },    // thumb
//   { 1313, -1382, 1813, 142, 21, 20 },    // index
//   { -1367, -1257, 1421, 250, 92, 52 },   // middle
//   // { 0, 0, 0, 0, 0, 0 },               // ring
//   // { -3495, -174, 1537, 42, 0, 0 },       // ring
//   { -3375, -1117, 1537, 182, 28, 23 },       // ring
//   // { 122, -1017, 1279, 141, 28, 23 }, // little
//   { -1357, -2650, 1810, 162, -59, 40 }, // wrist GY25
//   // { 0, 0, 0, 0, 0, 0 },                  // little
//   // { 0, 0, 0, 0, 0, 0 },                  // wrist GY25
//   // { -1285, -3177, 2002, 163, -59, -13 }, // wrist GY25
//   // { -145, 243, -380, -1, -1, 58 }, // wrist GY25
//   // { -1357, -3550, 2410, 162, -59, -13 }, // wrist GY25
//   // { -1357, -3550, 2410, 162, -59, -13 }, // wrist GY25
//     { -1357, -2650, 1810, 162, -59, 40 }, // wrist GY25
// // { 22, 921, -794, 0, 0, 57 }, // wrist GY25
//   // { 21, 919, -795, 0, 0, 57 }, // wrist GY25
//   // { -3, 345, -357, -1, -1, 57 }, // wrist GY25
//   { 2840, 4096, 6144, -144, 39, -22 },   // wrist MPU9250
//   { -24, 2376, 2781, 0, 0, 0 },          // wrist GY-511 (accel-only used in this sketch)
// };


static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
  { -1602, -847, 2079, 47, -26, 24 },     // thumb
  { 1313, -1382, 1813, 142, 21, 20 },     // index
  { -1367, -1257, 1421, 250, 92, 52 },    // middle
  { -3375, -1117, 1537, 182, 28, 23 },    // ring
  { 122, -1017, 1279, 141, 28, 23 },      // little
  { -1357, -2650, 1810, 162, -59, 40 },   // wrist GY25
  { 2840, 4096, 6144, -144, 39, -22 },    // wrist MPU9250
  { -24, 2376, 2781, 0, 0, 0 },           // wrist GY-511 (accel-only used in this sketch)
};

  // { 632, 1031, -210, -136, -27, -22 }, // ring
  // { 1593, 1947, -776, -20, 87, -16 }, // little
  // { 161, -674, -157, -1, 5, -14 }, // wrist GY25

// static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {
//   { 0, 0, 0, 0, 0, 0 }, // wrist MPU9250
//   { -1628, -840, 2091, 48, -27, 22 }, // thumb
//   { 1938, -1272, 1914, 146, 17, 20 }, // index
//   // { -1275, -1079, 1221, 250, 92, 53 }, // middle
//   { 110, 86, -137, 0, 4, 0 }, // middle
//   { 0, 0, 0, 0, 0, 0 }, // ring
//   { 0, 0, 0, 0, 0, 0 }, // little
//   { 493, 2309, 2129, 0, 0, 0 }, // wrist aux (accel-only used in this sketch)
// };

static const char* SENSOR_OFFSET_LABELS[SENSOR_COUNT_ALL] = {
  "thumb",
  "index",
  "middle",
  "ring",
  "little",
  "wrist GY25",
  "wrist MPU9250",
  "wrist GY-511 (accel-only used in this sketch)",
};

// Default per-sensor mounting compensation applied in the sensor's local/body
// frame before neutral offsets.
// Finger IMUs: +90 deg CCW around Z.
// Wrist GY25: -90 deg around Z.
// Wrist MPU9250: +180 deg around Z, then +90 deg around Y.
// Wrist GY-511: +90 deg CCW around Z, then -90 deg around X.
static const gag::Quaternion DEFAULT_SENSOR_ROTATION[SENSOR_COUNT_ALL] = {
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 180.0f),
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, 90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
};

static gag::Quaternion g_minorRotationOffset[SENSOR_COUNT_ALL] = {
  gag::Quaternion(0.96504295f, 0.02422791f, -0.23485672f, 0.11378706f),   // thumb
  gag::Quaternion(0.46711361f, -0.04733761f, 0.87993670f, 0.07263310f),   // index
  gag::Quaternion(0.87555826f, -0.05684654f, -0.47437924f, 0.07162798f),  // middle
  gag::Quaternion(0.42607555f, -0.74505758f, -0.24653897f, 0.45007503f), // ring
  gag::Quaternion(0.35652062f, -0.83403957f, -0.16701196f, 0.38649470f), // little
  gag::Quaternion(0.45700318f, -0.23116538f, 0.66169173f, -0.54760826f), // wrist GY25
  gag::Quaternion(0.30845252f, 0.77933824f, 0.27591035f, -0.47049168f),   // wrist MPU9250
  gag::Quaternion(0.10031156f, 0.00179863f, 0.99461555f, -0.02596957f),   // wrist GY-511
};

enum ExpectedSoftResetPoseIndex : uint8_t {
  EXPECTED_SOFT_RESET_POSE_DEFAULT = 0,
  EXPECTED_SOFT_RESET_POSE_WRIST_Z_CW_90,
  EXPECTED_SOFT_RESET_POSE_HAND_DOWN,
  EXPECTED_SOFT_RESET_POSE_COUNT,
};

static const char* const EXPECTED_SOFT_RESET_POSE_LABELS[EXPECTED_SOFT_RESET_POSE_COUNT] = {
  "default",
  "wrist_z_cw_90",
  "hand_down",
};

// Expected corrected orientations used by periodic soft reset.
// Pose 0 is the current DEFAULT_SENSOR_ROTATION + g_minorRotationOffset neutral,
// which corresponds to the identity quaternion in corrected space.
static const gag::Quaternion g_expectedSoftResetRotationDefault[SENSOR_COUNT_ALL] = {
  gag::Quaternion(), gag::Quaternion(), gag::Quaternion(), gag::Quaternion(),
  gag::Quaternion(), gag::Quaternion(), gag::Quaternion(), gag::Quaternion(),
};

// Pose 1: hand rotated 90 deg clockwise around the hand-local Z axis.
static const gag::Quaternion g_expectedSoftResetRotationWristZClockwise[SENSOR_COUNT_ALL] = {
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
  gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f),
};

// Pose 2: hand rotated down alongside the body: Y clockwise, then X clockwise.
static const gag::Quaternion g_expectedSoftResetRotationHandDown[SENSOR_COUNT_ALL] = {
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
  gag::Quaternion::mul(
    gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, -90.0f),
    gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, -90.0f)),
};

static const gag::Quaternion* const g_expectedSoftResetRotations[EXPECTED_SOFT_RESET_POSE_COUNT] = {
  g_expectedSoftResetRotationDefault,
  g_expectedSoftResetRotationWristZClockwise,
  g_expectedSoftResetRotationHandDown,
};

// =====================
// MPU9250 / AK8963 registers
// =====================
#define MPU6050_ADDR 0x68
#define MPU9250_ADDR_DEFAULT 0x68
#define MPU9250_ADDR_ALT 0x69
#define AK8963_ADDR 0x0C
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_FIFO_EN 0x23
#define REG_INT_PIN_CFG 0x37
#define REG_INT_STATUS 0x3A
#define REG_ACCEL_XOUT_H 0x3B
#define REG_USER_CTRL 0x6A
#define REG_PWR_MGMT_1 0x6B
#define REG_FIFO_COUNT_H 0x72
#define REG_FIFO_COUNT_L 0x73
#define REG_FIFO_R_W 0x74
#define REG_WHO_AM_I 0x75
#define AK8963_WHO_AM_I 0x00
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_ASAX 0x10

// =====================
// LSM303DLHC / GY-511
// =====================
#define LSM_ACC_ADDR 0x19
#define LSM_MAG_ADDR 0x1E
#define LSM_CTRL_REG1_A 0x20
#define LSM_CTRL_REG4_A 0x23
#define LSM_CTRL_REG5_A 0x24
#define LSM_OUT_X_L_A 0x28
#define LSM_FIFO_CTRL_REG_A 0x2E
#define LSM_FIFO_SRC_REG_A 0x2F
#define LSM_CRA_REG_M 0x00
#define LSM_CRB_REG_M 0x01
#define LSM_MR_REG_M 0x02
#define LSM_OUT_X_H_M 0x03

// =====================
// I2C helpers
// =====================
static void pcaReset() {
  if (DRIVE_PCA_ADDR_PINS) {
    pinMode(PIN_PCA_A0, OUTPUT);
    pinMode(PIN_PCA_A1, OUTPUT);
    pinMode(PIN_PCA_A2, OUTPUT);
    digitalWrite(PIN_PCA_A0, LOW);
    digitalWrite(PIN_PCA_A1, LOW);
    digitalWrite(PIN_PCA_A2, LOW);
  }
  pinMode(PIN_PCA_RST, OUTPUT);
  digitalWrite(PIN_PCA_RST, LOW);
  delay(2);
  digitalWrite(PIN_PCA_RST, HIGH);
  delay(2);
}

static void pcaSelect(uint8_t ch) {
  Wire.beginTransmission(pca_addr);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}

static void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t i2cReadByte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

static void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)len);
  for (uint8_t i = 0; i < len && Wire.available(); ++i) buf[i] = Wire.read();
}

static bool i2cAddressResponds(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void printWristMpuDiagnostic() {
  const uint8_t ch = ACTIVE_CHANNELS[SENSOR_WRIST_MPU9250];
  pcaSelect(ch);

  const bool ack68 = i2cAddressResponds(MPU9250_ADDR_DEFAULT);
  const bool ack69 = i2cAddressResponds(MPU9250_ADDR_ALT);
  const uint8_t who68 = i2cReadByte(MPU9250_ADDR_DEFAULT, REG_WHO_AM_I);
  const uint8_t who69 = i2cReadByte(MPU9250_ADDR_ALT, REG_WHO_AM_I);

  Serial.printf("Wrist MPU diagnostic: ch=%u ack68=%u who68=0x%02X ack69=%u who69=0x%02X\n",
                (unsigned)ch,
                ack68 ? 1u : 0u,
                (unsigned)who68,
                ack69 ? 1u : 0u,
                (unsigned)who69);

  uint8_t diagAddr = 0;
  if (who68 == 0x71 || who68 == 0x73 || ack68) diagAddr = MPU9250_ADDR_DEFAULT;
  else if (who69 == 0x71 || who69 == 0x73 || ack69) diagAddr = MPU9250_ADDR_ALT;

  if (diagAddr == 0) {
    Serial.println("  no response on 0x68 or 0x69");
    return;
  }

  i2cWriteByte(diagAddr, REG_PWR_MGMT_1, 0x00);
  delay(10);
  const uint8_t pwr = i2cReadByte(diagAddr, REG_PWR_MGMT_1);
  const uint8_t who = i2cReadByte(diagAddr, REG_WHO_AM_I);
  uint8_t buf[14] = { 0 };
  i2cReadBytes(diagAddr, REG_ACCEL_XOUT_H, buf, 14);
  const int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  const int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  const int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  const int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
  const int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
  const int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

  Serial.printf("  selected=0x%02X who=0x%02X pwr_mgmt_1=0x%02X\n",
                (unsigned)diagAddr,
                (unsigned)who,
                (unsigned)pwr);
  Serial.printf("  raw accel={%d,%d,%d} gyro={%d,%d,%d}\n",
                (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz);
}

static uint8_t detectMpuAddressForSensor(uint8_t sensorIdx) {
  if (sensorIdx >= SENSOR_COUNT_ALL || !isMpuBackedSensor(sensorIdx)) return 0;
  pcaSelect(ACTIVE_CHANNELS[sensorIdx]);
  if (sensorIdx == SENSOR_WRIST_MPU9250) {
    const uint8_t who68 = i2cReadByte(MPU9250_ADDR_DEFAULT, REG_WHO_AM_I);
    if (who68 == 0x71 || who68 == 0x73) return MPU9250_ADDR_DEFAULT;
    const uint8_t who69 = i2cReadByte(MPU9250_ADDR_ALT, REG_WHO_AM_I);
    if (who69 == 0x71 || who69 == 0x73) return MPU9250_ADDR_ALT;
    return 0;
  }
  if (i2cAddressResponds(MPU6050_ADDR)) return MPU6050_ADDR;
  if (i2cAddressResponds(MPU9250_ADDR_ALT)) return MPU9250_ADDR_ALT;
  return 0;
}

static inline bool usesDirectMpuRegisterAccess(uint8_t sensorIdx) {
  return sensorIdx == SENSOR_WRIST_MPU9250;
}

static void updateWristMpuAddress() {
  const uint8_t detected = detectMpuAddressForSensor(SENSOR_WRIST_MPU9250);
  if (detected != 0) g_wristMpuAddr = detected;
}

static uint8_t wristMpuAddress() {
  return g_wristMpuAddr;
}

static uint8_t mpuAddressForSensor(uint8_t sensorIdx) {
  if (sensorIdx == SENSOR_WRIST_MPU9250) return wristMpuAddress();
  if (sensorIdx < SENSOR_COUNT_ALL && g_sensorMpuAddr[sensorIdx] != 0) return g_sensorMpuAddr[sensorIdx];
  return MPU6050_ADDR;
}

static bool sensorCanUseRotationFifo(uint8_t sensorIdx) {
  if (sensorIdx >= SENSOR_COUNT_ALL || sensorIdx == SENSOR_WRIST_GY511 || !SENSOR_ENABLED[sensorIdx]) return false;
  if (sensorIdx == SENSOR_WRIST_MPU9250) return GAG_ENABLE_MPU9250_FIFO;
  return true;
}

static const uint8_t MPU_FIFO_PACKET_SIZE_BYTES = 12u;
static const uint8_t MPU_FIFO_ACCEL_GYRO_ENABLE_MASK = 0x78u;
static const uint8_t MPU_USER_CTRL_FIFO_EN = 0x40u;
static const uint8_t MPU_USER_CTRL_FIFO_RESET = 0x04u;
static const uint8_t MPU_INT_STATUS_FIFO_OVERFLOW = 0x10u;
static const uint8_t MPU_FIFO_CONFIG_DLPF = (uint8_t)GAG_MPU_FIFO_DLPF_CFG;
static const uint8_t MPU_FIFO_SMPLRT_DIV = (uint8_t)GAG_MPU_FIFO_SMPLRT_DIV;
static const uint8_t MPU_PWR_CLKSEL_ZGYRO = 0x03u;

static uint16_t fifoMaxBytesForSensor(uint8_t sensorIdx) {
  return (sensorIdx == SENSOR_WRIST_MPU9250) ? (uint16_t)GAG_MPU9250_FIFO_MAX_BYTES : (uint16_t)GAG_MPU6050_FIFO_MAX_BYTES;
}

static uint16_t fifoResetThresholdBytesForSensor(uint8_t sensorIdx) {
  const uint16_t maxBytes = fifoMaxBytesForSensor(sensorIdx);
  const uint16_t guardBytes = (uint16_t)(MPU_FIFO_PACKET_SIZE_BYTES * 2u);
  return (maxBytes > guardBytes) ? (uint16_t)(maxBytes - guardBytes) : maxBytes;
}

static uint16_t readMpuFifoCountBytes(uint8_t sensorIdx) {
  if (!sensorCanUseRotationFifo(sensorIdx)) return 0;
  if (!g_sensorInitOk[sensorIdx]) return 0;
  pcaSelect(ACTIVE_CHANNELS[sensorIdx]);
  const uint8_t addr = mpuAddressForSensor(sensorIdx);
  return (uint16_t)(((uint16_t)i2cReadByte(addr, REG_FIFO_COUNT_H) << 8) | i2cReadByte(addr, REG_FIFO_COUNT_L));
}

static void resetMpuFifo(uint8_t sensorIdx) {
#if GAG_ENABLE_MPU_FIFO
  if (!sensorCanUseRotationFifo(sensorIdx)) return;
  if (!g_sensorInitOk[sensorIdx]) return;
  pcaSelect(ACTIVE_CHANNELS[sensorIdx]);
  const uint8_t addr = mpuAddressForSensor(sensorIdx);
  i2cWriteByte(addr, REG_FIFO_EN, 0x00);
  i2cWriteByte(addr, REG_USER_CTRL, MPU_USER_CTRL_FIFO_RESET);
  delay(2);
  (void)i2cReadByte(addr, REG_INT_STATUS);
  i2cWriteByte(addr, REG_FIFO_EN, MPU_FIFO_ACCEL_GYRO_ENABLE_MASK);
  i2cWriteByte(addr, REG_USER_CTRL, MPU_USER_CTRL_FIFO_EN);
  g_lastFifoResetMs[sensorIdx] = millis();
#else
  (void)sensorIdx;
#endif
}

static void configureMpuFifo(uint8_t sensorIdx) {
#if GAG_ENABLE_MPU_FIFO
  if (!sensorCanUseRotationFifo(sensorIdx)) return;
  pcaSelect(ACTIVE_CHANNELS[sensorIdx]);
  const uint8_t addr = mpuAddressForSensor(sensorIdx);
  i2cWriteByte(addr, REG_USER_CTRL, 0x00);
  i2cWriteByte(addr, REG_FIFO_EN, 0x00);
  i2cWriteByte(addr, REG_PWR_MGMT_1, MPU_PWR_CLKSEL_ZGYRO);
  i2cWriteByte(addr, REG_CONFIG, MPU_FIFO_CONFIG_DLPF);
  i2cWriteByte(addr, REG_SMPLRT_DIV, MPU_FIFO_SMPLRT_DIV);
  i2cWriteByte(addr, REG_USER_CTRL, MPU_USER_CTRL_FIFO_RESET);
  delay(2);
  (void)i2cReadByte(addr, REG_INT_STATUS);
  i2cWriteByte(addr, REG_FIFO_EN, MPU_FIFO_ACCEL_GYRO_ENABLE_MASK);
  i2cWriteByte(addr, REG_USER_CTRL, MPU_USER_CTRL_FIFO_EN);
  g_lastFifoResetMs[sensorIdx] = millis();
#else
  (void)sensorIdx;
#endif
}

static bool readMpuFifoMotion6(uint8_t sensorIdx, int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
#if GAG_ENABLE_MPU_FIFO
  if (!sensorCanUseRotationFifo(sensorIdx)) return false;
  if (!g_sensorInitOk[sensorIdx]) return false;
  pcaSelect(ACTIVE_CHANNELS[sensorIdx]);
  const uint8_t addr = mpuAddressForSensor(sensorIdx);
  const uint8_t intStatus = i2cReadByte(addr, REG_INT_STATUS);
  const uint16_t fifoCount = readMpuFifoCountBytes(sensorIdx);
  if ((intStatus & MPU_INT_STATUS_FIFO_OVERFLOW) != 0u || fifoCount > fifoMaxBytesForSensor(sensorIdx)) {
    resetMpuFifo(sensorIdx);
    return false;
  }
  if (fifoCount < MPU_FIFO_PACKET_SIZE_BYTES) return false;
  if ((fifoCount % MPU_FIFO_PACKET_SIZE_BYTES) != 0u) {
    resetMpuFifo(sensorIdx);
    return false;
  }

  uint8_t packet[MPU_FIFO_PACKET_SIZE_BYTES] = { 0 };
  uint16_t packetsRemaining = (uint16_t)(fifoCount / MPU_FIFO_PACKET_SIZE_BYTES);
  while (packetsRemaining > 1u) {
    i2cReadBytes(addr, REG_FIFO_R_W, packet, MPU_FIFO_PACKET_SIZE_BYTES);
    --packetsRemaining;
  }
  i2cReadBytes(addr, REG_FIFO_R_W, packet, MPU_FIFO_PACKET_SIZE_BYTES);

  ax = (int16_t)((packet[0] << 8) | packet[1]);
  ay = (int16_t)((packet[2] << 8) | packet[3]);
  az = (int16_t)((packet[4] << 8) | packet[5]);
  gx = (int16_t)((packet[6] << 8) | packet[7]);
  gy = (int16_t)((packet[8] << 8) | packet[9]);
  gz = (int16_t)((packet[10] << 8) | packet[11]);
  return true;
#else
  (void)sensorIdx;
  (void)ax;
  (void)ay;
  (void)az;
  (void)gx;
  (void)gy;
  (void)gz;
  return false;
#endif
}

static void maybeResetMpuFifo(uint8_t sensorIdx) {
#if GAG_ENABLE_MPU_FIFO
  if (!sensorCanUseRotationFifo(sensorIdx)) return;
  if (!g_sensorInitOk[sensorIdx]) return;
  const uint16_t fifoCount = readMpuFifoCountBytes(sensorIdx);
  const uint32_t now = millis();
  const bool fifoNearOverflow = (fifoCount >= fifoResetThresholdBytesForSensor(sensorIdx));
  const bool fifoStalled = (fifoCount >= (uint16_t)(MPU_FIFO_PACKET_SIZE_BYTES * 8u)) && ((uint32_t)(now - g_lastFifoResetMs[sensorIdx]) >= (uint32_t)GAG_FIFO_RESET_INTERVAL_MS);
  if (fifoNearOverflow || fifoStalled) {
    resetMpuFifo(sensorIdx);
  }
#else
  (void)sensorIdx;
#endif
}

static void printMpuFifoBootTestForSensor(uint8_t sensorIdx) {
#if GAG_ENABLE_FIFO_BOOT_TEST
  if (!sensorCanUseRotationFifo(sensorIdx)) return;
  if (!g_sensorInitOk[sensorIdx]) {
    Serial.printf("FIFO boot test sensor=%u label=%s skipped init_failed\n",
                  (unsigned)sensorIdx,
                  SENSOR_OFFSET_LABELS[sensorIdx]);
    return;
  }
  delay(50);
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  const bool ok = readMpuFifoMotion6(sensorIdx, ax, ay, az, gx, gy, gz);
  Serial.printf("FIFO boot test sensor=%u label=%s count=%u max=%u reset_at=%u ok=%u sample={%d,%d,%d,%d,%d,%d}\n",
                (unsigned)sensorIdx,
                SENSOR_OFFSET_LABELS[sensorIdx],
                (unsigned)readMpuFifoCountBytes(sensorIdx),
                (unsigned)fifoMaxBytesForSensor(sensorIdx),
                (unsigned)fifoResetThresholdBytesForSensor(sensorIdx),
                ok ? 1u : 0u,
                (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz);
#else
  (void)sensorIdx;
#endif
}

static void printFifoCapabilityReportForSensor(uint8_t sensorIdx) {
  if (!isSensorEnabled(sensorIdx)) return;
  if (sensorIdx != SENSOR_WRIST_GY511 && !g_sensorInitOk[sensorIdx]) {
    Serial.printf("FIFO sensor=%u label=%s skipped init_failed\n",
                  (unsigned)sensorIdx,
                  SENSOR_OFFSET_LABELS[sensorIdx]);
    return;
  }

  if (sensorIdx == SENSOR_WRIST_GY511) {
    pcaSelect(CH_GY511);
    const uint8_t magWho = i2cReadByte(LSM_MAG_ADDR, 0x0A);
    const uint8_t ctrl5a = i2cReadByte(LSM_ACC_ADDR, LSM_CTRL_REG5_A);
    const uint8_t fifoCtrl = i2cReadByte(LSM_ACC_ADDR, LSM_FIFO_CTRL_REG_A);
    const uint8_t fifoSrc = i2cReadByte(LSM_ACC_ADDR, LSM_FIFO_SRC_REG_A);
    const bool fifoEnabled = (ctrl5a & 0x40u) != 0;
    const uint8_t fifoMode = (uint8_t)((fifoCtrl >> 6) & 0x03u);
    const uint8_t fifoSamples = (uint8_t)(fifoSrc & 0x1Fu);

    Serial.printf("FIFO sensor=%u label=%s type=GY-511/LSM303DLHC accel+mag mag_who=0x%02X\n",
                  (unsigned)sensorIdx, SENSOR_OFFSET_LABELS[sensorIdx], (unsigned)magWho);
    Serial.println("  accel_fifo_support=yes");
    Serial.println("  mag_fifo_support=no");
    Serial.printf("  accel_fifo_enabled=%s ctrl5_a=0x%02X fifo_ctrl_a=0x%02X fifo_src_a=0x%02X mode=%u samples=%u\n",
                  fifoEnabled ? "yes" : "no",
                  (unsigned)ctrl5a,
                  (unsigned)fifoCtrl,
                  (unsigned)fifoSrc,
                  (unsigned)fifoMode,
                  (unsigned)fifoSamples);
    Serial.println("  firmware_uses_fifo=no direct register polling");
    return;
  }

  const uint8_t ch = ACTIVE_CHANNELS[sensorIdx];
  pcaSelect(ch);
  const uint8_t mpuAddr = (sensorIdx == SENSOR_WRIST_MPU9250) ? wristMpuAddress() : MPU6050_ADDR;
  const uint8_t who = i2cReadByte(mpuAddr, REG_WHO_AM_I);
  const uint8_t fifoEn = i2cReadByte(mpuAddr, REG_FIFO_EN);
  const uint8_t userCtrl = i2cReadByte(mpuAddr, REG_USER_CTRL);
  const uint8_t fifoCountH = i2cReadByte(mpuAddr, REG_FIFO_COUNT_H);
  const uint8_t fifoCountL = i2cReadByte(mpuAddr, REG_FIFO_COUNT_L);
  const uint16_t fifoCount = (uint16_t)(((uint16_t)fifoCountH << 8) | fifoCountL);
  const bool fifoEnabled = (userCtrl & 0x40u) != 0;

  const char* sensorType = (sensorIdx == SENSOR_WRIST_GY25)
                             ? "GY25 / MPU6050-class wrist IMU"
                             : ((sensorIdx == SENSOR_WRIST_MPU9250)
                                  ? "MPU9250-class wrist IMU"
                                  : "MPU6050-class finger IMU");
  Serial.printf("FIFO sensor=%u label=%s type=%s addr=0x%02X who_am_i=0x%02X channel=%u\n",
                (unsigned)sensorIdx,
                SENSOR_OFFSET_LABELS[sensorIdx],
                sensorType,
                (unsigned)mpuAddr,
                (unsigned)who,
                (unsigned)ch);
  Serial.println("  fifo_support=yes");
  Serial.printf("  fifo_enabled=%s user_ctrl=0x%02X fifo_en=0x%02X fifo_count=%u\n",
                fifoEnabled ? "yes" : "no",
                (unsigned)userCtrl,
                (unsigned)fifoEn,
                (unsigned)fifoCount);
  Serial.println("  firmware_uses_fifo=no direct accel/gyro reads");
}

static void printFifoCapabilityReport() {
  Serial.println("FIFO capability report:");
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    printFifoCapabilityReportForSensor(s);
  }
}

// =====================
// Magnetometer helpers
// =====================
static float yawFromMagTiltComp(const Vec3& m, float rollDeg, float pitchDeg) {
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
static const uint8_t STARTUP_VIBRATION_TEST_SENSORS[] = { SENSOR_THUMB, SENSOR_INDEX, SENSOR_MIDDLE };

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

static void stopAllVibrations() {
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    g_motorState[i].active = false;
    g_motorState[i].until_ms = 0;
    setMotorOutput(i, false);
  }
}

static void scheduleVibration(uint8_t sensorMask, uint16_t durationMs) {
#if GAG_DISABLE_VIBRATION_WHEN_BLE_SEND_OFF
  if (!g_bleMouseSendEnabled) return;
#endif
  const uint32_t until = millis() + durationMs;
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    if (!(sensorMask & (1u << i))) continue;
    if (g_motorState[i].active) continue;
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

static void runStartupVibrationTest() {
  for (uint8_t i = 0; i < sizeof(STARTUP_VIBRATION_TEST_SENSORS); ++i) {
    const uint8_t sensorIdx = STARTUP_VIBRATION_TEST_SENSORS[i];
    if (!isSensorEnabled(sensorIdx)) continue;
    scheduleVibration((uint8_t)(1u << sensorIdx), 500);
    while (g_motorState[sensorIdx].active) {
      updateVibrations();
      delay(10);
    }
    delay(80);
  }

  uint8_t allMask = 0;
  for (uint8_t i = 0; i < sizeof(STARTUP_VIBRATION_TEST_SENSORS); ++i) {
    const uint8_t sensorIdx = STARTUP_VIBRATION_TEST_SENSORS[i];
    if (!isSensorEnabled(sensorIdx)) continue;
    allMask |= (uint8_t)(1u << sensorIdx);
  }

  if (allMask != 0) {
    scheduleVibration(allMask, 1000);
    while (true) {
      bool anyActive = false;
      updateVibrations();
      for (uint8_t i = 0; i < sizeof(STARTUP_VIBRATION_TEST_SENSORS); ++i) {
        const uint8_t sensorIdx = STARTUP_VIBRATION_TEST_SENSORS[i];
        if (g_motorState[sensorIdx].active) {
          anyActive = true;
          break;
        }
      }
      if (!anyActive) break;
      delay(10);
    }
  }
}
#else
static void initMotors() {}
static void stopAllVibrations() {}
static void scheduleVibration(uint8_t, uint16_t) {}
static void updateVibrations() {}
static void runStartupVibrationTest() {}
#endif

// =====================
// BLE mouse helpers
// =====================
static void execMouseAction(const gag::MouseAction& mouse) {
#if GAG_ENABLE_BLE_MOUSE
  if (!g_bleMouseSendEnabled) return;
  if (!g_bleMouse.isConnected()) return;
  if (g_isMouseMoving) {
    switch (mouse.type) {
      case gag::MouseActionType::CLICK:
      case gag::MouseActionType::DOUBLE_CLICK:
      case gag::MouseActionType::PRESS:
      case gag::MouseActionType::RELEASE:
        return;
      default:
        break;
    }
  }
  switch (mouse.type) {
    case gag::MouseActionType::MOVE:
      g_bleMouse.move(mouse.dx, mouse.dy, 0, 0);
      break;
    case gag::MouseActionType::CLICK:
      g_bleMouse.click(mouse.button);
      break;
    case gag::MouseActionType::DOUBLE_CLICK:
      g_bleMouse.click(mouse.button);
      delay(25);
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

static bool physicalSensorQuaternionAvailable(uint8_t sensorIdx);
static gag::Quaternion correctedQuaternionForPhysicalSensor(uint8_t sensorIdx);
static gag::Quaternion rawQuaternionForPhysicalSensor(uint8_t sensorIdx);
static uint8_t selectedWristQuaternionPhysicalSensor();
static bool selectedWristQuaternionAvailable();
static gag::Quaternion correctedLogicalWristQuaternion();
static Vec3 rotateVectorByQuat(const gag::Quaternion& qIn, const Vec3& v);

static void computeCircularThumbMouseTarget(float rawDxDeg,
                                          float rawDyDeg,
                                          float& targetDx,
                                          float& targetDy,
                                          float& maxStepPerLoop) {
  const float deadzoneRadiusDeg = 6.0f;
  const float fastRingRadiusDeg = 18.0f;
  const float fullScaleRadiusDeg = 24.0f;
  const float normalMaxStepPerLoop = 16.0f;
  const float fastMaxStepPerLoop = 64.0f;

  const float magnitudeDeg = sqrtf(rawDxDeg * rawDxDeg + rawDyDeg * rawDyDeg);
  if (magnitudeDeg <= deadzoneRadiusDeg) {
    targetDx = 0.0f;
    targetDy = 0.0f;
    maxStepPerLoop = normalMaxStepPerLoop;
    return;
  }

  const float clampedMagnitudeDeg = magnitudeDeg > fullScaleRadiusDeg ? fullScaleRadiusDeg : magnitudeDeg;
  const float dirX = rawDxDeg / magnitudeDeg;
  const float dirY = rawDyDeg / magnitudeDeg;
  float t = (clampedMagnitudeDeg - deadzoneRadiusDeg) / (fullScaleRadiusDeg - deadzoneRadiusDeg);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  const float curvedMagnitude = t * t * t;

  targetDx = dirX * curvedMagnitude;
  targetDy = dirY * curvedMagnitude;
  maxStepPerLoop = (clampedMagnitudeDeg >= fastRingRadiusDeg)
                     ? fastMaxStepPerLoop
                     : normalMaxStepPerLoop;
}

static void quaternionToThumbControlEulerDeg(const gag::Quaternion& qIn, float& xDeg, float& zDeg) {
  gag::Quaternion q = qIn;
  q.normalizeInPlace();

  // Use the quaternion's direct x/z channels instead of Euler roll/yaw.
  // The old discrete thumb gestures were defined as pure X-axis and Z-axis
  // rotations around the neutral thumb pose, so this keeps the continuous
  // mapping aligned with that same control frame and avoids cross-axis mixing.
  const float qx = q.x < -1.0f ? -1.0f : (q.x > 1.0f ? 1.0f : q.x);
  const float qz = q.z < -1.0f ? -1.0f : (q.z > 1.0f ? 1.0f : q.z);
  xDeg = rad2deg(2.0f * asinf(qx));
  zDeg = rad2deg(2.0f * asinf(qz));
}

static void quaternionToWristControlEulerDeg(const gag::Quaternion& qIn, float& xDeg, float& yDeg) {
  gag::Quaternion q = qIn;
  q.normalizeInPlace();

  const float qx = q.x < -1.0f ? -1.0f : (q.x > 1.0f ? 1.0f : q.x);
  const float qy = q.y < -1.0f ? -1.0f : (q.y > 1.0f ? 1.0f : q.y);
  xDeg = rad2deg(2.0f * asinf(qx));
  yDeg = rad2deg(2.0f * asinf(qy));
}

static bool imuOnlyMouseAccelAvailable(uint8_t sensorIdx) {
  return sensorIdx < SENSOR_COUNT_ALL
      && g_sensorInitOk[sensorIdx]
      && g_sensorFusionInitialized[sensorIdx]
      && g_lastAccelBodyValid[sensorIdx];
}

static bool computeImuOnlyMouseLinearAccelWorld(Vec3& out) {
  const uint8_t wristSensors[] = { SENSOR_WRIST_GY25, SENSOR_WRIST_MPU9250, SENSOR_WRIST_GY511 };
  Vec3 sum{ 0.0f, 0.0f, 0.0f };
  uint8_t count = 0;

  for (uint8_t i = 0; i < sizeof(wristSensors); ++i) {
    const uint8_t sensorIdx = wristSensors[i];
    if (!imuOnlyMouseAccelAvailable(sensorIdx)) continue;
    const Vec3 accelWorld = rotateVectorByQuat(rawQuaternionForPhysicalSensor(sensorIdx), g_lastAccelBody[sensorIdx]);
    const Vec3 linearWorld = vecSub(accelWorld, kWorldUp);
    sum.x += linearWorld.x;
    sum.y += linearWorld.y;
    sum.z += linearWorld.z;
    ++count;
  }

  if (count == 0u) {
    out = Vec3{ 0.0f, 0.0f, 0.0f };
    return false;
  }

  out = vecScale(sum, 1.0f / (float)count);
  return true;
}

static void resetContinuousThumbMouseControl() {
  g_thumbMouseFilteredDx = 0.0f;
  g_thumbMouseFilteredDy = 0.0f;
  g_thumbMouseResidualDx = 0.0f;
  g_thumbMouseResidualDy = 0.0f;
  g_thumbMouseVizDx = 0.0f;
  g_thumbMouseVizDy = 0.0f;
  g_isMouseMoving = false;
}

static void clearPendingLeftClick() {
  g_pendingLeftClick = false;
  g_pendingLeftClickDueMs = 0;
}

static void queuePendingLeftClick(uint32_t dueMs) {
  g_pendingLeftClick = true;
  g_pendingLeftClickDueMs = dueMs;
}

static void processPendingLeftClick() {
  if (!g_pendingLeftClick) return;
  const uint32_t now = millis();
  if ((int32_t)(now - g_pendingLeftClickDueMs) < 0) return;
  clearPendingLeftClick();
  gag::MouseAction mouse;
  mouse.type = gag::MouseActionType::CLICK;
  mouse.button = MOUSE_LEFT;
  execMouseAction(mouse);
}

static void updateContinuousThumbMouseControl() {
#if GAG_ENABLE_BLE_MOUSE
#if GAG_ENABLE_WRIST_MOUSE_EMULATION
  if (!g_wristMouseEmulationEnabled || !selectedWristQuaternionAvailable()) {
    resetContinuousThumbMouseControl();
    return;
  }

  float wristXDeg = 0.0f;
  float wristYDeg = 0.0f;
  quaternionToWristControlEulerDeg(correctedLogicalWristQuaternion(), wristXDeg, wristYDeg);

  const float rawTargetDxDeg = wristYDeg;
  const float rawTargetDyDeg = wristXDeg;
  float targetDx = 0.0f;
  float targetDy = 0.0f;
  float maxStepPerLoop = 20.0f;
  const float filterAlpha = 0.28f;

  computeCircularThumbMouseTarget(rawTargetDxDeg,
                                  rawTargetDyDeg,
                                  targetDx,
                                  targetDy,
                                  maxStepPerLoop);

  g_thumbMouseFilteredDx += (targetDx - g_thumbMouseFilteredDx) * filterAlpha;
  g_thumbMouseFilteredDy += (targetDy - g_thumbMouseFilteredDy) * filterAlpha;
  g_thumbMouseVizDx = -g_thumbMouseFilteredDx * maxStepPerLoop;
  g_thumbMouseVizDy = -g_thumbMouseFilteredDy * maxStepPerLoop;

  const float dxFloat = g_thumbMouseFilteredDx * maxStepPerLoop + g_thumbMouseResidualDx;
  const float dyFloat = g_thumbMouseFilteredDy * maxStepPerLoop + g_thumbMouseResidualDy;
  const int8_t dx = (int8_t)dxFloat;
  const int8_t dy = (int8_t)dyFloat;
  g_thumbMouseResidualDx = dxFloat - (float)dx;
  g_thumbMouseResidualDy = dyFloat - (float)dy;
  g_isMouseMoving = (dx != 0 || dy != 0 || fabsf(g_thumbMouseVizDx) > 0.25f || fabsf(g_thumbMouseVizDy) > 0.25f);

  if (g_bleMouseSendEnabled && g_bleMouse.isConnected() && (dx != 0 || dy != 0)) {
    g_bleMouse.move(-dx, -dy, 0, 0);
  }
#elif GAG_ENABLE_IMU_ONLY_MOUSE
  Vec3 linearWorld;
  if (!computeImuOnlyMouseLinearAccelWorld(linearWorld)) {
    resetContinuousThumbMouseControl();
    return;
  }

  const float rawTargetDxAccel = linearWorld.x;
  const float rawTargetDyAccel = linearWorld.y;
  const float deadzoneAccelG = 0.015f;
  const float fastBandAccelG = 0.06f;
  const float fullScaleAccelG = 0.14f;
  const float normalMaxStepPerLoop = 20.0f;
  const float fastMaxStepPerLoop = 52.0f;
  const float filterAlpha = 0.42f;

  float targetDx = 0.0f;
  float targetDy = 0.0f;
  float maxStepPerLoop = normalMaxStepPerLoop;
  const float magnitudeAccelG = sqrtf(rawTargetDxAccel * rawTargetDxAccel + rawTargetDyAccel * rawTargetDyAccel);
  if (magnitudeAccelG > deadzoneAccelG) {
    const float clampedMagnitudeAccelG = magnitudeAccelG > fullScaleAccelG ? fullScaleAccelG : magnitudeAccelG;
    const float dirX = rawTargetDxAccel / magnitudeAccelG;
    const float dirY = rawTargetDyAccel / magnitudeAccelG;
    float t = (clampedMagnitudeAccelG - deadzoneAccelG) / (fullScaleAccelG - deadzoneAccelG);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const float curvedMagnitude = t * t * t;
    targetDx = dirX * curvedMagnitude;
    targetDy = dirY * curvedMagnitude;
    if (clampedMagnitudeAccelG >= fastBandAccelG) maxStepPerLoop = fastMaxStepPerLoop;
  }

  g_thumbMouseFilteredDx += (targetDx - g_thumbMouseFilteredDx) * filterAlpha;
  g_thumbMouseFilteredDy += (targetDy - g_thumbMouseFilteredDy) * filterAlpha;
  if (targetDx == 0.0f) g_thumbMouseFilteredDx *= 0.82f;
  if (targetDy == 0.0f) g_thumbMouseFilteredDy *= 0.82f;
  g_thumbMouseVizDx = g_thumbMouseFilteredDx * maxStepPerLoop;
  g_thumbMouseVizDy = g_thumbMouseFilteredDy * maxStepPerLoop;

  const float dxFloat = g_thumbMouseFilteredDx * maxStepPerLoop + g_thumbMouseResidualDx;
  const float dyFloat = g_thumbMouseFilteredDy * maxStepPerLoop + g_thumbMouseResidualDy;
  const int8_t dx = (int8_t)dxFloat;
  const int8_t dy = (int8_t)dyFloat;
  g_thumbMouseResidualDx = dxFloat - (float)dx;
  g_thumbMouseResidualDy = dyFloat - (float)dy;
  g_isMouseMoving = (dx != 0 || dy != 0 || fabsf(g_thumbMouseVizDx) > 0.25f || fabsf(g_thumbMouseVizDy) > 0.25f);

  if (g_bleMouseSendEnabled && g_bleMouse.isConnected() && (dx != 0 || dy != 0)) {
    g_bleMouse.move(dx, dy, 0, 0);
  }
#elif GAG_ENABLE_THUMB_MOUSE_EMULATION
  if (!physicalSensorQuaternionAvailable(SENSOR_THUMB)) {
    resetContinuousThumbMouseControl();
    return;
  }

  float thumbXDeg = 0.0f;
  float thumbZDeg = 0.0f;
  quaternionToThumbControlEulerDeg(correctedQuaternionForPhysicalSensor(SENSOR_THUMB), thumbXDeg, thumbZDeg);

  const float rawTargetDxDeg = -thumbZDeg;
  const float rawTargetDyDeg = thumbXDeg;
  float targetDx = 0.0f;
  float targetDy = 0.0f;
  float maxStepPerLoop = 16.0f;
  const float filterAlpha = 0.22f;

  computeCircularThumbMouseTarget(rawTargetDxDeg,
                                  rawTargetDyDeg,
                                  targetDx,
                                  targetDy,
                                  maxStepPerLoop);

  g_thumbMouseFilteredDx += (targetDx - g_thumbMouseFilteredDx) * filterAlpha;
  g_thumbMouseFilteredDy += (targetDy - g_thumbMouseFilteredDy) * filterAlpha;
  g_thumbMouseVizDx = g_thumbMouseFilteredDx * maxStepPerLoop;
  g_thumbMouseVizDy = g_thumbMouseFilteredDy * maxStepPerLoop;

  const float dxFloat = g_thumbMouseFilteredDx * maxStepPerLoop + g_thumbMouseResidualDx;
  const float dyFloat = g_thumbMouseFilteredDy * maxStepPerLoop + g_thumbMouseResidualDy;
  const int8_t dx = (int8_t)dxFloat;
  const int8_t dy = (int8_t)dyFloat;
  g_thumbMouseResidualDx = dxFloat - (float)dx;
  g_thumbMouseResidualDy = dyFloat - (float)dy;
  g_isMouseMoving = (dx != 0 || dy != 0 || fabsf(g_thumbMouseVizDx) > 0.25f || fabsf(g_thumbMouseVizDy) > 0.25f);

  if (g_bleMouseSendEnabled && g_bleMouse.isConnected() && (dx != 0 || dy != 0)) {
    g_bleMouse.move(dx, dy, 0, 0);
  }
#else
  resetContinuousThumbMouseControl();
#endif
#endif
}

// =====================
// Sensor mapping / conversions
// =====================
static inline gag::Sensor mapToRecognizerSensor(uint8_t sensorIdx) {
  switch (sensorIdx) {
    case SENSOR_WRIST_GY25:
    case SENSOR_WRIST_MPU9250:
    case SENSOR_WRIST_GY511:
      return gag::Sensor::WRIST;
    case SENSOR_THUMB: return gag::Sensor::THUMB;
    case SENSOR_INDEX: return gag::Sensor::INDEX;
    case SENSOR_MIDDLE: return gag::Sensor::MIDDLE;
    case SENSOR_RING: return gag::Sensor::RING;
    case SENSOR_LITTLE: return gag::Sensor::LITTLE;
    default: return gag::Sensor::WRIST;
  }
}

static inline bool isFingerSensor(uint8_t sensorIdx) {
  return sensorIdx <= SENSOR_LITTLE;
}

static inline bool isMpuBackedSensor(uint8_t sensorIdx) {
  return sensorIdx < SENSOR_COUNT_ALL && sensorIdx != SENSOR_WRIST_GY511;
}

static inline uint8_t sensorToVizSlot(uint8_t sensorIdx) {
  switch (sensorIdx) {
    case SENSOR_WRIST_GY25: return 0;
    case SENSOR_THUMB: return 1;
    case SENSOR_INDEX: return 2;
    case SENSOR_MIDDLE: return 3;
    case SENSOR_RING: return 4;
    case SENSOR_LITTLE: return 5;
    case SENSOR_WRIST_MPU9250: return 6;
    case SENSOR_WRIST_GY511: return 7;
    default: return 7;
  }
}

static gag::Quaternion quatFromAxisAngleRad(float ax, float ay, float az, float rad) {
  const float axisNorm = sqrtf(ax * ax + ay * ay + az * az);
  if (axisNorm <= 1e-6f || fabsf(rad) <= 1e-6f) return gag::Quaternion();
  const float half = 0.5f * rad;
  const float s = sinf(half) / axisNorm;
  gag::Quaternion q(cosf(half), ax * s, ay * s, az * s);
  q.normalizeInPlace();
  return q;
}

static Vec3 rotateVectorByQuat(const gag::Quaternion& qIn, const Vec3& v) {
  gag::Quaternion q = qIn;
  q.normalizeInPlace();
  const float tx = 2.0f * (q.y * v.z - q.z * v.y);
  const float ty = 2.0f * (q.z * v.x - q.x * v.z);
  const float tz = 2.0f * (q.x * v.y - q.y * v.x);
  Vec3 out{
    v.x + q.w * tx + (q.y * tz - q.z * ty),
    v.y + q.w * ty + (q.z * tx - q.x * tz),
    v.z + q.w * tz + (q.x * ty - q.y * tx)
  };
  return out;
}

static Vec3 rotateVectorByQuatInverse(const gag::Quaternion& qIn, const Vec3& v) {
  gag::Quaternion q = qIn;
  q.normalizeInPlace();
  return rotateVectorByQuat(q.inverseUnit(), v);
}

static gag::Quaternion quatNlerp(const gag::Quaternion& aIn, const gag::Quaternion& bIn, float t) {
  const float tt = clamp01(t);
  gag::Quaternion a = aIn;
  gag::Quaternion b = bIn;
  a.normalizeInPlace();
  b.normalizeInPlace();
  if (gag::Quaternion::dot(a, b) < 0.0f) {
    b.w = -b.w;
    b.x = -b.x;
    b.y = -b.y;
    b.z = -b.z;
  }
  gag::Quaternion out(
    a.w + (b.w - a.w) * tt,
    a.x + (b.x - a.x) * tt,
    a.y + (b.y - a.y) * tt,
    a.z + (b.z - a.z) * tt);
  out.normalizeInPlace();
  return out;
}

static gag::Quaternion quatFromTwoUnitVectors(const Vec3& fromIn, const Vec3& toIn) {
  const Vec3 from = vecNormalize(fromIn);
  const Vec3 to = vecNormalize(toIn);
  const float dRaw = vecDot(from, to);
  const float d = dRaw < -1.0f ? -1.0f : (dRaw > 1.0f ? 1.0f : dRaw);
  if (d >= 0.999999f) return gag::Quaternion();
  if (d <= -0.999999f) {
    const Vec3 helper = (fabsf(from.z) < 0.9f) ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
    const Vec3 axis = vecNormalize(vecCross(from, helper));
    return quatFromAxisAngleRad(axis.x, axis.y, axis.z, (float)M_PI);
  }
  const Vec3 axis = vecNormalize(vecCross(from, to));
  return quatFromAxisAngleRad(axis.x, axis.y, axis.z, acosf(d));
}

static bool wristGy25LooksStationary(const Vec3& accelBody,
                                    float gxDegPerSec,
                                    float gyDegPerSec,
                                    float gzDegPerSec) {
  const float accelNorm = vecNorm(accelBody);
  const float gyroAbsMax = fmaxf(fabsf(gxDegPerSec), fmaxf(fabsf(gyDegPerSec), fabsf(gzDegPerSec)));
  return fabsf(accelNorm - 1.0f) <= 0.08f && gyroAbsMax <= 3.0f;
}

static void updateWristGy25RuntimeBias(const Vec3& accelBody,
                                       float gxDegPerSecRaw,
                                       float gyDegPerSecRaw,
                                       float gzDegPerSecRaw,
                                       uint32_t nowMs) {
  if (!wristGy25LooksStationary(accelBody, gxDegPerSecRaw, gyDegPerSecRaw, gzDegPerSecRaw)) {
    g_wristGy25StillSinceMs = 0;
    return;
  }

  if (g_wristGy25StillSinceMs == 0) {
    g_wristGy25StillSinceMs = nowMs;
    return;
  }

  if ((uint32_t)(nowMs - g_wristGy25StillSinceMs) < 1500UL) return;

  const float adapt = 0.01f;
  g_wristGy25RuntimeBiasDegX += (gxDegPerSecRaw - g_wristGy25RuntimeBiasDegX) * adapt;
  g_wristGy25RuntimeBiasDegY += (gyDegPerSecRaw - g_wristGy25RuntimeBiasDegY) * adapt;
  g_wristGy25RuntimeBiasDegZ += (gzDegPerSecRaw - g_wristGy25RuntimeBiasDegZ) * adapt;

  if ((uint32_t)(nowMs - g_lastWristGy25BiasLogMs) >= 5000UL) {
    g_lastWristGy25BiasLogMs = nowMs;
    Serial.printf("GY25 runtime gyro bias deg/s = { %.3f, %.3f, %.3f }\n",
                  g_wristGy25RuntimeBiasDegX,
                  g_wristGy25RuntimeBiasDegY,
                  g_wristGy25RuntimeBiasDegZ);
  }
}

static gag::Quaternion integrateGyroQuaternion(const gag::Quaternion& currentIn,
                                               float gxDegPerSec,
                                               float gyDegPerSec,
                                               float gzDegPerSec,
                                               float dtSec) {
  gag::Quaternion current = currentIn;
  current.normalizeInPlace();
  const float omegaDegPerSec = sqrtf(gxDegPerSec * gxDegPerSec + gyDegPerSec * gyDegPerSec + gzDegPerSec * gzDegPerSec);
  if (omegaDegPerSec <= 1e-6f || dtSec <= 0.0f) return current;
  const gag::Quaternion delta = quatFromAxisAngleRad(gxDegPerSec, gyDegPerSec, gzDegPerSec, deg2rad(omegaDegPerSec * dtSec));
  gag::Quaternion out = gag::Quaternion::mul(current, delta);
  out.normalizeInPlace();
  return out;
}

static gag::Quaternion applyTiltCorrection(const gag::Quaternion& currentIn, const Vec3& accelBody, float gain) {
  const Vec3 upBody = vecNormalize(accelBody);
  if (vecNorm(upBody) <= 1e-6f) return currentIn;
  gag::Quaternion current = currentIn;
  current.normalizeInPlace();
  const Vec3 predictedUpWorld = vecNormalize(rotateVectorByQuat(current, upBody));
  const gag::Quaternion correction = quatFromTwoUnitVectors(predictedUpWorld, kWorldUp);
  const gag::Quaternion partial = quatNlerp(gag::Quaternion(), correction, gain);
  gag::Quaternion out = gag::Quaternion::mul(partial, current);
  out.normalizeInPlace();
  return out;
}

static gag::Quaternion applyHeadingCorrection(const gag::Quaternion& currentIn, const Vec3& magBody, float gain) {
  const gag::Quaternion current = currentIn.normalized();
  const Vec3 upBody = vecNormalize(rotateVectorByQuatInverse(current, kWorldUp));
  if (vecNorm(upBody) <= 1e-6f) return current;

  Vec3 northBody = vecSub(magBody, vecScale(upBody, vecDot(magBody, upBody)));
  northBody = vecNormalize(northBody);
  if (vecNorm(northBody) <= 1e-6f) return current;

  Vec3 predictedNorthWorld = rotateVectorByQuat(current, northBody);
  predictedNorthWorld.z = 0.0f;
  predictedNorthWorld = vecNormalize(predictedNorthWorld);
  if (vecNorm(predictedNorthWorld) <= 1e-6f) return current;

  const float crossZ = predictedNorthWorld.x * kWorldNorth.y - predictedNorthWorld.y * kWorldNorth.x;
  const float dotv = predictedNorthWorld.x * kWorldNorth.x + predictedNorthWorld.y * kWorldNorth.y;
  const gag::Quaternion correction = quatFromAxisAngleRad(0.0f, 0.0f, 1.0f, atan2f(crossZ, dotv) * clamp01(gain));
  gag::Quaternion out = gag::Quaternion::mul(correction, current);
  out.normalizeInPlace();
  return out;
}

static gag::Quaternion applyQuaternionAngularDeadband(uint8_t sensorIdx,
                                                      const gag::Quaternion& prevIn,
                                                      const gag::Quaternion& candidateIn,
                                                      float dtSec) {
  if (sensorIdx >= SENSOR_COUNT_ALL) return candidateIn;
  const float thresholdDegPerSec = SENSOR_ROTATION_IGNORE_DEGS[sensorIdx];
  if (thresholdDegPerSec <= 0.0f || dtSec <= 0.0f) return candidateIn;

  const gag::Quaternion prev = prevIn.normalized();
  const gag::Quaternion candidate = candidateIn.normalized();
  const float deltaDeg = rad2deg(gag::Quaternion::angularDistance(prev, candidate));
  const float speedDegPerSec = deltaDeg / dtSec;

  if (speedDegPerSec <= thresholdDegPerSec) return prev;
  const float fullAcceptDegPerSec = thresholdDegPerSec * 2.0f;
  if (speedDegPerSec >= fullAcceptDegPerSec) return candidate;

  const float blend = (speedDegPerSec - thresholdDegPerSec) / (fullAcceptDegPerSec - thresholdDegPerSec);
  return quatNlerp(prev, candidate, blend);
}

static void remapMpuRawAxesToGloveFrame(uint8_t sensorIdx,
                                        int16_t& ax, int16_t& ay, int16_t& az,
                                        int16_t& gx, int16_t& gy, int16_t& gz) {
  const int16_t axIn = ax;
  const int16_t ayIn = ay;
  const int16_t azIn = az;
  const int16_t gxIn = gx;
  const int16_t gyIn = gy;
  const int16_t gzIn = gz;

  if (sensorIdx == SENSOR_INDEX) {
    // Index IMU is mounted differently from the other finger modules.
    // ax = axIn;
    // ay = azIn;
    // az = (int16_t)-ayIn;
    // gx = gxIn;
    // gy = gzIn;
    // gz = (int16_t)-gyIn;
    ax = (int16_t)ayIn;
    ay = (int16_t)azIn;
    az = (int16_t)axIn;
    gx = (int16_t)gyIn;
    gy = (int16_t)gzIn;
    gz = (int16_t)gxIn;
    return;
  }

  // if (sensorIdx == SENSOR_MIDDLE || sensorIdx == SENSOR_THUMB) {
  //   // Thumb and middle IMUs share a second mounting orientation.
  //   ax = axIn;
  //   ay = (int16_t)-azIn;
  //   az = (int16_t)ayIn;
  //   gx = gxIn;
  //   gy = (int16_t)-gzIn;
  //   gz = (int16_t)gyIn;
  //   return;
  // }

   if (sensorIdx == SENSOR_MIDDLE || sensorIdx == SENSOR_THUMB || sensorIdx == SENSOR_RING) {
    // Thumb, middle, and ring IMUs share a second mounting orientation.
    // Serial.println("SENSOR_WRIST_GY25");
    // Serial.println(sensorIdx);
    ax = (int16_t)ayIn;
    ay = (int16_t)-axIn;
    az = (int16_t)azIn;
    gx = (int16_t)gyIn;
    gy = (int16_t)-gxIn;
    gz = (int16_t)gzIn;
    return;
  }

  if (sensorIdx == SENSOR_WRIST_MPU9250) {
    // Wrist MPU9250 has its own mounting orientation.
    ax = (int16_t)-azIn;
    ay = (int16_t)-ayIn;
    az = (int16_t)axIn;
    gx = (int16_t)-gzIn;
    gy = (int16_t)-gyIn;
    gz = (int16_t)gxIn;
    return;
  }

  if (sensorIdx == SENSOR_WRIST_GY25) {
    // Wrist GY25 has its own mounting orientation.
    // Serial.println("SENSOR_WRIST_GY25");
    // Serial.println(sensorIdx);
    ax = (int16_t)-ayIn;
    ay = (int16_t)axIn;
    az = (int16_t)azIn;
    gx = (int16_t)-gyIn;
    gy = (int16_t)gxIn;
    gz = (int16_t)gzIn;
    return;
  }
  
  // Default mapping for the remaining MPU6050-class modules in the current glove build.
  ax = azIn;
  ay = ayIn;
  az = axIn;
  gx = gzIn;
  gy = gyIn;
  gz = gxIn;
}

static gag::Quaternion rawQuaternionForPhysicalSensor(uint8_t sensorIdx) {
  if (sensorIdx >= SENSOR_COUNT_ALL) return gag::Quaternion();
  gag::Quaternion q = g_sensorFusionQuat[sensorIdx];
  q.normalizeInPlace();
  return q;
}

static gag::Quaternion applyDefaultSensorRotation(uint8_t sensorIdx, const gag::Quaternion& rawIn) {
  gag::Quaternion raw = rawIn;
  raw.normalizeInPlace();
  if (sensorIdx >= SENSOR_COUNT_ALL) return raw;
  gag::Quaternion out = gag::Quaternion::mul(raw, DEFAULT_SENSOR_ROTATION[sensorIdx]);
  out.normalizeInPlace();
  return out;
}

static gag::Quaternion applyMinorRotationOffset(uint8_t sensorIdx, const gag::Quaternion& physicalFixedIn) {
  gag::Quaternion q = physicalFixedIn;
  q.normalizeInPlace();
  if (sensorIdx >= SENSOR_COUNT_ALL) return q;
#if GAG_APPLY_MINOR_ROTATION_OFFSET
  gag::Quaternion out = gag::Quaternion::mul(q, g_minorRotationOffset[sensorIdx]);
  out.normalizeInPlace();
  return out;
#else
  return q;
#endif
}

static bool isSensorEnabled(uint8_t sensorIdx) {
  return sensorIdx < SENSOR_COUNT_ALL && SENSOR_ENABLED[sensorIdx];
}

static bool physicalSensorQuaternionAvailable(uint8_t sensorIdx) {
  if (!isSensorEnabled(sensorIdx)) return false;
  return g_sensorInitOk[sensorIdx] && g_sensorFusionInitialized[sensorIdx];
}

static gag::Quaternion physicalFixedQuaternionForPhysicalSensor(uint8_t sensorIdx) {
  const gag::Quaternion physicalFixed = applyDefaultSensorRotation(sensorIdx, rawQuaternionForPhysicalSensor(sensorIdx));
  return applyMinorRotationOffset(sensorIdx, physicalFixed);
}

static gag::Quaternion correctedQuaternionForPhysicalSensor(uint8_t sensorIdx) {
  return g_offsets.applySoftwareOffset(sensorIdx, physicalFixedQuaternionForPhysicalSensor(sensorIdx));
}

static gag::Quaternion applyWristPivotRotationCorrection(uint8_t sensorIdx, const gag::Quaternion& qIn) {
  gag::Quaternion q = qIn;
  q.normalizeInPlace();
#if GAG_APPLY_GY511_WRIST_PIVOT_ROTATION_FIX
  if (sensorIdx == SENSOR_WRIST_GY511) {
    // Keep the GY-511 cube mapping fix, but rotate the wrist pivot back into
    // the finger-relative frame used by the hand skeleton and relative cubes.
    // const gag::Quaternion pivotFixY = gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, 90.0f);
    // const gag::Quaternion pivotFixZ = gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, -90.0f);
    // const gag::Quaternion pivotFix = gag::Quaternion::mul(pivotFixY, pivotFixZ);
    const gag::Quaternion pivotFix  = gag::Quaternion::fromAxisAngleDeg(1.0f, 0.0f, 0.0f, 180.0f);
    const gag::Quaternion pivotFix2 = gag::Quaternion::fromAxisAngleDeg(0.0f, 1.0f, 0.0f, 180.0f);
    const gag::Quaternion pivotFix3 = gag::Quaternion::fromAxisAngleDeg(0.0f, 0.0f, 1.0f, 0.0f);
    gag::Quaternion out = gag::Quaternion::mul(gag::Quaternion::mul(gag::Quaternion::mul(q, pivotFix), pivotFix2), pivotFix3);
    // gag::Quaternion out = gag::Quaternion::mul(gag::Quaternion::mul(q, pivotFix), pivotFix2);
    out.normalizeInPlace();
    return out;
  }
#endif
  return q;
}

static uint8_t firstAvailableWristSensor(uint8_t a, uint8_t b, uint8_t c) {
  if (physicalSensorQuaternionAvailable(a)) return a;
  if (physicalSensorQuaternionAvailable(b)) return b;
  if (physicalSensorQuaternionAvailable(c)) return c;
  return a;
}

// Select the single wrist sensor used by the hand skeleton and recognizer.
// All physical wrist sensors can still be updated and drawn as separate cubes.
static uint8_t selectedWristQuaternionPhysicalSensor() {
#if GAG_PRIMARY_WRIST_SENSOR == GAG_PRIMARY_WRIST_SENSOR_GY511
  return firstAvailableWristSensor(SENSOR_WRIST_GY511, SENSOR_WRIST_GY25, SENSOR_WRIST_MPU9250);
#elif GAG_PRIMARY_WRIST_SENSOR == GAG_PRIMARY_WRIST_SENSOR_MPU9250
  return firstAvailableWristSensor(SENSOR_WRIST_MPU9250, SENSOR_WRIST_GY25, SENSOR_WRIST_GY511);
#else
  return firstAvailableWristSensor(SENSOR_WRIST_GY25, SENSOR_WRIST_MPU9250, SENSOR_WRIST_GY511);
#endif
}

static uint8_t selectedWristAccelPhysicalSensor() {
  const uint8_t wristSensor = selectedWristQuaternionPhysicalSensor();
  if (isSensorEnabled(wristSensor) && g_sensorInitOk[wristSensor]) return wristSensor;
  if (isSensorEnabled(SENSOR_WRIST_GY25) && g_sensorInitOk[SENSOR_WRIST_GY25]) return SENSOR_WRIST_GY25;
  if (isSensorEnabled(SENSOR_WRIST_MPU9250) && g_sensorInitOk[SENSOR_WRIST_MPU9250]) return SENSOR_WRIST_MPU9250;
  if (isSensorEnabled(SENSOR_WRIST_GY511) && g_sensorInitOk[SENSOR_WRIST_GY511]) return SENSOR_WRIST_GY511;
  return wristSensor;
}

static bool selectedWristQuaternionAvailable() {
  return physicalSensorQuaternionAvailable(selectedWristQuaternionPhysicalSensor());
}

static gag::Quaternion correctedLogicalWristQuaternion() {
  return correctedQuaternionForPhysicalSensor(selectedWristQuaternionPhysicalSensor());
}

static gag::Quaternion recognitionQuaternionForPhysicalSensor(uint8_t sensorIdx) {
#if GAG_ENABLE_RECOGNITION_DRIFT_RESET_OFFSET
  return correctedQuaternionForPhysicalSensor(sensorIdx);
#else
  return physicalFixedQuaternionForPhysicalSensor(sensorIdx);
#endif
}

static gag::Quaternion recognitionLogicalWristQuaternion() {
#if GAG_ENABLE_RECOGNITION_DRIFT_RESET_OFFSET
  return correctedLogicalWristQuaternion();
#else
  return physicalFixedQuaternionForPhysicalSensor(selectedWristQuaternionPhysicalSensor());
#endif
}

static uint16_t selectedLogicalWristColor() {
  return SENSOR_COLORS[selectedWristQuaternionPhysicalSensor()];
}

static void maybeLogSerialSensorQuaternions() {
#if GAG_ENABLE_SERIAL_SENSOR_QUAT_LOG
  if (g_serialQuatLogSensorMask == 0u) return;

  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastSerialQuatLogMs) < (uint32_t)GAG_SERIAL_SENSOR_QUAT_LOG_INTERVAL_MS) return;
  g_lastSerialQuatLogMs = now;

  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (!(g_serialQuatLogSensorMask & sensorBitMask(s))) continue;

    if (!physicalSensorQuaternionAvailable(s)) {
      Serial.printf("quat sensor=%u label=%s available=0\n",
                    (unsigned)s,
                    SENSOR_OFFSET_LABELS[s]);
      continue;
    }

    const gag::Quaternion q = correctedQuaternionForPhysicalSensor(s);
    Serial.printf("quat sensor=%u label=%s available=1 w=%.5f x=%.5f y=%.5f z=%.5f\n",
                  (unsigned)s,
                  SENSOR_OFFSET_LABELS[s],
                  q.w, q.x, q.y, q.z);
  }
#endif
}

// =====================
// GY-511
// =====================
static bool initGY511() {
  pcaSelect(CH_GY511);
  i2cWriteByte(LSM_ACC_ADDR, LSM_CTRL_REG1_A, 0x57);  // 100Hz, XYZ enable
  i2cWriteByte(LSM_ACC_ADDR, LSM_CTRL_REG4_A, 0x00);  // ±2g
  i2cWriteByte(LSM_MAG_ADDR, LSM_CRA_REG_M, 0x14);    // 30Hz
  i2cWriteByte(LSM_MAG_ADDR, LSM_CRB_REG_M, 0x20);    // +/-1.3 gauss
  i2cWriteByte(LSM_MAG_ADDR, LSM_MR_REG_M, 0x00);     // continuous
  delay(20);
  const uint8_t ctrl1a = i2cReadByte(LSM_ACC_ADDR, LSM_CTRL_REG1_A);
  const uint8_t who = i2cReadByte(LSM_MAG_ADDR, 0x0A);
  gy511MagOk = (who == 'H');
  gy511LastT = millis();
  return (ctrl1a == 0x57) || gy511MagOk;
}

static bool readGY511Accel(Vec3& accel_g) {
  pcaSelect(CH_GY511);
  uint8_t buf[6] = { 0 };
  i2cReadBytes(LSM_ACC_ADDR, (LSM_OUT_X_L_A | 0x80), buf, 6);
  int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
  const gag::offsets::HwOffset6 hw = g_offsets.hardware(SENSOR_WRIST_GY511);
  ax -= hw.ax;
  ay -= hw.ay;
  az -= hw.az;
  accel_g.x = (float)ax / 16384.0f;
  accel_g.y = (float)ay / 16384.0f;
  accel_g.z = (float)az / 16384.0f;
  return true;
}

static bool readGY511Mag(Vec3& magRaw) {
  pcaSelect(CH_GY511);
  uint8_t buf[6] = { 0 };
  i2cReadBytes(LSM_MAG_ADDR, LSM_OUT_X_H_M, buf, 6);
  int16_t mx = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t mz = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t my = (int16_t)((buf[4] << 8) | buf[5]);
  magRaw.x = (float)mx;
  magRaw.y = (float)my;
  magRaw.z = (float)mz;
  return true;
}

static void remapGy511VectorToGloveFrame(Vec3& v) {
  const float xIn = v.x;
  const float yIn = v.y;
  const float zIn = v.z;

  // GY-511 is mounted with +180 deg around Z and then +90 deg around X.
  // Apply the inverse mounting rotation to raw vectors before deriving
  // roll, pitch, and tilt-compensated yaw.
  // v.x = -zIn;
  // v.y = -xIn;
  // v.z = -yIn;
  // v.x = zIn;
  // v.y = xIn;
  // v.z = yIn;
  v.x = yIn;
  v.y = -zIn;
  v.z = xIn;
}

static void updateGY511() {
  if (!gy511Ok || !g_sensorInitOk[SENSOR_WRIST_GY511]) return;
  const uint32_t nowMs = millis();
  float dtSec = (gy511LastT == 0) ? 0.01f : (float)(nowMs - gy511LastT) / 1000.0f;
  if (dtSec <= 0.0f) dtSec = 0.01f;
  if (dtSec > 0.1f) dtSec = 0.1f;
  Vec3 a;
  if (!readGY511Accel(a)) return;
  remapGy511VectorToGloveFrame(a);
  gy511Accel_g = a;
  g_lastAccelBody[SENSOR_WRIST_GY511] = a;
  g_lastAccelBodyValid[SENSOR_WRIST_GY511] = true;

  const gag::Quaternion prevQ = g_sensorFusionQuat[SENSOR_WRIST_GY511];
  gag::Quaternion q = prevQ;
  if (!g_sensorFusionInitialized[SENSOR_WRIST_GY511]) {
    q = gag::Quaternion();
  }
  q = applyTiltCorrection(q, a, 1.0f);

  if (gy511MagOk) {
    Vec3 m;
    if (readGY511Mag(m)) {
      remapGy511VectorToGloveFrame(m);
      gy511MagRaw = m;
      q = applyHeadingCorrection(q, m, 1.0f);
    }
  }

  if (g_sensorFusionInitialized[SENSOR_WRIST_GY511]) {
    q = applyQuaternionAngularDeadband(SENSOR_WRIST_GY511, prevQ, q, dtSec);
  }

  g_sensorFusionQuat[SENSOR_WRIST_GY511] = q;
  g_sensorFusionInitialized[SENSOR_WRIST_GY511] = true;
  gy511LastT = nowMs;
}

// =====================
// Wrist AK8963
// =====================
static bool initWristMagAK8963() {
  if (!isSensorEnabled(SENSOR_WRIST_MPU9250)) return false;
  pcaSelect(ACTIVE_CHANNELS[SENSOR_WRIST_MPU9250]);
  i2cWriteByte(wristMpuAddress(), REG_INT_PIN_CFG, 0x02);  // bypass enable
  delay(10);
  uint8_t who = i2cReadByte(AK8963_ADDR, AK8963_WHO_AM_I);
  if (who != 0x48) return false;
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x00);
  delay(10);
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x16);
  delay(10);  // continuous 2, 16-bit
  return true;
}

static bool readWristMag(Vec3& magOut) {
  if (!isSensorEnabled(SENSOR_WRIST_MPU9250)) return false;
  pcaSelect(ACTIVE_CHANNELS[SENSOR_WRIST_MPU9250]);
  uint8_t st1 = i2cReadByte(AK8963_ADDR, AK8963_ST1);
  if (!(st1 & 0x01)) return false;
  uint8_t buf[7] = { 0 };
  i2cReadBytes(AK8963_ADDR, AK8963_HXL, buf, 7);
  int16_t mx = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t my = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t mz = (int16_t)((buf[5] << 8) | buf[4]);
  magOut.x = (float)mx;
  magOut.y = (float)my;
  magOut.z = (float)mz;
  return true;
}

static void updateWristMagYaw() {
  if (!wristMagOk) return;
  if (!g_sensorFusionInitialized[SENSOR_WRIST_MPU9250]) return;
  Vec3 m;
  if (readWristMag(m)) {
    wristMagRaw = m;
    const gag::Quaternion prevQ = g_sensorFusionQuat[SENSOR_WRIST_MPU9250];
    gag::Quaternion q = applyHeadingCorrection(prevQ, m, kHeadingCorrectionGain);
    const uint32_t nowMs = millis();
    float dtSec = (lastT[SENSOR_WRIST_MPU9250] == 0)
                    ? 0.02f
                    : (float)(nowMs - lastT[SENSOR_WRIST_MPU9250]) / 1000.0f;
    if (dtSec < 0.02f) dtSec = 0.02f;
    if (dtSec > 0.1f) dtSec = 0.1f;
    g_sensorFusionQuat[SENSOR_WRIST_MPU9250] =
      applyQuaternionAngularDeadband(SENSOR_WRIST_MPU9250, prevQ, q, dtSec);
  }
}

// =====================
// IMU init/update
// =====================
// Current implementation keeps the original non-DMP accel/gyro pipeline.
// A future DMP quaternion path should be wired under GAG_USE_MPU_DMP_QUAT_FIFO.
static bool initOneIMU(uint8_t idx) {
  if (!isSensorEnabled(idx)) return true;
  if (!isMpuBackedSensor(idx)) return true;
  const uint8_t ch = ACTIVE_CHANNELS[idx];
  pcaSelect(ch);

  if (usesDirectMpuRegisterAccess(idx)) {
    const uint8_t mpuAddr = detectMpuAddressForSensor(idx);
    uint8_t who = 0;
    if (idx == SENSOR_WRIST_MPU9250) {
      if (mpuAddr == 0) return false;
      g_wristMpuAddr = mpuAddr;
      who = i2cReadByte(mpuAddr, REG_WHO_AM_I);
#if GAG_ENABLE_WRIST_MPU_PROBE_LOG
      Serial.printf("Wrist MPU probe: ch=%u addr68=0x%02X addr69=0x%02X selected=0x%02X who=0x%02X\n",
                    (unsigned)ch,
                    (unsigned)i2cReadByte(MPU9250_ADDR_DEFAULT, REG_WHO_AM_I),
                    (unsigned)i2cReadByte(MPU9250_ADDR_ALT, REG_WHO_AM_I),
                    (unsigned)mpuAddr,
                    (unsigned)who);
#endif
      if (!(who == 0x71 || who == 0x73)) return false;
    } else {
      if (mpuAddr == 0) return false;
      g_sensorMpuAddr[idx] = mpuAddr;
    }
    i2cWriteByte(mpuAddr, REG_PWR_MGMT_1, MPU_PWR_CLKSEL_ZGYRO);
    delay(10);
    i2cWriteByte(mpuAddr, REG_GYRO_CONFIG, 0x00);
    i2cWriteByte(mpuAddr, REG_ACCEL_CONFIG, 0x00);
    configureMpuFifo(idx);
    return true;
  }

  const uint8_t mpuAddr = detectMpuAddressForSensor(idx);
  if (mpuAddr == 0) return false;
  g_sensorMpuAddr[idx] = mpuAddr;
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
    configureMpuFifo(idx);
  }
  return ok;
}

static void updateOneIMU(uint8_t idx) {
  if (!isSensorEnabled(idx)) return;
  if (!isMpuBackedSensor(idx)) return;
  if (!g_sensorInitOk[idx]) return;
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  maybeResetMpuFifo(idx);

  bool haveSample = readMpuFifoMotion6(idx, ax, ay, az, gx, gy, gz);
  if (!haveSample) {
    pcaSelect(ACTIVE_CHANNELS[idx]);
    if (usesDirectMpuRegisterAccess(idx)) {
      uint8_t buf[14] = { 0 };
      i2cReadBytes(mpuAddressForSensor(idx), REG_ACCEL_XOUT_H, buf, 14);
      ax = (int16_t)((buf[0] << 8) | buf[1]);
      ay = (int16_t)((buf[2] << 8) | buf[3]);
      az = (int16_t)((buf[4] << 8) | buf[5]);
      gx = (int16_t)((buf[8] << 8) | buf[9]);
      gy = (int16_t)((buf[10] << 8) | buf[11]);
      gz = (int16_t)((buf[12] << 8) | buf[13]);
    } else {
      mpu[idx].getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    }
  }

  if (usesDirectMpuRegisterAccess(idx)) {
    // Direct raw-register paths do not get hardware offsets applied by the driver.
    const gag::offsets::HwOffset6 hw = g_offsets.hardware(idx);
    ax -= hw.ax;
    ay -= hw.ay;
    az -= hw.az;
    gx -= hw.gx;
    gy -= hw.gy;
    gz -= hw.gz;
  }

  remapMpuRawAxesToGloveFrame(idx, ax, ay, az, gx, gy, gz);

  const unsigned long now = millis();
  float dt = (now - lastT[idx]) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.1f) dt = 0.1f;
  lastT[idx] = now;

  const Vec3 accelBody{
    ax / 16384.0f,
    ay / 16384.0f,
    az / 16384.0f
  };
  g_lastAccelBody[idx] = accelBody;
  g_lastAccelBodyValid[idx] = true;
  float gxDegPerSec = gx / 131.0f;
  float gyDegPerSec = gy / 131.0f;
  float gzDegPerSec = gz / 131.0f;

  if (idx == SENSOR_WRIST_GY25) {
    updateWristGy25RuntimeBias(accelBody, gxDegPerSec, gyDegPerSec, gzDegPerSec, now);
    gxDegPerSec -= g_wristGy25RuntimeBiasDegX;
    gyDegPerSec -= g_wristGy25RuntimeBiasDegY;
    gzDegPerSec -= g_wristGy25RuntimeBiasDegZ;
  }

  const gag::Quaternion prevQ = g_sensorFusionQuat[idx];
  gag::Quaternion q = prevQ;
  if (!g_sensorFusionInitialized[idx]) {
    q = applyTiltCorrection(gag::Quaternion(), accelBody, 1.0f);
    g_sensorFusionInitialized[idx] = true;
  } else {
    q = integrateGyroQuaternion(q, gxDegPerSec, gyDegPerSec, gzDegPerSec, dt);
    q = applyTiltCorrection(q, accelBody, kTiltCorrectionGain);
    q = applyQuaternionAngularDeadband(idx, prevQ, q, dt);
  }

  g_sensorFusionQuat[idx] = q;
}

// =====================
// Optional boot-time offset measurement
// =====================
static bool readRawSampleForOffset(uint8_t sensorIdx, gag::offsets::RawImuSample& out) {
  if (!isSensorAvailableForOffsetMeasurement(sensorIdx)) return false;
  if (sensorIdx == SENSOR_WRIST_GY511) {
    Vec3 a;
    if (!readGY511Accel(a)) return false;
    out.ax = (int16_t)(a.x * 16384.0f);
    out.ay = (int16_t)(a.y * 16384.0f);
    out.az = (int16_t)(a.z * 16384.0f);
    out.gx = out.gy = out.gz = 0;
    return true;
  }

  pcaSelect(ACTIVE_CHANNELS[sensorIdx]);
  if (usesDirectMpuRegisterAccess(sensorIdx)) {
    uint8_t buf[14] = { 0 };
    i2cReadBytes(mpuAddressForSensor(sensorIdx), REG_ACCEL_XOUT_H, buf, 14);
    out.ax = (int16_t)((buf[0] << 8) | buf[1]);
    out.ay = (int16_t)((buf[2] << 8) | buf[3]);
    out.az = (int16_t)((buf[4] << 8) | buf[5]);
    out.gx = (int16_t)((buf[8] << 8) | buf[9]);
    out.gy = (int16_t)((buf[10] << 8) | buf[11]);
    out.gz = (int16_t)((buf[12] << 8) | buf[13]);
    return true;
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpu[sensorIdx].getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  out.ax = ax;
  out.ay = ay;
  out.az = az;
  out.gx = gx;
  out.gy = gy;
  out.gz = gz;
  return true;
}

static bool hasConfiguredImuChannel(uint8_t sensorIdx) {
  return sensorIdx < SENSOR_COUNT_ALL && isMpuBackedSensor(sensorIdx) && isSensorEnabled(sensorIdx) && g_sensorInitOk[sensorIdx];
}

static bool isSensorAvailableForOffsetMeasurement(uint8_t sensorIdx) {
  if (!isSensorEnabled(sensorIdx)) return false;
  if (sensorIdx == SENSOR_WRIST_GY511) return gy511Ok;
  return hasConfiguredImuChannel(sensorIdx);
}

static bool hardwareCalibrationEnabledForSensor(uint8_t sensorIdx) {
#if GAG_MEASURE_HW_OFFSETS_AT_BOOT
  return sensorIdx < SENSOR_COUNT_ALL;
#else
  switch (sensorIdx) {
    case SENSOR_THUMB: return GAG_MEASURE_HW_OFFSETS_SENSOR_THUMB;
    case SENSOR_INDEX: return GAG_MEASURE_HW_OFFSETS_SENSOR_INDEX;
    case SENSOR_MIDDLE: return GAG_MEASURE_HW_OFFSETS_SENSOR_MIDDLE;
    case SENSOR_RING: return GAG_MEASURE_HW_OFFSETS_SENSOR_RING;
    case SENSOR_LITTLE: return GAG_MEASURE_HW_OFFSETS_SENSOR_LITTLE;
    case SENSOR_WRIST_GY25: return GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_GY25;
    case SENSOR_WRIST_MPU9250: return GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_MPU9250;
    case SENSOR_WRIST_GY511: return GAG_MEASURE_HW_OFFSETS_SENSOR_WRIST_GY511;
    default: return false;
  }
#endif
}

static bool anyHardwareCalibrationEnabled() {
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (hardwareCalibrationEnabledForSensor(s)) return true;
  }
  return false;
}

static void printHardwareOffsetsArray() {
  Serial.println("static const gag::offsets::HwOffset6 DEFAULT_HW_OFFSETS[SENSOR_COUNT_ALL] = {");
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    const gag::offsets::HwOffset6 hw = g_offsets.hardware(s);
    Serial.printf("  { %d, %d, %d, %d, %d, %d }, // %s\n",
                  (int)hw.ax, (int)hw.ay, (int)hw.az,
                  (int)hw.gx, (int)hw.gy, (int)hw.gz,
                  SENSOR_OFFSET_LABELS[s]);
  }
  Serial.println("};");
}

static void printHardwareCalibrationStats(uint8_t sensorIdx,
                                          const gag::offsets::SampleStats& stats,
                                          bool stable,
                                          bool applied,
                                          const gag::offsets::HwOffset6& hw) {
  Serial.printf("HW calib sensor=%u label=%s stable=%u applied=%u samples=%u\n",
                (unsigned)sensorIdx,
                SENSOR_OFFSET_LABELS[sensorIdx],
                stable ? 1u : 0u,
                applied ? 1u : 0u,
                (unsigned)stats.count);
  if (!stats.valid) {
    Serial.println("  no samples captured");
    return;
  }
  Serial.printf("  mean={ %d, %d, %d, %d, %d, %d }\n",
                (int)stats.mean.ax, (int)stats.mean.ay, (int)stats.mean.az,
                (int)stats.mean.gx, (int)stats.mean.gy, (int)stats.mean.gz);
  Serial.printf("  min ={ %d, %d, %d, %d, %d, %d }\n",
                (int)stats.min.ax, (int)stats.min.ay, (int)stats.min.az,
                (int)stats.min.gx, (int)stats.min.gy, (int)stats.min.gz);
  Serial.printf("  max ={ %d, %d, %d, %d, %d, %d }\n",
                (int)stats.max.ax, (int)stats.max.ay, (int)stats.max.az,
                (int)stats.max.gx, (int)stats.max.gy, (int)stats.max.gz);
  Serial.printf("  hw  ={ %d, %d, %d, %d, %d, %d }\n",
                (int)hw.ax, (int)hw.ay, (int)hw.az,
                (int)hw.gx, (int)hw.gy, (int)hw.gz);
}

static void applyCurrentHardwareOffsetsToInitializedSensors() {
  for (uint8_t idx = 0; idx < SENSOR_COUNT_ALL; ++idx) {
    if (!isSensorEnabled(idx) || !isMpuBackedSensor(idx) || !g_sensorInitOk[idx]) continue;
    if (usesDirectMpuRegisterAccess(idx)) continue;
    const gag::offsets::HwOffset6 hw = g_offsets.hardware(idx);
    mpu[idx].setXAccelOffset(hw.ax);
    mpu[idx].setYAccelOffset(hw.ay);
    mpu[idx].setZAccelOffset(hw.az);
    mpu[idx].setXGyroOffset(hw.gx);
    mpu[idx].setYGyroOffset(hw.gy);
    mpu[idx].setZGyroOffset(hw.gz);
  }
}

static void measureHardwareOffsetsAtBoot() {
  if (!anyHardwareCalibrationEnabled()) return;
  gag::offsets::StableWindowConfig cfg;
  cfg.required_samples = GAG_HW_CALIBRATION_REQUIRED_SAMPLES;
  Serial.printf("HW calib start samples=%u delay_ms=%u\n",
                (unsigned)cfg.required_samples,
                (unsigned)GAG_HW_CALIBRATION_SAMPLE_DELAY_MS);
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (!hardwareCalibrationEnabledForSensor(s)) {
      Serial.printf("HW calib sensor=%u label=%s skipped disabled\n",
                    (unsigned)s,
                    SENSOR_OFFSET_LABELS[s]);
      continue;
    }
    if (!isSensorAvailableForOffsetMeasurement(s)) {
      Serial.printf("HW calib sensor=%u label=%s skipped unavailable\n",
                    (unsigned)s,
                    SENSOR_OFFSET_LABELS[s]);
      continue;
    }

    gag::offsets::StablePoseAccumulator acc;
    for (uint16_t i = 0; i < cfg.required_samples; ++i) {
      gag::offsets::RawImuSample sample;
      if (readRawSampleForOffset(s, sample)) {
        acc.push(sample);
      }
      delay(GAG_HW_CALIBRATION_SAMPLE_DELAY_MS);
    }

    const gag::offsets::SampleStats stats = acc.stats();
    const bool stable = acc.isStable(cfg);
    const gag::offsets::HwOffset6 previousHw = g_offsets.hardware(s);
    const gag::offsets::HwOffset6 proposedHw = gag::offsets::computeHardwareOffsetsFromStablePose(stats);
    bool applied = false;
    gag::offsets::HwOffset6 hw = previousHw;
    if (stats.valid && (stable || GAG_HW_CALIBRATION_APPLY_UNSTABLE)) {
      hw = proposedHw;
      g_offsets.setHardware(s, hw);
      applied = true;
      if (!stable) {
        Serial.println("  calibration window unstable, applying proposed hardware offsets anyway");
      }
    } else if (!stable) {
      Serial.println("  calibration window unstable, keeping previous hardware offsets");
    }
    printHardwareCalibrationStats(s, stats, stable, applied, hw);
  }
  applyCurrentHardwareOffsetsToInitializedSensors();
  printHardwareOffsetsArray();
}

// =====================
// Boot-time software offset neutral capture
// =====================
static gag::Quaternion averageCurrentSensorQuat(uint8_t sensorIdx, bool includeMinorRotationOffset, uint8_t samples = 16) {
  gag::Quaternion ref(1, 0, 0, 0);
  gag::Quaternion sum(0, 0, 0, 0);
  bool haveRef = false;

  if (!physicalSensorQuaternionAvailable(sensorIdx)) {
    return gag::Quaternion();
  }

  for (uint8_t i = 0; i < samples; ++i) {
    if (sensorIdx == SENSOR_WRIST_GY511) {
      updateGY511();
    } else {
      updateOneIMU(sensorIdx);
      if (sensorIdx == SENSOR_WRIST_MPU9250) updateWristMagYaw();
    }

    gag::Quaternion q = applyDefaultSensorRotation(sensorIdx, rawQuaternionForPhysicalSensor(sensorIdx));
    if (includeMinorRotationOffset) {
      q = applyMinorRotationOffset(sensorIdx, q);
    }
    q.normalizeInPlace();

    if (!haveRef) {
      ref = q;
      haveRef = true;
    }
    if (gag::Quaternion::dot(ref, q) < 0.0f) {
      q.w = -q.w;
      q.x = -q.x;
      q.y = -q.y;
      q.z = -q.z;
    }
    sum.w += q.w;
    sum.x += q.x;
    sum.y += q.y;
    sum.z += q.z;
    delay(10);
  }

  sum.normalizeInPlace();
  return sum;
}

static gag::Quaternion computeMinorRotationCompensation(const gag::Quaternion& currentPhysicalFixed,
                                                        const gag::Quaternion& desiredOrientation = gag::Quaternion()) {
  gag::Quaternion current = currentPhysicalFixed;
  current.normalizeInPlace();
  gag::Quaternion desired = desiredOrientation;
  desired.normalizeInPlace();
  gag::Quaternion out = gag::Quaternion::mul(current.inverseUnit(), desired);
  out.normalizeInPlace();
  return out;
}

static gag::Quaternion simultaneousDriftResetTrackingQuaternionForSensor(uint8_t sensorIdx) {
  gag::Quaternion q = physicalFixedQuaternionForPhysicalSensor(sensorIdx);
#if GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_RELATIVE_WRIST
  if (sensorIdx < SENSOR_COUNT_FINGERS && selectedWristQuaternionAvailable()) {
    q = gag::Quaternion::mul(physicalFixedQuaternionForPhysicalSensor(selectedWristQuaternionPhysicalSensor()).inverseUnit(), q);
    q.normalizeInPlace();
  }
#endif
  return q;
}

static gag::Quaternion simultaneousDriftResetCorrectedQuaternionForSensor(uint8_t sensorIdx) {
  gag::Quaternion q = correctedQuaternionForPhysicalSensor(sensorIdx);
#if GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_RELATIVE_WRIST
  if (sensorIdx < SENSOR_COUNT_FINGERS && selectedWristQuaternionAvailable()) {
    q = gag::Quaternion::mul(correctedLogicalWristQuaternion().inverseUnit(), q);
    q.normalizeInPlace();
  }
#endif
  return q;
}

static gag::Quaternion simultaneousDriftResetTargetMinorForSensor(uint8_t sensorIdx,
                                                                  const gag::Quaternion& currentDefaultFixed) {
#if GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET_RELATIVE_WRIST
  if (sensorIdx < SENSOR_COUNT_FINGERS && selectedWristQuaternionAvailable()) {
    gag::Quaternion targetPhysical = gag::Quaternion::mul(g_offsets.softwareQuaternion(sensorIdx),
                                                          correctedLogicalWristQuaternion());
    targetPhysical.normalizeInPlace();
    return computeMinorRotationCompensation(currentDefaultFixed, targetPhysical);
  }
#else
  (void)sensorIdx;
#endif
  return computeMinorRotationCompensation(currentDefaultFixed, gag::Quaternion());
}

static void printQuaternionWxyz(const char* prefix, const gag::Quaternion& qIn) {
  gag::Quaternion q = qIn;
  q.normalizeInPlace();
  Serial.printf("%s{ w=%.5f, x=%.5f, y=%.5f, z=%.5f }\n", prefix, q.w, q.x, q.y, q.z);
}

static const char* recognizerSensorEnumToken(gag::Sensor sensor) {
  switch (sensor) {
    case gag::Sensor::WRIST: return "gag::Sensor::WRIST";
    case gag::Sensor::THUMB: return "gag::Sensor::THUMB";
    case gag::Sensor::INDEX: return "gag::Sensor::INDEX";
    case gag::Sensor::MIDDLE: return "gag::Sensor::MIDDLE";
    case gag::Sensor::RING: return "gag::Sensor::RING";
    case gag::Sensor::LITTLE: return "gag::Sensor::LITTLE";
    default: return "gag::Sensor::WRIST";
  }
}

static bool physicalSensorIndexForRecognizerSensor(gag::Sensor sensor, uint8_t& sensorIdx) {
  switch (sensor) {
    case gag::Sensor::THUMB: sensorIdx = SENSOR_THUMB; return true;
    case gag::Sensor::INDEX: sensorIdx = SENSOR_INDEX; return true;
    case gag::Sensor::MIDDLE: sensorIdx = SENSOR_MIDDLE; return true;
    case gag::Sensor::RING: sensorIdx = SENSOR_RING; return true;
    case gag::Sensor::LITTLE: sensorIdx = SENSOR_LITTLE; return true;
    default: return false;
  }
}

static bool capturePoseQuaternionForRecognizerSensor(gag::Sensor sensor,
                                                     gag::Quaternion& absoluteQ,
                                                     gag::Quaternion& relativeQ,
                                                     bool& haveRelative) {
  haveRelative = false;
  if (sensor == gag::Sensor::WRIST) {
    if (!selectedWristQuaternionAvailable()) return false;
    absoluteQ = recognitionLogicalWristQuaternion();
    relativeQ = absoluteQ;
    return true;
  }

  uint8_t sensorIdx = SENSOR_THUMB;
  if (!physicalSensorIndexForRecognizerSensor(sensor, sensorIdx)) return false;
  if (!physicalSensorQuaternionAvailable(sensorIdx)) return false;

  absoluteQ = recognitionQuaternionForPhysicalSensor(sensorIdx);
  relativeQ = absoluteQ;
  if (selectedWristQuaternionAvailable()) {
    relativeQ = gag::Quaternion::mul(recognitionLogicalWristQuaternion().inverseUnit(), absoluteQ);
    relativeQ.normalizeInPlace();
    haveRelative = true;
  }
  return true;
}

static void printCapturedQuaternion(const gag::Quaternion& qIn) {
  gag::Quaternion q = qIn;
  q.normalizeInPlace();
  Serial.printf("gag::Quaternion(%.8ff, %.8ff, %.8ff, %.8ff)\n",
                q.w, q.x, q.y, q.z);
}

static void printCapturedGestureQuaternions() {
  static const gag::Sensor sensors[] = {
    gag::Sensor::WRIST,
    gag::Sensor::THUMB,
    gag::Sensor::INDEX,
    gag::Sensor::MIDDLE,
    gag::Sensor::RING,
    gag::Sensor::LITTLE
  };

  Serial.println("CAPTURED_GESTURE_QUATERNIONS_BEGIN");
  if (selectedWristQuaternionAvailable()) {
    Serial.printf("// logical wrist source: %s\n", SENSOR_OFFSET_LABELS[selectedWristQuaternionPhysicalSensor()]);
  } else {
    Serial.println("// logical wrist source unavailable; relative finger captures omitted");
  }

  for (uint8_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); ++i) {
    gag::Quaternion absoluteQ;
    gag::Quaternion relativeQ;
    bool haveRelative = false;
    const gag::Sensor sensor = sensors[i];
    if (!capturePoseQuaternionForRecognizerSensor(sensor, absoluteQ, relativeQ, haveRelative)) {
      Serial.printf("// %s unavailable\n", recognizerSensorEnumToken(sensor));
      continue;
    }

    Serial.printf("// %s absolute\n", recognizerSensorEnumToken(sensor));
    printCapturedQuaternion(absoluteQ);
    if (sensor != gag::Sensor::WRIST && haveRelative) {
      Serial.printf("// %s relative_to_wrist\n", recognizerSensorEnumToken(sensor));
      printCapturedQuaternion(relativeQ);
    }
  }
  Serial.println("CAPTURED_GESTURE_QUATERNIONS_END");
}

static void printMinorRotationOffsetArray(const gag::Quaternion* offsets, const char* title) {
  if (title && title[0]) Serial.println(title);
  Serial.println("static gag::Quaternion g_minorRotationOffset[SENSOR_COUNT_ALL] = {");
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    gag::Quaternion q = offsets[s];
    q.normalizeInPlace();
    Serial.printf("  gag::Quaternion(%.8ff, %.8ff, %.8ff, %.8ff), // %s\n",
                  q.w, q.x, q.y, q.z, SENSOR_OFFSET_LABELS[s]);
  }
  Serial.println("};");
}

static void maybePrintMinorRotationOffsetCandidates() {
#if GAG_PRINT_MINOR_ROTATION_OFFSET_CANDIDATES && !GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastMinorRotationOffsetPrintMs) < (uint32_t)GAG_MINOR_ROTATION_OFFSET_PRINT_INTERVAL_MS) return;
  g_lastMinorRotationOffsetPrintMs = now;

  gag::Quaternion currentPose[SENSOR_COUNT_ALL];
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    currentPose[s] = gag::Quaternion();
    if (!physicalSensorQuaternionAvailable(s)) continue;
    currentPose[s] = applyDefaultSensorRotation(s, rawQuaternionForPhysicalSensor(s));
  }
  printMinorRotationOffsetArray(currentPose, "Current physical-fixed quaternions:");
#endif
}

static bool consumeTtgoLeftButtonPress() {
  const bool pressed = readTtgoLeftButtonPressed();
  const uint32_t now = millis();
  const bool risingEdge = pressed && !g_leftButtonPrevPressed;
  g_leftButtonPrevPressed = pressed;
  if (!risingEdge) return false;
  if (((uint32_t)(now - g_leftButtonLastTriggerMs)) < (uint32_t)GAG_TTGO_BUTTON_DEBOUNCE_MS) return false;
  g_leftButtonLastTriggerMs = now;
  return true;
}

static void resetSensorRuntimeOrientationState() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    g_sensorFusionQuat[i] = gag::Quaternion();
    g_sensorFusionInitialized[i] = false;
    g_lastAccelBody[i] = Vec3{ 0.0f, 0.0f, 0.0f };
    g_lastAccelBodyValid[i] = false;
    lastT[i] = now;
  }
  wristMagRaw = Vec3{ 0, 0, 0 };
  gy511Accel_g = Vec3{ 0, 0, 0 };
  gy511MagRaw = Vec3{ 0, 0, 0 };
  gy511LastT = 0;
  g_wristGy25RuntimeBiasDegX = 0.0f;
  g_wristGy25RuntimeBiasDegY = 0.0f;
  g_wristGy25RuntimeBiasDegZ = 0.0f;
  g_wristGy25StillSinceMs = 0;
  g_lastWristGy25BiasLogMs = 0;
  g_lastSoftSensorResetMs = now;
  g_lastSimultaneousDriftResetMs = now;
  resetContinuousThumbMouseControl();
}

static void warmSensorsAfterSoftReset(uint8_t loops = 20) {
  for (uint8_t warm = 0; warm < loops; ++warm) {
    for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) updateOneIMU(i);
    updateWristMagYaw();
    updateGY511();
    delay(10);
  }
}

static void performSensorHardRotationReset() {
  g_lastSoftSensorResetMs = millis();
  Serial.println("Hard sensor rotation reset requested.");
  g_viz.pushLog("HARD RESET");

  gag::Quaternion oldMinor[SENSOR_COUNT_ALL];
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    oldMinor[s] = g_minorRotationOffset[s];
  }

  resetSensorRuntimeOrientationState();
  warmSensorsAfterSoftReset(20);

  Serial.println("Hard reset rotation delta:");
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (!physicalSensorQuaternionAvailable(s)) {
      Serial.printf("sensor=%u label=%s unavailable, keeping previous offset\n", (unsigned)s, SENSOR_OFFSET_LABELS[s]);
      continue;
    }

    const gag::Quaternion qAvg = averageCurrentSensorQuat(s, false, 16);
    gag::Quaternion newMinor = computeMinorRotationCompensation(qAvg, gag::Quaternion());
    newMinor.normalizeInPlace();
    g_minorRotationOffset[s] = newMinor;

    gag::Quaternion oldCombined = gag::Quaternion::mul(DEFAULT_SENSOR_ROTATION[s], oldMinor[s]);
    gag::Quaternion newCombined = gag::Quaternion::mul(DEFAULT_SENSOR_ROTATION[s], newMinor);
    gag::Quaternion delta = gag::Quaternion::mul(oldCombined.inverseUnit(), newCombined);
    oldCombined.normalizeInPlace();
    newCombined.normalizeInPlace();
    delta.normalizeInPlace();

    Serial.printf("sensor=%u label=%s\n", (unsigned)s, SENSOR_OFFSET_LABELS[s]);
    printQuaternionWxyz("  old_combined = ", oldCombined);
    printQuaternionWxyz("  new_combined = ", newCombined);
    printQuaternionWxyz("  delta        = ", delta);
  }

  printMinorRotationOffsetArray(g_minorRotationOffset, "Updated minor rotation offsets:");
  autoCaptureSoftwareNeutralOffsets();
  resetSensorRuntimeOrientationState();
  warmSensorsAfterSoftReset(8);
  Serial.println("Hard sensor rotation reset finished.");
}

static gag::Quaternion expectedSoftResetRotationForPose(uint8_t poseIdx, uint8_t sensorIdx) {
  if (poseIdx >= (uint8_t)EXPECTED_SOFT_RESET_POSE_COUNT) return gag::Quaternion();
  if (sensorIdx >= SENSOR_COUNT_ALL) return gag::Quaternion();
  gag::Quaternion q = g_expectedSoftResetRotations[poseIdx][sensorIdx];
  q.normalizeInPlace();
  return q;
}

static uint8_t selectClosestExpectedSoftResetPose(uint8_t sensorMask) {
  float bestAverageDeg = 1.0e9f;
  uint8_t bestPose = (uint8_t)EXPECTED_SOFT_RESET_POSE_DEFAULT;

  for (uint8_t poseIdx = 0; poseIdx < (uint8_t)EXPECTED_SOFT_RESET_POSE_COUNT; ++poseIdx) {
    float sumDeg = 0.0f;
    uint8_t sampleCount = 0;
    for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
      if ((sensorMask & sensorBitMask(s)) == 0u) continue;
      if (!physicalSensorQuaternionAvailable(s)) continue;

      const gag::Quaternion currentPhysicalFixed = physicalFixedQuaternionForPhysicalSensor(s);
      const gag::Quaternion expectedRotation = expectedSoftResetRotationForPose(poseIdx, s);
      sumDeg += rad2deg(gag::Quaternion::angularDistance(currentPhysicalFixed, expectedRotation));
      ++sampleCount;
    }

    if (sampleCount == 0u) continue;
    const float averageDeg = sumDeg / (float)sampleCount;
    if (averageDeg < bestAverageDeg) {
      bestAverageDeg = averageDeg;
      bestPose = poseIdx;
    }
  }

  return bestPose;
}

static bool performSensorSoftRotationResetForMaskToPose(uint8_t sensorMask, uint8_t poseIdx, bool logToViz = true) {
  if (sensorMask == 0u) return false;
  if (poseIdx >= (uint8_t)EXPECTED_SOFT_RESET_POSE_COUNT) {
    poseIdx = (uint8_t)EXPECTED_SOFT_RESET_POSE_DEFAULT;
  }

  g_lastSoftSensorResetMs = millis();
  ++g_softResetOperationIndex;
  Serial.printf("Soft sensor rotation reset requested idx=%lu mask=0x%02X pose=%u(%s).\n",
                (unsigned long)g_softResetOperationIndex,
                (unsigned)sensorMask,
                (unsigned)poseIdx,
                EXPECTED_SOFT_RESET_POSE_LABELS[poseIdx]);
  if (logToViz) {
    char softResetLog[28];
    snprintf(softResetLog, sizeof(softResetLog), "%lu SOFT %s",
             (unsigned long)g_softResetOperationIndex,
             EXPECTED_SOFT_RESET_POSE_LABELS[poseIdx]);
    g_viz.pushLog(softResetLog);
  }

  bool changedAny = false;
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if ((sensorMask & sensorBitMask(s)) == 0u) continue;
    if (!physicalSensorQuaternionAvailable(s)) {
      Serial.printf("sensor=%u label=%s unavailable, keeping previous offsets\n",
                    (unsigned)s,
                    SENSOR_OFFSET_LABELS[s]);
      continue;
    }

    const gag::Quaternion currentDefaultFixed = applyDefaultSensorRotation(s, rawQuaternionForPhysicalSensor(s));
    const gag::Quaternion targetExpectedRotation = expectedSoftResetRotationForPose(poseIdx, s);
    const gag::Quaternion oldMinor = g_minorRotationOffset[s];
    const gag::Quaternion oldSw = g_offsets.softwareQuaternion(s);
    gag::Quaternion newMinor = computeMinorRotationCompensation(currentDefaultFixed, targetExpectedRotation);
    newMinor.normalizeInPlace();
    g_minorRotationOffset[s] = newMinor;
    g_offsets.setSoftwareQuaternion(s, gag::Quaternion());
    changedAny = true;

    Serial.printf("sensor=%u label=%s\n", (unsigned)s, SENSOR_OFFSET_LABELS[s]);
    printQuaternionWxyz("  default_fixed    = ", currentDefaultFixed);
    printQuaternionWxyz("  target_expected  = ", targetExpectedRotation);
    printQuaternionWxyz("  old_minor        = ", oldMinor);
    printQuaternionWxyz("  new_minor        = ", newMinor);
    printQuaternionWxyz("  old_sw           = ", oldSw);
    printQuaternionWxyz("  new_sw           = ", gag::Quaternion());
  }

  if (changedAny) {
    clearPendingLeftClick();
    resetContinuousThumbMouseControl();
  }
  Serial.println("Soft sensor rotation reset finished.");
  return changedAny;
}

static bool performSensorSoftRotationResetForMask(uint8_t sensorMask, bool logToViz = true) {
  return performSensorSoftRotationResetForMaskToPose(sensorMask, (uint8_t)EXPECTED_SOFT_RESET_POSE_DEFAULT, logToViz);
}

static void resetSimultaneousDriftResetTracking(uint8_t sensorMask = 0xFFu);

static void performSensorSoftRotationReset() {
  (void)performSensorSoftRotationResetForMask(0xFFu, true);
  resetSimultaneousDriftResetTracking(0xFFu);
}

static void performPeriodicSensorSoftRotationReset() {
  const uint8_t poseIdx = selectClosestExpectedSoftResetPose(0xFFu);
  Serial.printf("Periodic soft sensor rotation reset selected pose=%u(%s).\n",
                (unsigned)poseIdx,
                EXPECTED_SOFT_RESET_POSE_LABELS[poseIdx]);
  (void)performSensorSoftRotationResetForMaskToPose(0xFFu, poseIdx, true);
  resetSimultaneousDriftResetTracking(0xFFu);
}

static void resetSimultaneousDriftResetTracking(uint8_t sensorMask) {
  const uint32_t now = millis();
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if ((sensorMask & sensorBitMask(s)) == 0u) continue;
    g_driftResetLastPhysicalFixed[s] = gag::Quaternion();
    g_driftResetLastPhysicalFixedValid[s] = false;
    g_driftResetStillSinceMs[s] = now;
    g_driftResetActive[s] = false;
  }
}

static void updateSimultaneousDriftReset() {
#if GAG_ENABLE_SIMULTANEOUS_DRIFT_RESET
  const uint8_t enabledSensorMask = (uint8_t)(simultaneousDriftResetCompileTimeMask() & (uint8_t)GAG_SIMULTANEOUS_DRIFT_RESET_SENSOR_MASK);
  if (enabledSensorMask == 0u) return;
  const uint32_t intervalMs = (uint32_t)GAG_SIMULTANEOUS_DRIFT_RESET_INTERVAL_MS;
  if (intervalMs == 0u) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastSimultaneousDriftResetMs) < intervalMs) return;

  const float dtSec = g_lastSimultaneousDriftResetMs == 0u
                        ? ((float)intervalMs * 0.001f)
                        : ((float)(uint32_t)(now - g_lastSimultaneousDriftResetMs) * 0.001f);
  g_lastSimultaneousDriftResetMs = now;

  const float movementThresholdDeg = (float)GAG_SIMULTANEOUS_DRIFT_RESET_MOVEMENT_THRESHOLD_DEG;
  const float deadbandDeg = (float)GAG_SIMULTANEOUS_DRIFT_RESET_DEADBAND_DEG;
  const float maxCorrectionDeg = (float)GAG_SIMULTANEOUS_DRIFT_RESET_MAX_CORRECTION_DEG;
  const float blend = clamp01((float)GAG_SIMULTANEOUS_DRIFT_RESET_RATE_PER_SEC * dtSec);
  if (blend <= 0.0f) return;

  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    g_driftResetActive[s] = false;
    if ((enabledSensorMask & sensorBitMask(s)) == 0u) continue;
    if (!g_enableDriftReset[s]) {
      g_driftResetLastPhysicalFixedValid[s] = false;
      g_driftResetStillSinceMs[s] = now;
      continue;
    }
    if (!physicalSensorQuaternionAvailable(s)) {
      g_driftResetLastPhysicalFixedValid[s] = false;
      g_driftResetStillSinceMs[s] = now;
      continue;
    }

    const gag::Quaternion currentTrackingQuaternion = simultaneousDriftResetTrackingQuaternionForSensor(s);
    if (!g_driftResetLastPhysicalFixedValid[s]) {
      g_driftResetLastPhysicalFixed[s] = currentTrackingQuaternion;
      g_driftResetLastPhysicalFixedValid[s] = true;
      g_driftResetStillSinceMs[s] = now;
      continue;
    }

    const float movementDeg = rad2deg(gag::Quaternion::angularDistance(g_driftResetLastPhysicalFixed[s], currentTrackingQuaternion));
    g_driftResetLastPhysicalFixed[s] = currentTrackingQuaternion;
    if (movementDeg > movementThresholdDeg) {
      g_driftResetStillSinceMs[s] = now;
      continue;
    }
    if ((uint32_t)(now - g_driftResetStillSinceMs[s]) < (uint32_t)GAG_SIMULTANEOUS_DRIFT_RESET_STILL_MS) continue;

    const gag::Quaternion currentCorrected = simultaneousDriftResetCorrectedQuaternionForSensor(s);
    const float correctionDeg = rad2deg(gag::Quaternion::angularDistance(currentCorrected, gag::Quaternion()));
    if (correctionDeg <= deadbandDeg) continue;
    if (maxCorrectionDeg > 0.0f && correctionDeg > maxCorrectionDeg) continue;

    const gag::Quaternion currentDefaultFixed = applyDefaultSensorRotation(s, rawQuaternionForPhysicalSensor(s));
    const gag::Quaternion currentMinor = g_minorRotationOffset[s];
    const gag::Quaternion targetMinor = simultaneousDriftResetTargetMinorForSensor(s, currentDefaultFixed);
    g_minorRotationOffset[s] = quatNlerp(currentMinor, targetMinor, blend);
    g_driftResetActive[s] = true;
  }
#endif
}

static void scheduleGestureSoftReset(uint8_t sensorMask, uint32_t dueMs) {
  if (sensorMask == 0u) return;
  for (uint8_t i = 0; i < GAG_MAX_PENDING_GESTURE_SOFT_RESETS; ++i) {
    if (g_pendingGestureSoftResetMask[i] != 0u && g_pendingGestureSoftResetDueMs[i] == dueMs) {
      g_pendingGestureSoftResetMask[i] |= sensorMask;
      return;
    }
  }
  for (uint8_t i = 0; i < GAG_MAX_PENDING_GESTURE_SOFT_RESETS; ++i) {
    if (g_pendingGestureSoftResetMask[i] == 0u) {
      g_pendingGestureSoftResetMask[i] = sensorMask;
      g_pendingGestureSoftResetDueMs[i] = dueMs;
      return;
    }
  }
  uint8_t fallbackIdx = 0u;
  for (uint8_t i = 1; i < GAG_MAX_PENDING_GESTURE_SOFT_RESETS; ++i) {
    if ((int32_t)(g_pendingGestureSoftResetDueMs[i] - g_pendingGestureSoftResetDueMs[fallbackIdx]) > 0) {
      fallbackIdx = i;
    }
  }
  g_pendingGestureSoftResetMask[fallbackIdx] |= sensorMask;
  g_pendingGestureSoftResetDueMs[fallbackIdx] = dueMs;
}

static void processPendingGestureSoftResets() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < GAG_MAX_PENDING_GESTURE_SOFT_RESETS; ++i) {
    const uint8_t mask = g_pendingGestureSoftResetMask[i];
    if (mask == 0u) continue;
    if ((int32_t)(now - g_pendingGestureSoftResetDueMs[i]) < 0) continue;
    g_pendingGestureSoftResetMask[i] = 0u;
    g_pendingGestureSoftResetDueMs[i] = 0;
    performSensorSoftRotationResetForMask(mask, false);
    resetSimultaneousDriftResetTracking(mask);
  }
}

static void scheduleWristMouseToggleSoftReset(uint32_t dueMs) {
  g_pendingWristMouseToggleSoftReset = true;
  g_pendingWristMouseToggleSoftResetDueMs = dueMs;
}

static void processPendingWristMouseToggleSoftReset() {
  if (!g_pendingWristMouseToggleSoftReset) return;
  const uint32_t now = millis();
  if ((int32_t)(now - g_pendingWristMouseToggleSoftResetDueMs) < 0) return;
  g_pendingWristMouseToggleSoftReset = false;
  g_pendingWristMouseToggleSoftResetDueMs = 0;
  performSensorSoftRotationReset();
}

static uint8_t physicalSensorMaskForRecognizerSensor(gag::Sensor sensor) {
  switch (sensor) {
    case gag::Sensor::WRIST:
      return (uint8_t)(sensorBitMask(SENSOR_WRIST_GY25) |
                       sensorBitMask(SENSOR_WRIST_MPU9250) |
                       sensorBitMask(SENSOR_WRIST_GY511));
    case gag::Sensor::THUMB:  return sensorBitMask(SENSOR_THUMB);
    case gag::Sensor::INDEX:  return sensorBitMask(SENSOR_INDEX);
    case gag::Sensor::MIDDLE: return sensorBitMask(SENSOR_MIDDLE);
    case gag::Sensor::RING:   return sensorBitMask(SENSOR_RING);
    case gag::Sensor::LITTLE: return sensorBitMask(SENSOR_LITTLE);
    default:                  return 0u;
  }
}

static uint8_t physicalSensorMaskForGestureSoftReset(const gag::RecognizedGesture& gr) {
  uint8_t mask = 0u;
  const uint8_t count = (gr.softResetLen <= (uint8_t)gag::Sensor::COUNT)
                          ? gr.softResetLen
                          : (uint8_t)gag::Sensor::COUNT;
  for (uint8_t i = 0; i < count; ++i) {
    mask |= physicalSensorMaskForRecognizerSensor(gr.softReset[i]);
  }
  return mask;
}

static void maybeHandleTtgoLeftButtonHardReset() {
  if (!consumeTtgoLeftButtonPress()) return;
  Serial.println("TTGO left button press detected.");
#if GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
  if (g_printQuaternionsOnLeftClick) {
    printCapturedGestureQuaternions();
    g_viz.pushLog("POSE CAPTURED");
  } else {
    Serial.println("Quaternion capture is off; left button hard reset is disabled in capture mode.");
    g_viz.pushLog("POSE CAP OFF");
  }
  return;
#endif
  g_wristMouseEmulationEnabled = false;
  syncDriftResetEnableState();
  clearPendingLeftClick();
  resetContinuousThumbMouseControl();
  g_viz.pushLog("WRIST MOUSE OFF");
  performSensorHardRotationReset();
}

static void maybeHandlePeriodicSoftSensorReset() {
#if GAG_PERIODIC_SOFT_SENSOR_RESET
  const uint32_t intervalMs = (uint32_t)GAG_PERIODIC_SOFT_SENSOR_RESET_INTERVAL_MS;
  if (intervalMs == 0u) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastSoftSensorResetMs) < intervalMs) return;
  Serial.printf("Periodic soft sensor rotation reset triggered interval_ms=%lu\n",
                (unsigned long)intervalMs);
  g_viz.pushLog("AUTO RESET");
  performPeriodicSensorSoftRotationReset();
#endif
}

static bool consumeTtgoRightButtonPress() {
  const bool pressed = readTtgoRightButtonPressed();
  const uint32_t now = millis();
  const bool risingEdge = pressed && !g_rightButtonPrevPressed;
  g_rightButtonPrevPressed = pressed;
  if (!risingEdge) return false;
  if (((uint32_t)(now - g_rightButtonLastTriggerMs)) < (uint32_t)GAG_TTGO_BUTTON_DEBOUNCE_MS) return false;
  g_rightButtonLastTriggerMs = now;
  return true;
}

static void maybeHandleTtgoRightButtonMouseToggle() {
  if (!consumeTtgoRightButtonPress()) return;
  Serial.println("TTGO right button press detected.");
#if GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
  g_printQuaternionsOnLeftClick = !g_printQuaternionsOnLeftClick;
  syncDriftResetEnableState();
  clearPendingLeftClick();
  Serial.printf("Quaternion capture on left click %s.\n",
                g_printQuaternionsOnLeftClick ? "enabled" : "disabled");
  g_viz.pushLog(g_printQuaternionsOnLeftClick ? "POSE CAP ON" : "POSE CAP OFF");
#else
#if GAG_ENABLE_BLE_MOUSE
  g_bleMouseSendEnabled = !g_bleMouseSendEnabled;
  Serial.printf("BLE mouse sending %s.\n", g_bleMouseSendEnabled ? "enabled" : "disabled");
  g_viz.pushLog(g_bleMouseSendEnabled ? "BLE SEND ON" : "BLE SEND OFF");
#if GAG_DISABLE_VIBRATION_WHEN_BLE_SEND_OFF
  if (!g_bleMouseSendEnabled) stopAllVibrations();
#endif
#else
  Serial.println("BLE mouse support is unavailable at compile time.");
#endif
#endif
}

static void printRotationOffsetsAtBoot() {
  Serial.println("Rotation offsets:");
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (!isSensorEnabled(s)) continue;
    Serial.printf("sensor=%u label=%s\n", (unsigned)s, SENSOR_OFFSET_LABELS[s]);
    printQuaternionWxyz("  ideal   = ", gag::Quaternion());
    printQuaternionWxyz("  default = ", DEFAULT_SENSOR_ROTATION[s]);
    printQuaternionWxyz("  minor   = ", g_minorRotationOffset[s]);
    printQuaternionWxyz("  combined= ", gag::Quaternion::mul(DEFAULT_SENSOR_ROTATION[s], g_minorRotationOffset[s]));
  }
}

static void captureMinorRotationOffsetsAtBoot() {
#if GAG_AUTO_CAPTURE_MINOR_ROTATION_FIX
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (!physicalSensorQuaternionAvailable(s)) continue;
    const gag::Quaternion qAvg = averageCurrentSensorQuat(s, false, 16);
    g_minorRotationOffset[s] = computeMinorRotationCompensation(qAvg, gag::Quaternion());
  }
#endif
}

static void autoCaptureSoftwareNeutralOffsets() {
#if GAG_AUTO_CAPTURE_SW_NEUTRAL
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (!physicalSensorQuaternionAvailable(s)) continue;
    const gag::Quaternion qAvg = averageCurrentSensorQuat(s, true, 16);
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
                           const gag::GestureAction& action,
                           bool relativeToWrist = false) {
  gag::GestureDef g;
  strncpy(g.name, name, sizeof(g.name) - 1);
  strncpy(g.command, command, sizeof(g.command) - 1);
  strncpy(g.label, label, sizeof(g.label) - 1);
  g.threshold_rad = deg2rad(thresholdDeg);
  g.recognition_delay_ms = cooldownMs;
  g.max_time_ms = maxTimeMs;
  g.relative = relativeToWrist;
  g.active = true;
  g.action = action;
  if (relativeToWrist) {
    g.perSensor[(uint8_t)gag::Sensor::WRIST].len = 1;
    g.perSensor[(uint8_t)gag::Sensor::WRIST].q[0] = gag::Quaternion();
  }
  g.perSensor[(uint8_t)sensor].len = 2;
  g.perSensor[(uint8_t)sensor].q[0] = target;
  g.perSensor[(uint8_t)sensor].q[1] = target;
  g_recognizer.addGesture(g);
}

static void addPoseGesture1(const char* name,
                           const char* command,
                           const char* label,
                           gag::Sensor sensor,
                           const gag::Quaternion& target,
                           float thresholdDeg,
                           uint32_t cooldownMs,
                           uint32_t maxTimeMs,
                           const gag::GestureAction& action,
                           bool relativeToWrist = false) {
  gag::GestureDef g;
  strncpy(g.name, name, sizeof(g.name) - 1);
  strncpy(g.command, command, sizeof(g.command) - 1);
  strncpy(g.label, label, sizeof(g.label) - 1);
  g.threshold_rad = deg2rad(thresholdDeg);
  g.recognition_delay_ms = cooldownMs;
  g.max_time_ms = maxTimeMs;
  g.relative = relativeToWrist;
  g.active = true;
  g.action = action;
  if (relativeToWrist) {
    g.perSensor[(uint8_t)gag::Sensor::WRIST].len = 1;
    g.perSensor[(uint8_t)gag::Sensor::WRIST].q[0] = gag::Quaternion();
  }
  g.perSensor[(uint8_t)sensor].len = 1;
  g.perSensor[(uint8_t)sensor].q[0] = target;
  g_recognizer.addGesture(g);
}

static void addPoseGesture2(const char* name,
                           const char* command,
                           const char* label,
                           gag::Sensor sensor,
                           const gag::Quaternion& target,
                           const gag::Quaternion& target2,
                           float thresholdDeg,
                           uint32_t cooldownMs,
                           uint32_t maxTimeMs,
                           const gag::GestureAction& action,
                           bool relativeToWrist = false) {
  gag::GestureDef g;
  strncpy(g.name, name, sizeof(g.name) - 1);
  strncpy(g.command, command, sizeof(g.command) - 1);
  strncpy(g.label, label, sizeof(g.label) - 1);
  g.threshold_rad = deg2rad(thresholdDeg);
  g.recognition_delay_ms = cooldownMs;
  g.max_time_ms = maxTimeMs;
  g.relative = relativeToWrist;
  g.softResetLen = 2;
  g.softReset[0] = gag::Sensor::INDEX;
  g.softReset[1] = gag::Sensor::WRIST;
  g.softResetDelayMs = (uint32_t)GAG_DEFAULT_GESTURE_SOFT_RESET_DELAY_MS;

  g.active = true;
  g.action = action;
  if (relativeToWrist) {
    g.perSensor[(uint8_t)gag::Sensor::WRIST].len = 1;
    g.perSensor[(uint8_t)gag::Sensor::WRIST].q[0] = gag::Quaternion();
  }
  g.perSensor[(uint8_t)sensor].len = 3;
  g.perSensor[(uint8_t)sensor].q[0] = gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 0.0f);
  g.perSensor[(uint8_t)sensor].q[1] = target;
  g.perSensor[(uint8_t)sensor].q[2] = target2;
  g_recognizer.addGesture(g);
}


static void installDefaultGestures() {
  using gag::MouseActionType;

  // Little finger up -> visualization mode cycle + blink + vibration.
  // {
  //   gag::GestureAction a;
  //   a.switch_visualization_mode = true;
  //   a.blink_visualization = true;
  //   a.blink_color565 = TFT_ORANGE;
  //   a.vibrate = true;
  //   a.vibrate_sensor_mask = (1u << SENSOR_LITTLE);
  //   a.vibrate_duration_ms = 220;
  //   addPoseGesture("little_up_mode", "GAG_CYCLE_MODE", "MODE",
  //                  gag::Sensor::LITTLE,
  //                  gag::Quaternion::fromAxisAngleDeg(1, 0, 0, -28.0f),
  //                  18.0f, 700, 1200, a);
  // }

  // Index down -> left click.
  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_GREEN;
    a.mouse.type = MouseActionType::CLICK;
    a.mouse.button = MOUSE_LEFT;
    a.vibrate = false;
    a.vibrate_sensor_mask = (1u << SENSOR_INDEX);
    a.vibrate_duration_ms = 140;
    addPoseGesture2("index_left_click", "MOUSE_LEFT_CLICK", "LCLK",
                   gag::Sensor::INDEX,
                  //  gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 25.0f),
                  //  gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 25.0f),
                  //  gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 25.0f),
                  // gag::Sensor::INDEX absolute
                  // gag::Quaternion(0.99992257f, 0.01183951f, -0.00383035f, -0.00002276f),
                  // gag::Quaternion(0.99992257f, 0.01183951f, -0.00383035f, -0.00002276f),
                  // gag::Quaternion(0.99992245f, 0.01200788f, -0.00322799f, -0.00076021f),
                  // gag::Quaternion(0.99992245f, 0.01200788f, -0.00322799f, -0.00076021f),
                  // gag::Quaternion(0.93727487f, 0.34785417f, 0.01777399f, -0.01404932f)
                  gag::Quaternion(0.93932605f, 0.34259737f, 0.01534605f, -0.00762683f),
                  gag::Quaternion(0.93932605f, 0.34259737f, 0.01534605f, -0.00762683f),
                  15.0f, 250, 300, a, true);
  }

  // Ring down -> right click, aligned with the left-click angle definition.
  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_MAGENTA;
    a.mouse.type = MouseActionType::CLICK;
    a.mouse.button = MOUSE_RIGHT;
                       gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 25.0f),
    addPoseGesture2("ring_right_click", "MOUSE_RIGHT_CLICK", "RCLK",
                  gag::Sensor::RING,
                  //  gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 25.0f),
                  //  gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 25.0f),
                  //  gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 25.0f),
                  // gag::Quaternion(0.91867083f, 0.36967331f, -0.04149037f, -0.13290632f)
// gag::Sensor::RING relative_to_wrist
                  gag::Quaternion(0.92188686f, 0.36352029f, -0.03977941f, -0.12804425f),
                  gag::Quaternion(0.92188686f, 0.36352029f, -0.03977941f, -0.12804425f),
                  15.0f, 250, 300, a, true);
  }

  // Middle up/down -> continuous wheel scroll.
  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_CYAN;
    a.mouse.type = MouseActionType::SCROLL;
    a.mouse.wheel = 1;
    addPoseGesture1("middle_scroll_up", "MOUSE_SCROLL_UP", "SCRU",
                   gag::Sensor::MIDDLE,
                   gag::Quaternion::fromAxisAngleDeg(1, 0, 0, -40.0f),
                   18.0f, 80, 220, a, true);
  }

  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_CYAN;
    a.mouse.type = MouseActionType::SCROLL;
    a.mouse.wheel = -1;
    addPoseGesture1("middle_scroll_down", "MOUSE_SCROLL_DOWN", "SCRD",
                   gag::Sensor::MIDDLE,
                   gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 40.0f),
                   18.0f, 80, 220, a, true);
  }

  // Little down -> left double click.
  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_GREEN;
    a.mouse.type = MouseActionType::DOUBLE_CLICK;
    a.mouse.button = MOUSE_LEFT;
    addPoseGesture2("little_left_double_click", "MOUSE_LEFT_DOUBLE_CLICK", "LDCLK",
                   gag::Sensor::LITTLE,
                   gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 30.0f),
                   gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 0.0f),
                   15.0f, 250, 300, a, true);
  }

  // Little up -> wheel button click.
  {
    gag::GestureAction a;
    a.blink_visualization = true;
    a.blink_color565 = TFT_ORANGE;
    a.mouse.type = MouseActionType::CLICK;
    a.mouse.button = MOUSE_MIDDLE;
    addPoseGesture2("little_middle_click", "MOUSE_MIDDLE_CLICK", "MCLK",
                   gag::Sensor::LITTLE,
                   gag::Quaternion::fromAxisAngleDeg(1, 0, 0, -30.0f),
                   gag::Quaternion::fromAxisAngleDeg(1, 0, 0, 0.0f),
                   15.0f, 250, 300, a, true);
  }

  // Thumb right gesture toggles wrist mouse emulation.
  {
    gag::GestureAction a;
    a.toggle_wrist_mouse_emulation = true;
    a.blink_visualization = true;
    a.blink_color565 = TFT_CYAN;
    addPoseGesture("thumb_toggle_wrist_mouse", "TOGGLE_WRIST_MOUSE", "WMOU",
                   gag::Sensor::THUMB,
                   gag::Quaternion::fromAxisAngleDeg(0, 0, 1, -34.0f),
                   18.0f, 450, 900, a, true);
  }

  // Thumb mouse movement now uses continuous control driven from the
  // corrected wrist pose in updateContinuousThumbMouseControl().
}

// =====================
// Gesture callback
// =====================
static void onGestureRecognized(const gag::RecognizedGesture& gr) {
  const char* label = (gr.label && gr.label[0]) ? gr.label : ((gr.name && gr.name[0]) ? gr.name : "GEST");
#if !GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
  char logLine[28];
  snprintf(logLine, sizeof(logLine), "%lu %s", (unsigned long)(millis() % 10000UL), label);
  if (!gr.action || gr.action->log_to_history) {
    g_viz.pushLog(logLine);
  }
#endif

  if (gr.action) {
    const uint32_t nowMs = millis();
#if !GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
    const uint8_t softResetMask = physicalSensorMaskForGestureSoftReset(gr);
    if (softResetMask != 0u) {
#if GAG_ENABLE_DELAYED_GESTURE_SOFT_RESET
      scheduleGestureSoftReset(softResetMask, nowMs + gr.softResetDelayMs);
#else
      performSensorSoftRotationResetForMask(softResetMask, false);
      resetSimultaneousDriftResetTracking(softResetMask);
#endif
    }
#endif
    if (gr.action->mouse.type == gag::MouseActionType::CLICK && gr.action->mouse.button == MOUSE_LEFT) {
      if ((int32_t)(nowMs - g_ignoreSingleLeftClickUntilMs) < 0) return;
      queuePendingLeftClick(nowMs + 120U);
      return;
    }
    if (gr.action->mouse.type == gag::MouseActionType::DOUBLE_CLICK && gr.action->mouse.button == MOUSE_LEFT) {
      clearPendingLeftClick();
      g_ignoreSingleLeftClickUntilMs = nowMs + 220U;
    }
    if (gr.action->switch_visualization_mode) {
      g_viz.nextMode();
    }
    if (gr.action->toggle_wrist_mouse_emulation) {
      g_wristMouseEmulationEnabled = !g_wristMouseEmulationEnabled;
      syncDriftResetEnableState();
      clearPendingLeftClick();
      resetContinuousThumbMouseControl();
      Serial.printf("Wrist mouse emulation %s.\n", g_wristMouseEmulationEnabled ? "enabled" : "disabled");
#if !GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
      g_viz.pushLog(g_wristMouseEmulationEnabled ? "WRIST MOUSE ON" : "WRIST MOUSE OFF");
#endif
      scheduleVibration((uint8_t)(1u << SENSOR_THUMB), g_wristMouseEmulationEnabled ? 60 : 120);
#if GAG_SOFT_RESET_ON_WRIST_MOUSE_TOGGLE && !GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
      scheduleWristMouseToggleSoftReset(nowMs + (uint32_t)GAG_WRIST_MOUSE_TOGGLE_SOFT_RESET_DELAY_MS);
#endif
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
// Feed all finger sensors individually, but only one logical wrist source
// into recognition. The primary wrist source is selected by
// GAG_PRIMARY_WRIST_SENSOR; all physical wrist sensors can still be visualized.
static void feedRecognizerFromCurrentPose() {
  const uint32_t now = millis();
  for (uint8_t s = SENSOR_THUMB; s <= SENSOR_LITTLE; ++s) {
    if (!physicalSensorQuaternionAvailable(s)) continue;
    gag::Quaternion qCorr = recognitionQuaternionForPhysicalSensor(s);
    g_recognizer.processSample(mapToRecognizerSensor(s), qCorr, now);
  }

  if (selectedWristQuaternionAvailable()) {
    g_recognizer.processSample(gag::Sensor::WRIST, recognitionLogicalWristQuaternion(), now);
  }

  // Wrist accel follows the selected/default wrist sensor when available.
  gag::AccelData a;
  const uint8_t wristAccelSensor = selectedWristAccelPhysicalSensor();
  if (wristAccelSensor == SENSOR_WRIST_GY511) {
    Vec3 accelG;
    if (readGY511Accel(accelG)) {
      a = gag::AccelData(accelG.x, accelG.y, accelG.z);
    }
  } else if (isSensorEnabled(wristAccelSensor) && g_sensorInitOk[wristAccelSensor]) {
    pcaSelect(ACTIVE_CHANNELS[wristAccelSensor]);
    uint8_t buf[6] = { 0 };
    i2cReadBytes((wristAccelSensor == SENSOR_WRIST_MPU9250) ? wristMpuAddress() : MPU6050_ADDR, REG_ACCEL_XOUT_H, buf, 6);
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    const gag::offsets::HwOffset6 hw = g_offsets.hardware(wristAccelSensor);
    a = gag::AccelData((float)(ax - hw.ax) / 16384.0f,
                       (float)(ay - hw.ay) / 16384.0f,
                       (float)(az - hw.az) / 16384.0f);
  }
  g_recognizer.processSample(gag::Sensor::WRIST, gag::RecogData::fromAccel(a), now);
}

// =====================
// Visualization input
// =====================
// Populate visualization for every physical sensor cube. This is independent
// from the logical wrist-source selection used by the skeleton and recognizer.
static gag::viz::FrameInput buildVizFrame() {
  gag::viz::FrameInput frame;
  frame.sensor_count = SENSOR_COUNT_ALL;
  frame.hand_relative_rotation = GAG_VIZ_HAND_RELATIVE_ROTATION;
  frame.cube_relative_rotation = GAG_VIZ_CUBES_RELATIVE_ROTATION;
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    const uint8_t vizIdx = sensorToVizSlot(i);
    frame.base_color[vizIdx] = SENSOR_COLORS[i];
    frame.present[vizIdx] = physicalSensorQuaternionAvailable(i);
    frame.drift_reset_active[vizIdx] = g_driftResetActive[i];
  }

  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    if (!physicalSensorQuaternionAvailable(s)) continue;
    const uint8_t vizIdx = sensorToVizSlot(s);
    frame.sensor_q[vizIdx] = GAG_VIZ_CUBES_RELATIVE_ROTATION
                               ? physicalFixedQuaternionForPhysicalSensor(s)
                               : correctedQuaternionForPhysicalSensor(s);
    frame.hand_sensor_q[vizIdx] = GAG_VIZ_HAND_RELATIVE_ROTATION
                                    ? physicalFixedQuaternionForPhysicalSensor(s)
                                    : correctedQuaternionForPhysicalSensor(s);
  }

  const uint8_t wristSensor = selectedWristQuaternionPhysicalSensor();
  const gag::Quaternion wristBaseQ = (GAG_VIZ_HAND_RELATIVE_ROTATION || GAG_VIZ_CUBES_RELATIVE_ROTATION)
                                       ? physicalFixedQuaternionForPhysicalSensor(wristSensor)
                                       : correctedLogicalWristQuaternion();
  frame.hand_wrist_q = applyWristPivotRotationCorrection(wristSensor, wristBaseQ);
  frame.hand_wrist_present = selectedWristQuaternionAvailable();
  frame.hand_wrist_color = selectedLogicalWristColor();
  frame.mouse_dx = g_thumbMouseVizDx;
  frame.mouse_dy = g_thumbMouseVizDy;
#if GAG_ENABLE_BLE_MOUSE
  frame.mouse_send_enabled = g_bleMouseSendEnabled;
  frame.mouse_ble_connected = g_bleMouse.isConnected();
#endif

  return frame;
}
static void resetFusionState() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    g_sensorFusionQuat[i] = gag::Quaternion();
    g_sensorFusionInitialized[i] = false;
    g_sensorInitOk[i] = false;
    g_sensorMpuAddr[i] = 0;
    g_lastAccelBody[i] = Vec3{ 0.0f, 0.0f, 0.0f };
    g_lastAccelBodyValid[i] = false;
    lastT[i] = now;
    g_lastFifoResetMs[i] = 0;
    g_driftResetLastPhysicalFixed[i] = gag::Quaternion();
    g_driftResetLastPhysicalFixedValid[i] = false;
    g_driftResetStillSinceMs[i] = now;
    g_driftResetActive[i] = false;
  }
  wristMagOk = false;
  wristMagRaw = Vec3{ 0, 0, 0 };
  gy511Ok = true;
  gy511MagOk = true;
  gy511Accel_g = Vec3{ 0, 0, 0 };
  gy511MagRaw = Vec3{ 0, 0, 0 };
  gy511LastT = 0;
  g_lastSerialQuatLogMs = 0;
  g_lastMinorRotationOffsetPrintMs = 0;
  g_bleMouseSendEnabled = false;
  g_wristMouseEmulationEnabled = true;
  syncDriftResetEnableState();
  clearPendingLeftClick();
  g_ignoreSingleLeftClickUntilMs = 0;
  g_softResetOperationIndex = 0;
  g_wristGy25RuntimeBiasDegX = 0.0f;
  g_wristGy25RuntimeBiasDegY = 0.0f;
  g_wristGy25RuntimeBiasDegZ = 0.0f;
  g_wristGy25StillSinceMs = 0;
  g_lastWristGy25BiasLogMs = 0;
  g_lastSoftSensorResetMs = now;
  g_lastSimultaneousDriftResetMs = now;
  resetContinuousThumbMouseControl();
}

static void initializeGloveRuntime(bool coldBootLog) {
  resetFusionState();

  tft.fillScreen(TFT_BLACK);
  g_viz.begin(tft, TFT_BLACK);
  g_viz.pushLog(coldBootLog ? "BOOT" : "RESTART");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  pcaReset();

  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    g_offsets.setHardware(i, DEFAULT_HW_OFFSETS[i]);
    g_offsets.setSoftwareQuaternion(i, gag::Quaternion());
  }

  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    if (!isSensorEnabled(i) || !isMpuBackedSensor(i)) continue;
    bool ok = initOneIMU(i);
    g_sensorInitOk[i] = ok;
    lastT[i] = millis();
    if (!ok) {
      Serial.printf("IMU init failed idx=%u ch=%u wrist_addr=0x%02X\n", i, ACTIVE_CHANNELS[i], (unsigned)wristMpuAddress());
      if (i == SENSOR_WRIST_MPU9250) {
        printWristMpuDiagnostic();
      }
    }
    delay(10);
  }

  wristMagOk = g_sensorInitOk[SENSOR_WRIST_MPU9250] && initWristMagAK8963();
  gy511Ok = initGY511();
  g_sensorInitOk[SENSOR_WRIST_GY511] = gy511Ok;
#if GAG_ENABLE_FIFO_BOOT_TEST
  for (uint8_t s = 0; s < SENSOR_COUNT_ALL; ++s) {
    printMpuFifoBootTestForSensor(s);
  }
#endif
#if GAG_ENABLE_FIFO_REPORT
  printFifoCapabilityReport();
#endif

  for (uint8_t warm = 0; warm < 20; ++warm) {
    for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) updateOneIMU(i);
    updateWristMagYaw();
    updateGY511();
    delay(10);
  }

  measureHardwareOffsetsAtBoot();

  for (uint8_t warm = 0; warm < 20; ++warm) {
    for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) updateOneIMU(i);
    updateWristMagYaw();
    updateGY511();
    delay(10);
  }

  captureMinorRotationOffsetsAtBoot();
  printRotationOffsetsAtBoot();
  autoCaptureSoftwareNeutralOffsets();

#if GAG_ENABLE_RECOGNITION
  g_recognizer.begin(Serial);
  g_recognizer.setOnRecognized(onGestureRecognized);
  installDefaultGestures();
#endif

#if GAG_ENABLE_BLE_MOUSE
  g_viz.pushLog("BLE SEND OFF");
#endif
  g_viz.pushLog("READY");
}

// =====================
// Setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(GAG_TTGO_RIGHT_BUTTON_PIN, INPUT);
  pinMode(GAG_PAIR_CONFIRM_BUTTON_PIN, INPUT);
  g_rightButtonPrevPressed = readTtgoRightButtonPressed();
  g_leftButtonPrevPressed = readTtgoLeftButtonPressed();

  tft.init();
  tft.setRotation(GAG_TFT_ROTATION);  // TFT_eSPI rotates the whole UI, including text primitives.
  initializeGloveRuntime(true);

#if GAG_ENABLE_BLE_MOUSE
  Serial.println("Starting BLE mouse backend.");
  g_bleMouse.begin();
#else
  Serial.println("BLE mouse backend not available. Install BleMouse.");
#endif
  initMotors();
  // runStartupVibrationTest();
}

void loop() {
  for (uint8_t i = 0; i < SENSOR_COUNT_ALL; ++i) {
    updateOneIMU(i);
  }
  updateWristMagYaw();
  updateGY511();

#if GAG_ENABLE_RECOGNITION
  feedRecognizerFromCurrentPose();
#endif
  updateVibrations();
#if !GAG_ENABLE_LEFT_BUTTON_QUAT_CAPTURE
  maybeLogSerialSensorQuaternions();
  maybePrintMinorRotationOffsetCandidates();
#endif
  maybeHandleTtgoLeftButtonHardReset();
  maybeHandlePeriodicSoftSensorReset();
  maybeHandleTtgoRightButtonMouseToggle();
  updateContinuousThumbMouseControl();
  processPendingLeftClick();
  processPendingWristMouseToggleSoftReset();
  processPendingGestureSoftResets();
  updateSimultaneousDriftReset();

  gag::viz::FrameInput frame = buildVizFrame();
  g_viz.draw(frame);

  delay(10);
}
