/*
 * ESP32 (LilyGO T-Display) — 3D Hand Skeleton + IMU Debug HUD
 *
 * Hand skeleton:
 *   - Wrist:  MPU9250 (Accel+Gyro) on PCA9548A/TCA9548A channel ACTIVE_CHANNELS[0]
 *   - Fingers: MPU6050-compatible IMUs on remaining ACTIVE_CHANNELS[]
 *
 * Added HUD (left panel):
 *   - 4 mini 3D cubes:
 *       * MPU9250 orientation from gyro integration (r/p from complementary, yaw from gyro)
 *       * MPU9250 orientation with yaw from magnetometer (AK8963) (tilt-compensated heading)
 *       * GY-511 orientation "gyro-like" (smoothed/integrated yaw derived from heading)  [see note]
 *       * GY-511 orientation from magnetometer heading (tilt-compensated heading)
 *   - Motion indicators per sensor:
 *       * Acceleration arrow (linear accel = accel - low-pass gravity)
 *       * Movement arrow (velocity estimate from integrated linear accel, with damping)
 *
 * Wiring note / mux remap (per your description):
 *   - What used to be on SC0/SD0 is now on SC1/SD1
 *   - What used to be on SC1/SD1 is now on SC2/SD2
 *   - What used to be on SC2/SD2 is now on SC6/SD6
 *   - SC0/SD0 now has the GY-511
 *
 * IMPORTANT:
 *   - GY-511 (LSM303DLHC) has accel + magnetometer (no real gyro). The "GY-511 gyro cube"
 *     uses a smoothed / integrated yaw derived from successive magnetometer headings to
 *     provide a "gyro-like" view for comparison. If you later add a real gyro, replace
 *     that estimator with actual gyro rates.
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MPU6050.h>
#include <math.h>

#ifndef TFT_ORANGE
#define TFT_ORANGE 0xFDA0
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
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

// ---------------------
// Channel mapping (PHYSICAL channels on the mux)
//
// Index 0 must remain the wrist IMU for the hand skeleton logic.
// The order of 1..5 must match how you mapped fingers in FINGER_MAP[].
//
// Updated to match your rewire:
//   old ch0 -> ch1
//   old ch1 -> ch2
//   old ch2 -> ch6
//   old ch3 -> ch3
//   old ch4 -> ch4
//   old ch5 -> ch5
// ---------------------
const uint8_t ACTIVE_CHANNELS[] = {1, 2, 6, 3, 4, 5}; // [0]=wrist, [1..5]=fingers
const uint8_t NUM_SENSORS = sizeof(ACTIVE_CHANNELS)/sizeof(ACTIVE_CHANNELS[0]);

// GY-511 is connected to mux channel 0 (SC0/SD0)
static const uint8_t CH_GY511 = 0;

// ============== Finger→index mapping (indexes into ACTIVE_CHANNELS / roll_/pitch_/yaw_ arrays)
// Order: Thumb, Index, Middle, Ring, Little
uint8_t FINGER_MAP[5] = {1,2,3,4,5}; // adjust if your wiring differs

// ============== Options
#define DRAW_PALM 1
#define PALM_STYLE_SPOKES 1

// ============== Display
TFT_eSPI tft;

// ---------------------
// HUD layout
// ---------------------
static const int HUD_W = 100;   // left panel width
static const int HUD_PAD = 2;

// ============== Colors
static const uint16_t COL_BG        = TFT_BLACK;
static const uint16_t COL_TEXT      = 0xB5B6;
static const uint16_t COL_PALM      = 0x8410;
static const uint16_t COL_BONE      = TFT_YELLOW;
static const uint16_t COL_PIVOT     = 0xFD20;
static const uint16_t COL_WRIST     = TFT_RED;

static const uint16_t COL_HUD_FRAME = 0x4208;
static const uint16_t COL_CUBE_A    = TFT_GREEN;     // MPU9250 gyro
static const uint16_t COL_CUBE_B    = TFT_CYAN;      // MPU9250 mag
static const uint16_t COL_CUBE_C    = TFT_MAGENTA;   // GY511 "gyro-like"
static const uint16_t COL_CUBE_D    = TFT_ORANGE;    // GY511 mag
static const uint16_t COL_ACC       = 0x07FF;        // cyan-ish
static const uint16_t COL_MOV       = 0xFBE0;        // yellow-ish


// ============== Motor test (6 vibration motors)
// IMPORTANT:
//   These GPIOs are appropriate as LOGIC/control outputs.
//   Do NOT power motors directly from ESP32 GPIOs in a production design.
//   In this sketch they are used as simple HIGH/LOW control lines.
//
// Suggested pin order matches ACTIVE_CHANNELS[] order:
//   [0]=wrist, [1]=thumb, [2]=index, [3]=middle, [4]=ring, [5]=little
//
// Best candidate pins on the original LilyGO T-Display, assuming GPIO25/26/27
// are NOT physically tied to PCA9548A address pins on your hardware:
//   13, 17, 25, 26, 27, 32
//
// If any pin is unavailable on your wiring, replace it with -1 to disable that motor.
#define ENABLE_MOTOR_TEST 1


// ============== IMU addresses
#define MPU9250_ADDR   0x68  // wrist (IMU)
#define AK8963_ADDR    0x0C  // MPU9250 magnetometer

// ================= Math =================
struct Vec3 { float x,y,z; };
struct Mat3 { float m[3][3]; };

// Motion integration state (for HUD arrows)
struct MotionState {
  Vec3 grav_g{0,0,0};
  Vec3 linAcc_g{0,0,0};
  Vec3 vel_ms{0,0,0};
};


static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r){ return r * 180.0f / (float)M_PI; }

static inline Vec3 add(const Vec3&a,const Vec3&b){ return Vec3{a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline Vec3 sub(const Vec3&a,const Vec3&b){ return Vec3{a.x-b.x,a.y-b.y,a.z-b.z}; }
static inline Vec3 scale(const Vec3&a,float s){ return Vec3{a.x*s,a.y*s,a.z*s}; }
static inline float norm(const Vec3&a){ return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z); }

static Mat3 Rx(float a){ float c=cosf(a), s=sinf(a); return Mat3{{{1,0,0},{0,c,-s},{0,s,c}}}; }
static Mat3 Ry(float a){ float c=cosf(a), s=sinf(a); return Mat3{{{c,0,s},{0,1,0},{-s,0,c}}}; }
static Mat3 Rz(float a){ float c=cosf(a), s=sinf(a); return Mat3{{{c,-s,0},{s,c,0},{0,0,1}}}; }
static Mat3 mul(const Mat3&a,const Mat3&b){
  Mat3 r{};
  for(int i=0;i<3;++i) for(int j=0;j<3;++j){
    r.m[i][j]=0;
    for(int k=0;k<3;++k) r.m[i][j]+=a.m[i][k]*b.m[k][j];
  }
  return r;
}
static Vec3 mul(const Mat3&r,const Vec3&v){
  return Vec3{
    r.m[0][0]*v.x + r.m[0][1]*v.y + r.m[0][2]*v.z,
    r.m[1][0]*v.x + r.m[1][1]*v.y + r.m[1][2]*v.z,
    r.m[2][0]*v.x + r.m[2][1]*v.y + r.m[2][2]*v.z
  };
}
static Mat3 eulerZYX_deg(float rollX,float pitchY,float yawZ){
  return mul(Rz(deg2rad(yawZ)), mul(Ry(deg2rad(pitchY)), Rx(deg2rad(rollX))));
}
static float wrap180(float a){
  while(a > 180.0f) a -= 360.0f;
  while(a < -180.0f) a += 360.0f;
  return a;
}
static float deltaAngleDeg(float a, float b){
  // a-b wrapped to [-180,180]
  float d = a - b;
  return wrap180(d);
}

// ================= Camera & projection (for hand skeleton) =================
struct Camera { Vec3 pos; Mat3 R; float f; };
static Camera cam;
static int proj_cx = 0;
static int proj_cy = 0;

static bool project(const Vec3& P, int &sx, int &sy){
  Vec3 Pm{ P.x - cam.pos.x, P.y - cam.pos.y, P.z - cam.pos.z };
  Vec3 Pc = mul(cam.R, Pm);
  if (Pc.z <= 1.0f) return false;
  float xp = cam.f * (Pc.x / Pc.z);
  float yp = cam.f * (Pc.y / Pc.z);
  sx = (int)(proj_cx + xp);
  sy = (int)(proj_cy - yp);
  return true;
}

static void drawLine3D(const Vec3&A, const Vec3&B, uint16_t col){
  int x1,y1,x2,y2;
  if(project(A,x1,y1) && project(B,x2,y2)) tft.drawLine(x1,y1,x2,y2,col);
}
static void drawPoint3D(const Vec3&P, uint16_t col, int r=2){
  int x,y;
  if(project(P,x,y)) tft.fillCircle(x,y,r,col);
}

// ================= Hand model =================
static const float PALM_WIDTH   = 55;
static const float PALM_LENGTH  = 35;
static const float FINGER_GAP   = PALM_WIDTH/4.0f;
static const float BONE_MCP     = 28;
static const float BONE_PIP     = 22;
static const float BONE_DIP     = 16;
static const float BONE_TH_MB   = 22;
static const float BONE_TH_IP   = 18;

static Vec3 mcpBase[5]; // 0=Thumb,1=Index,2=Middle,3=Ring,4=Little

static void initHandLayout(){
  mcpBase[1] = Vec3{-1.5f*FINGER_GAP, PALM_LENGTH, 0};
  mcpBase[2] = Vec3{-0.5f*FINGER_GAP, PALM_LENGTH, 0};
  mcpBase[3] = Vec3{+0.5f*FINGER_GAP, PALM_LENGTH, 0};
  mcpBase[4] = Vec3{+1.5f*FINGER_GAP, PALM_LENGTH, 0};
  mcpBase[0] = Vec3{-PALM_WIDTH*0.25f, PALM_LENGTH*0.3f, 0};
}

// ================= PCA9548A helpers =================
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
  Wire.write(1<<ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}

// ================= Minimal I2C helpers =================
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
  Wire.requestFrom((int)addr,(int)len);
  for(uint8_t i=0;i<len && Wire.available();++i) buf[i]=Wire.read();
}

// ================= Wrist MPU9250 (Accel+Gyro) =================
#define REG_PWR_MGMT_1     0x6B
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_USER_CTRL      0x6A
#define REG_INT_PIN_CFG    0x37

// AK8963
#define AK8963_WHO_AM_I    0x00
#define AK8963_ST1         0x02
#define AK8963_HXL         0x03
#define AK8963_ST2         0x09
#define AK8963_CNTL1       0x0A
#define AK8963_CNTL2       0x0B
#define AK8963_ASAX        0x10

// ============== IMU state (hand)
MPU6050 mpu[NUM_SENSORS];
float roll_[NUM_SENSORS]  = {0};
float pitch_[NUM_SENSORS] = {0};
float yaw_[NUM_SENSORS]   = {0};
float dt_[NUM_SENSORS]    = {0};
unsigned long lastT[NUM_SENSORS] = {0};
const float alpha = 0.98f; // complementary filter weight

Vec3 accel_g_[NUM_SENSORS] = {};  // scaled accel in g
Vec3 gyro_dps_[NUM_SENSORS] = {}; // scaled gyro in deg/s

// MPU9250 magnetometer state
static bool wristMagOk = false;
static Vec3 wristMagRaw = {0,0,0};
static float wristMagAdj[3] = {1,1,1};
static float yawMagWristDeg = 0.0f;

// Motion integration state (wrist + GY511)
static MotionState motionWrist;
static MotionState motionGy511;

static void updateMotion(MotionState &ms, const Vec3& accel_g, float dt){
  // Low-pass gravity estimate in sensor frame
  const float gravLP = 0.92f;
  ms.grav_g = add(scale(ms.grav_g, gravLP), scale(accel_g, 1.0f - gravLP));

  // Linear acceleration approximation
  ms.linAcc_g = sub(accel_g, ms.grav_g);

  // Deadband to reduce noise integration
  if (norm(ms.linAcc_g) < 0.03f) ms.linAcc_g = Vec3{0,0,0};

  // Integrate to velocity (very approximate; damping reduces drift)
  const float velDamp = 0.96f;
  Vec3 acc_ms2 = scale(ms.linAcc_g, 9.80665f);
  ms.vel_ms = add(scale(ms.vel_ms, velDamp), scale(acc_ms2, dt));
}

// ---------------------
// Tilt-compensated yaw from magnetometer
// rollDeg/pitchDeg are used for compensation
// ---------------------
static float yawFromMagTiltComp(const Vec3& mag, float rollDeg, float pitchDeg){
  float roll  = deg2rad(rollDeg);
  float pitch = deg2rad(pitchDeg);

  float mx = mag.x;
  float my = mag.y;
  float mz = mag.z;

  // Tilt compensation
  float Xh = mx * cosf(pitch) + mz * sinf(pitch);
  float Yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

  float yaw = atan2f(-Yh, Xh); // sign convention; change if needed
  return wrap180(rad2deg(yaw));
}

// ---------------------
// Init/read MPU9250 mag (AK8963) via BYPASS
// ---------------------
static bool initWristMagAK8963(){
  pcaSelect(ACTIVE_CHANNELS[0]);

  // Enable I2C bypass so the AK8963 is visible on the main bus
  i2cWriteByte(MPU9250_ADDR, REG_USER_CTRL, 0x00);
  delay(5);
  i2cWriteByte(MPU9250_ADDR, REG_INT_PIN_CFG, 0x02); // BYPASS_EN
  delay(5);

  uint8_t who = i2cReadByte(AK8963_ADDR, AK8963_WHO_AM_I);
  if (who != 0x48){
    return false;
  }

  // Reset mag
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL2, 0x01);
  delay(10);

  // Power down
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x00);
  delay(10);

  // Enter fuse ROM access mode
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x0F);
  delay(10);

  uint8_t asa[3] = {0};
  i2cReadBytes(AK8963_ADDR, AK8963_ASAX, asa, 3);

  // Sensitivity adjustment values
  wristMagAdj[0] = ((asa[0] - 128) / 256.0f) + 1.0f;
  wristMagAdj[1] = ((asa[1] - 128) / 256.0f) + 1.0f;
  wristMagAdj[2] = ((asa[2] - 128) / 256.0f) + 1.0f;

  // Power down
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x00);
  delay(10);

  // Continuous measurement mode 2 (100 Hz), 16-bit output
  i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x16);
  delay(10);

  return true;
}

static bool readWristMag(Vec3 &magOut){
  pcaSelect(ACTIVE_CHANNELS[0]);

  uint8_t st1 = i2cReadByte(AK8963_ADDR, AK8963_ST1);
  if ((st1 & 0x01) == 0) return false; // not ready

  uint8_t buf[7] = {0};
  i2cReadBytes(AK8963_ADDR, AK8963_HXL, buf, 7);

  // ST2 must be read; buf[6] contains it already
  if (buf[6] & 0x08) return false; // magnetic overflow

  int16_t mx = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t my = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t mz = (int16_t)((buf[5] << 8) | buf[4]);

  // Apply factory sensitivity adjustment. (Scale to arbitrary units; heading only needs ratios.)
  magOut.x = (float)mx * wristMagAdj[0];
  magOut.y = (float)my * wristMagAdj[1];
  magOut.z = (float)mz * wristMagAdj[2];

  return true;
}

// ================= GY-511 (LSM303DLHC) minimal driver =================
#define LSM_ACC_ADDR1  0x19
#define LSM_ACC_ADDR2  0x18
#define LSM_MAG_ADDR   0x1E

// accel regs
#define LSM_CTRL_REG1_A 0x20
#define LSM_CTRL_REG4_A 0x23
#define LSM_OUT_X_L_A   0x28

// mag regs
#define LSM_CRA_REG_M   0x00
#define LSM_CRB_REG_M   0x01
#define LSM_MR_REG_M    0x02
#define LSM_OUT_X_H_M   0x03

static bool gy511Ok = false;
static uint8_t gy511AccAddr = LSM_ACC_ADDR1;

// GY511 orientation state
static float gy511RollDeg = 0.0f;
static float gy511PitchDeg = 0.0f;
static float gy511YawMagDeg = 0.0f;
static float gy511YawGyroLikeDeg = 0.0f;
static float gy511YawMagPrevDeg = 0.0f;
static float gy511YawRateLP = 0.0f;
static unsigned long gy511LastT = 0;

static Vec3 gy511Accel_g = {0,0,0};
static Vec3 gy511MagRaw  = {0,0,0};

static bool initGY511(){
  pcaSelect(CH_GY511);

  // Try accel address 0x19 then 0x18
  uint8_t whoA = i2cReadByte(LSM_ACC_ADDR1, 0x0F);
  if (whoA == 0x33){
    gy511AccAddr = LSM_ACC_ADDR1;
  } else {
    whoA = i2cReadByte(LSM_ACC_ADDR2, 0x0F);
    if (whoA == 0x33){
      gy511AccAddr = LSM_ACC_ADDR2;
    } else {
      return false;
    }
  }

  uint8_t whoM = i2cReadByte(LSM_MAG_ADDR, 0x0F);
  if (whoM != 0x3C){
    // Some clones omit WHO_AM_I; we still attempt init, but flag as not-ok if missing.
    // Returning false here would disable the whole sensor, so we proceed.
  }

  // Accel: 100 Hz, enable XYZ (0x57)
  i2cWriteByte(gy511AccAddr, LSM_CTRL_REG1_A, 0x57);
  delay(5);
  // Accel: high resolution, +/-2g (HR=1, FS=00) => 0x08
  i2cWriteByte(gy511AccAddr, LSM_CTRL_REG4_A, 0x08);
  delay(5);

  // Mag: 30 Hz (0x14)
  i2cWriteByte(LSM_MAG_ADDR, LSM_CRA_REG_M, 0x14);
  delay(5);
  // Mag gain (0x20 ~ 1.3 gauss)
  i2cWriteByte(LSM_MAG_ADDR, LSM_CRB_REG_M, 0x20);
  delay(5);
  // Mag continuous conversion
  i2cWriteByte(LSM_MAG_ADDR, LSM_MR_REG_M, 0x00);
  delay(5);

  gy511LastT = millis();
  gy511YawRateLP = 0;
  return true;
}

static bool readGY511Accel(Vec3 &accel_g){
  pcaSelect(CH_GY511);
  uint8_t buf[6] = {0};

  // auto-increment sub-address bit 7
  i2cReadBytes(gy511AccAddr, (uint8_t)(LSM_OUT_X_L_A | 0x80), buf, 6);

  int16_t ax = (int16_t)((buf[1]<<8) | buf[0]);
  int16_t ay = (int16_t)((buf[3]<<8) | buf[2]);
  int16_t az = (int16_t)((buf[5]<<8) | buf[4]);

  // In high-resolution mode, data is left-justified 12-bit => shift right 4
  ax >>= 4; ay >>= 4; az >>= 4;

  // Sensitivity at +/-2g (high-res): 1 mg/LSB => 1000 LSB per g
  accel_g.x = ax / 1000.0f;
  accel_g.y = ay / 1000.0f;
  accel_g.z = az / 1000.0f;

  return true;
}

static bool readGY511Mag(Vec3 &magRaw){
  pcaSelect(CH_GY511);
  uint8_t buf[6] = {0};
  i2cReadBytes(LSM_MAG_ADDR, LSM_OUT_X_H_M, buf, 6);

  // LSM303DLHC mag order: X, Z, Y (big-endian per axis)
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

  // Orientation from accel (roll/pitch)
  float rollAcc  = atan2f(a.y, a.z);
  float pitchAcc = atan2f(-a.x, sqrtf(a.y*a.y + a.z*a.z));
  gy511RollDeg  = rad2deg(rollAcc);
  gy511PitchDeg = rad2deg(pitchAcc);

  // Tilt-compensated yaw from mag
  gy511YawMagDeg = yawFromMagTiltComp(m, gy511RollDeg, gy511PitchDeg);

  unsigned long now = millis();
  float dt = (now - gy511LastT) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  gy511LastT = now;

  // "Gyro-like" yaw: integrate a smoothed yaw rate derived from successive headings.
  static bool first = true;
  if (first){
    gy511YawMagPrevDeg = gy511YawMagDeg;
    gy511YawGyroLikeDeg = gy511YawMagDeg;
    first = false;
  } else {
    float dyaw = deltaAngleDeg(gy511YawMagDeg, gy511YawMagPrevDeg);
    float yawRate = dyaw / dt; // deg/s
    const float rateLP = 0.7f;
    gy511YawRateLP = rateLP * gy511YawRateLP + (1.0f - rateLP) * yawRate;
    gy511YawGyroLikeDeg = wrap180(gy511YawGyroLikeDeg + gy511YawRateLP * dt);
    gy511YawMagPrevDeg = gy511YawMagDeg;
  }

  // Motion (accel + derived movement)
  updateMotion(motionGy511, gy511Accel_g, dt);
}

// ================= Minimal init/update for hand IMUs =================
static bool initOneIMU(uint8_t idx){
  uint8_t ch = ACTIVE_CHANNELS[idx];
  pcaSelect(ch);

  if (idx==0){
    // Wrist: MPU9250 (we only need accel+gyro registers here)
    i2cWriteByte(MPU9250_ADDR, REG_PWR_MGMT_1, 0x00); delay(10);
    i2cWriteByte(MPU9250_ADDR, REG_GYRO_CONFIG,  0x00);
    i2cWriteByte(MPU9250_ADDR, REG_ACCEL_CONFIG, 0x00);
    return true;
  } else {
    // Fingers: MPU6050-like
    mpu[idx].initialize();
    bool ok = mpu[idx].testConnection();
    if (ok){
      mpu[idx].setFullScaleGyroRange(MPU6050_GYRO_FS_250);
      mpu[idx].setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    }
    return ok;
  }
}

static void updateOneIMU(uint8_t idx){
  int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  pcaSelect(ACTIVE_CHANNELS[idx]);

  if (idx==0){
    uint8_t buf[14] = {0};
    i2cReadBytes(MPU9250_ADDR, REG_ACCEL_XOUT_H, buf, 14);
    ax=(int16_t)((buf[0]<<8)|buf[1]);  ay=(int16_t)((buf[2]<<8)|buf[3]);  az=(int16_t)((buf[4]<<8)|buf[5]);
    gx=(int16_t)((buf[8]<<8)|buf[9]);  gy=(int16_t)((buf[10]<<8)|buf[11]); gz=(int16_t)((buf[12]<<8)|buf[13]);
  } else {
    mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  }

  unsigned long now = millis();
  float dt = (now - lastT[idx]) / 1000.0f;
  if (dt<=0) dt=0.001f;
  lastT[idx]=now;
  dt_[idx] = dt;

  float axg=ax/16384.0f, ayg=ay/16384.0f, azg=az/16384.0f; // ±2g
  float gxds=gx/131.0f,  gyds=gy/131.0f,  gzds=gz/131.0f;  // dps

  accel_g_[idx] = Vec3{axg, ayg, azg};
  gyro_dps_[idx] = Vec3{gxds, gyds, gzds};

  float roll_gyro  = roll_[idx]  + gxds*dt;
  float pitch_gyro = pitch_[idx] + gyds*dt;
  float yaw_gyro   = yaw_[idx]   + gzds*dt; // drift acceptable (gyro integration)

  float roll_acc  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  float pitch_acc = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / (float)M_PI;

  roll_[idx]  = alpha*roll_gyro  + (1-alpha)*roll_acc;
  pitch_[idx] = alpha*pitch_gyro + (1-alpha)*pitch_acc;
  yaw_[idx]   = yaw_gyro;

  yaw_[idx] = wrap180(yaw_[idx]);

  // Motion for wrist only
  if (idx == 0){
    updateMotion(motionWrist, accel_g_[0], dt);
  }
}

// ================= Rendering: Hand =================
static void renderFingerChain(const Vec3& mcp, const Mat3& Rwrist,
                              float rx,float ry,float rz,
                              float len1,float len2,float len3,
                              uint16_t col)
{
  Mat3 Rloc = eulerZYX_deg(rx, ry, rz);
  Mat3 Rf   = mul(Rwrist, Rloc);

  Vec3 p0 = mul(Rwrist, mcp);
  Vec3 step1 = Vec3{0, len1, 0};
  Vec3 step2 = Vec3{0, len2, 0};
  Vec3 step3 = Vec3{0, len3, 0};

  Vec3 p1 = add(p0, mul(Rf, step1));
  Vec3 p2 = add(p1, mul(Rf, step2));
  Vec3 p3 = add(p2, mul(Rf, step3));

  drawLine3D(p0,p1,col); drawLine3D(p1,p2,col); drawLine3D(p2,p3,col);
  drawPoint3D(p0, COL_PIVOT, 2);
}

static void renderPalm(const Mat3& Rwrist){
#if DRAW_PALM
#if PALM_STYLE_SPOKES
  Vec3 origin = mul(Rwrist, Vec3{0,0,0});
  for (int i=0;i<5;++i){
    Vec3 tip = mul(Rwrist, mcpBase[i]);
    drawLine3D(origin, tip, COL_PALM);
  }
#else
  Vec3 w0 = mul(Rwrist, Vec3{-PALM_WIDTH*0.6f,  PALM_LENGTH*0.3f, 0});
  Vec3 w1 = mul(Rwrist, Vec3{ PALM_WIDTH*0.6f,  PALM_LENGTH*0.3f, 0});
  Vec3 w2 = mul(Rwrist, Vec3{ 0, PALM_LENGTH*1.2f,  0});
  Vec3 w3 = mul(Rwrist, Vec3{ 0,-PALM_LENGTH*0.2f,  0});
  drawLine3D(w0,w2,COL_PALM); drawLine3D(w2,w1,COL_PALM); drawLine3D(w1,w3,COL_PALM); drawLine3D(w3,w0,COL_PALM);
#endif
#endif
}

static void updateWristMagYaw(){
  if (!wristMagOk) return;
  Vec3 m;
  if (readWristMag(m)){
    wristMagRaw = m;
    yawMagWristDeg = yawFromMagTiltComp(m, roll_[0], pitch_[0]);
  }
}

static void drawHand3D(){
  // Update all hand sensors
  for(uint8_t i=0;i<NUM_SENSORS;++i) updateOneIMU(i);

  // Update wrist magnetometer yaw (separate from gyro integration)
  updateWristMagYaw();

  Mat3 Rwrist = eulerZYX_deg(roll_[0], pitch_[0], yaw_[0]);
  renderPalm(Rwrist);

  auto relFinger = [&](int finger){
    int idx = FINGER_MAP[finger];
    return Vec3{
      roll_[idx]-roll_[0],
      pitch_[idx]-pitch_[0],
      yaw_[idx]-yaw_[0]
    };
  };

  Vec3 r;
  r = relFinger(0);
  renderFingerChain(mcpBase[0], Rwrist,
                    r.x*0.6f, r.y*0.6f - 25.0f, r.z*0.2f,
                    BONE_TH_MB, BONE_TH_IP*0.9f, BONE_TH_IP*0.7f, COL_BONE);

  r = relFinger(1);
  renderFingerChain(mcpBase[1], Rwrist, r.x, r.y, r.z, BONE_MCP, BONE_PIP, BONE_DIP, COL_BONE);

  r = relFinger(2);
  renderFingerChain(mcpBase[2], Rwrist, r.x, r.y, r.z, BONE_MCP, BONE_PIP, BONE_DIP, COL_BONE);

  r = relFinger(3);
  renderFingerChain(mcpBase[3], Rwrist, r.x, r.y, r.z, BONE_MCP*0.98f, BONE_PIP*0.98f, BONE_DIP*0.98f, COL_BONE);

  r = relFinger(4);
  renderFingerChain(mcpBase[4], Rwrist, r.x, r.y, r.z, BONE_MCP*0.95f, BONE_PIP*0.95f, BONE_DIP*0.90f, COL_BONE);

  drawPoint3D(mul(Rwrist, Vec3{0,0,0}), COL_WRIST, 3);
}

// ================= HUD drawing =================
static void drawArrow2D(int x0, int y0, float vx, float vy, float pxPerUnit, int maxLenPx, uint16_t col){
  // Map +vy up on screen (invert y)
  float sx = vx * pxPerUnit;
  float sy = vy * pxPerUnit;

  float L = sqrtf(sx*sx + sy*sy);
  if (L < 1.0f) return;

  if (L > maxLenPx){
    float k = (float)maxLenPx / L;
    sx *= k; sy *= k;
    L = (float)maxLenPx;
  }

  int x1 = x0 + (int)roundf(sx);
  int y1 = y0 - (int)roundf(sy);

  tft.drawLine(x0, y0, x1, y1, col);

  // Arrowhead
  float ang = atan2f(-(float)(y1 - y0), (float)(x1 - x0)); // screen coords: y inverted already in construction
  float ah = 5.0f;
  float a1 = ang + deg2rad(150.0f);
  float a2 = ang - deg2rad(150.0f);
  int hx1 = x1 + (int)roundf(ah * cosf(a1));
  int hy1 = y1 - (int)roundf(ah * sinf(a1));
  int hx2 = x1 + (int)roundf(ah * cosf(a2));
  int hy2 = y1 - (int)roundf(ah * sinf(a2));
  tft.drawLine(x1, y1, hx1, hy1, col);
  tft.drawLine(x1, y1, hx2, hy2, col);
}

static void drawCubeInRect(int x, int y, int w, int h, const Mat3& R, uint16_t col){
  // Simple local perspective projection into a rectangle
  const float dist = 3.2f;
  float S = (float)min(w, h) * 0.35f;

  int cx = x + w/2;
  int cy = y + h/2;

  Vec3 v[8] = {
    {-1,-1,-1}, {+1,-1,-1}, {+1,+1,-1}, {-1,+1,-1},
    {-1,-1,+1}, {+1,-1,+1}, {+1,+1,+1}, {-1,+1,+1}
  };

  int sx[8], sy[8];
  for (int i=0;i<8;++i){
    Vec3 p = mul(R, v[i]);
    float zz = p.z + dist;
    float xp = (p.x / zz) * S;
    float yp = (p.y / zz) * S;
    sx[i] = cx + (int)roundf(xp);
    sy[i] = cy - (int)roundf(yp);
  }

  auto edge = [&](int a, int b){
    tft.drawLine(sx[a], sy[a], sx[b], sy[b], col);
  };

  // 12 edges
  edge(0,1); edge(1,2); edge(2,3); edge(3,0);
  edge(4,5); edge(5,6); edge(6,7); edge(7,4);
  edge(0,4); edge(1,5); edge(2,6); edge(3,7);
}

static void drawHudCell(int x, int y, int w, int h,
                        const char* label,
                        const Mat3& R, uint16_t cubeCol,
                        const MotionState* motionOrNull)
{
  // Frame
  tft.drawRect(x, y, w, h, COL_HUD_FRAME);

  // Label
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(label, x + 2, y + 1);

  // Cube area
  int cubeY = y + 10;
  int cubeH = h - 10;
  drawCubeInRect(x + 1, cubeY, w - 2, cubeH - 2, R, cubeCol);

  // Motion arrows (optional)
  if (motionOrNull){
    int cx = x + w/2;
    int cy = y + h - 10;

    // Acc arrow: use linear acceleration (g)
    drawArrow2D(cx, cy, motionOrNull->linAcc_g.x, motionOrNull->linAcc_g.y,
                60.0f, 18, COL_ACC);
    // Move arrow: use velocity (m/s)
    drawArrow2D(cx, cy, motionOrNull->vel_ms.x, motionOrNull->vel_ms.y,
                12.0f, 18, COL_MOV);

    // Legend
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString("a", x + 2, y + h - 10);
    tft.drawString("v", x + 12, y + h - 10);
  }
}

static void drawHUD(){
  int H = tft.height();

  // Panel background and separator
  tft.fillRect(0, 0, HUD_W, H, COL_BG);
  tft.drawLine(HUD_W, 0, HUD_W, H, COL_HUD_FRAME);

  int cellW = HUD_W / 2;
  int cellH = H / 2;

  // Build rotation matrices for cubes
  Mat3 RwGyro = eulerZYX_deg(roll_[0], pitch_[0], yaw_[0]);
  Mat3 RwMag  = eulerZYX_deg(roll_[0], pitch_[0], yawMagWristDeg);

  Mat3 RgGyro = eulerZYX_deg(gy511RollDeg, gy511PitchDeg, gy511YawGyroLikeDeg);
  Mat3 RgMag  = eulerZYX_deg(gy511RollDeg, gy511PitchDeg, gy511YawMagDeg);

  drawHudCell(0,         0,         cellW, cellH, "9250 G", RwGyro, COL_CUBE_A, &motionWrist);
  drawHudCell(cellW,     0,         cellW, cellH, "9250 M", RwMag,  COL_CUBE_B, (const MotionState*)nullptr);
  drawHudCell(0,         cellH,     cellW, cellH, "GY511 G",RgGyro, COL_CUBE_C, &motionGy511);
  drawHudCell(cellW,     cellH,     cellW, cellH, "GY511 M",RgMag,  COL_CUBE_D, (const MotionState*)nullptr);
}

const uint8_t NUM_SENSORS_MOTOR = NUM_SENSORS - 1;

// static const int8_t MOTOR_PINS[NUM_SENSORS] = {13, 17, 25, 26, 27, 32};
static const int8_t MOTOR_PINS[NUM_SENSORS_MOTOR] = {17, 2, 15, 13, 12};
static const bool MOTOR_ACTIVE_HIGH = true;

static const unsigned long MOTOR_ONE_ON_MS  = 500;
static const unsigned long MOTOR_ONE_OFF_MS = 500;
static const unsigned long MOTOR_ALL_ON_MS  = 2000;
static const unsigned long MOTOR_ALL_OFF_MS = 2000;

enum MotorTestPhase : uint8_t {
  MOTOR_PHASE_ONE_ON = 0,
  MOTOR_PHASE_ONE_OFF,
  MOTOR_PHASE_ALL_ON,
  MOTOR_PHASE_ALL_OFF
};

static MotorTestPhase motorPhase = MOTOR_PHASE_ONE_ON;
static uint8_t motorPhaseIndex = 0;
static unsigned long motorPhaseStartMs = 0;

static void setMotorOutput(uint8_t motorIdx, bool on){
#if ENABLE_MOTOR_TEST
  if (motorIdx >= NUM_SENSORS_MOTOR) return;
  int8_t pin = MOTOR_PINS[motorIdx];
  if (pin < 0) return;
  digitalWrite((uint8_t)pin, (on == MOTOR_ACTIVE_HIGH) ? HIGH : LOW);
#endif
}

static void setAllMotors(bool on){
#if ENABLE_MOTOR_TEST
  for (uint8_t i = 0; i < NUM_SENSORS_MOTOR; ++i){
    setMotorOutput(i, on);
  }
#endif
}

static void initMotorTest(){
#if ENABLE_MOTOR_TEST
  for (uint8_t i = 0; i < NUM_SENSORS_MOTOR; ++i){
    int8_t pin = MOTOR_PINS[i];
    if (pin < 0) continue;
    pinMode((uint8_t)pin, OUTPUT);
    digitalWrite((uint8_t)pin, (MOTOR_ACTIVE_HIGH ? LOW : HIGH));
  }

  motorPhase = MOTOR_PHASE_ONE_ON;
  motorPhaseIndex = 0;
  motorPhaseStartMs = millis();
  setAllMotors(false);
  setMotorOutput(motorPhaseIndex, true);
#endif
}

static void updateMotorTest(){
#if ENABLE_MOTOR_TEST
  unsigned long now = millis();

  switch (motorPhase){
    case MOTOR_PHASE_ONE_ON:
      if (now - motorPhaseStartMs >= MOTOR_ONE_ON_MS){
        setMotorOutput(motorPhaseIndex, false);
        motorPhase = MOTOR_PHASE_ONE_OFF;
        motorPhaseStartMs = now;
      }
      break;

    case MOTOR_PHASE_ONE_OFF:
      if (now - motorPhaseStartMs >= MOTOR_ONE_OFF_MS){
        ++motorPhaseIndex;
        if (motorPhaseIndex < NUM_SENSORS_MOTOR){
          setMotorOutput(motorPhaseIndex, true);
          motorPhase = MOTOR_PHASE_ONE_ON;
        } else {
          setAllMotors(true);
          motorPhase = MOTOR_PHASE_ALL_ON;
        }
        motorPhaseStartMs = now;
      }
      break;

    case MOTOR_PHASE_ALL_ON:
      if (now - motorPhaseStartMs >= MOTOR_ALL_ON_MS){
        setAllMotors(false);
        motorPhase = MOTOR_PHASE_ALL_OFF;
        motorPhaseStartMs = now;
      }
      break;

    case MOTOR_PHASE_ALL_OFF:
      if (now - motorPhaseStartMs >= MOTOR_ALL_OFF_MS){
        motorPhaseIndex = 0;
        setAllMotors(false);
        setMotorOutput(motorPhaseIndex, true);
        motorPhase = MOTOR_PHASE_ONE_ON;
        motorPhaseStartMs = now;
      }
      break;
  }
#endif
}

// ================= Setup/Loop =================
void setup(){
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  pcaReset();

  // Init hand sensors (wrist + fingers)
  for(uint8_t i=0;i<NUM_SENSORS;++i){
    bool ok = initOneIMU(i);
    lastT[i] = millis();
    if(!ok){
      Serial.printf("IMU init fail idx=%u muxCh=%u\n", i, ACTIVE_CHANNELS[i]);
    }
    delay(10);
  }

  // Init wrist magnetometer
  wristMagOk = initWristMagAK8963();
  if (!wristMagOk){
    Serial.println("Wrist AK8963 mag not detected (check MPU9250 mag/bypass).");
  }

  // Init GY-511
  gy511Ok = initGY511();
  if (!gy511Ok){
    Serial.println("GY-511 not detected on mux channel 0 (check wiring/addresses).");
  }

  initHandLayout();

  // Init motor test outputs
  initMotorTest();

  // Camera for hand skeleton
  cam.pos = Vec3{0, -20, -140};
  cam.R   = mul(Ry(deg2rad(0)), Rx(deg2rad(0)));
  cam.f   = 95.0f;

  proj_cy = tft.height()/2;
}

void loop(){
  // Clear full screen
  tft.fillScreen(COL_BG);

  // Update non-hand sensor first (so HUD has fresh values)
  updateGY511();

  // Shift the hand projection center right to make space for HUD
  proj_cx = HUD_W + (tft.width() - HUD_W)/2;
  proj_cy = tft.height()/2;

  // Small orbit helps depth perception
  static float orbit=0;
  orbit += 0.02f;
  if (orbit > 2*(float)M_PI) orbit -= 2*(float)M_PI;
  cam.pos.x = 8.0f * cosf(orbit);
  cam.pos.z = -140.0f + 8.0f * sinf(orbit);

  // Update motor test sequence
  updateMotorTest();

  // Draw hand skeleton
  drawHand3D();

  // Draw HUD last (overlay)
  drawHUD();

  // Optional serial debug
  Serial.printf("Wrist rpy(g): %.1f %.1f %.1f | yawMag: %.1f | GY511 rpy: %.1f %.1f %.1f\n",
                roll_[0], pitch_[0], yaw_[0], yawMagWristDeg,
                gy511RollDeg, gy511PitchDeg, gy511YawMagDeg);

  delay(10);
}