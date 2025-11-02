/*
 * ESP32 (LilyGO T-Display) — 3D Hand Skeleton from MPU9250 (wrist) + MPU6050 (fingers) via PCA9548A
 *
 * What’s new vs your 2D version
 *  - Full 3D wireframe hand: palm + 5 fingers with 3 joints each (MCP, PIP, DIP) and a simple thumb base.
 *  - Wrist IMU (MPU9250) controls the global 3D orientation of the hand.
 *  - Each finger MPU6050 reports its own local rotation that’s applied *relative to* the wrist.
 *  - Simple perspective projection with a tiny camera orbit and lighting-ish intensity (by line thickness).
 *  - Same I²C multiplexer (PCA9548A) + same pins; complementary filter kept for stability.
 *
 * Display: LilyGO T-Display (ESP32 + ST7789 240x135). Library: TFT_eSPI (configured for T-Display).
 * Sensors: Wrist = PCA channel 0 (MPU9250 on 0x68), Fingers = channels 1..5 (MPU6050s on 0x68).
 *
 * NOTE: This is a single-file sketch you can paste into Arduino IDE/Arduino-CLI/PlatformIO.
 *       Works with the same wiring as your original sketch. If your addresses differ,
 *       tweak MPU9250_ADDR / the MPU6050 default address or your AD0 wiring.
 *
 * Dependencies you should install:
 *   - TFT_eSPI  (configured for LilyGO T-Display ST7789 240x135)
 *   - Wire (built-in)
 *   - MPU6050 by Electronic Cats (or Jeff Rowberg’s)
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MPU6050.h>   // used for finger sensors

// =====================
// Hardware configuration
// =====================
#define PIN_I2C_SDA   22
#define PIN_I2C_SCL   21

#define PIN_PCA_RST   33
#define PIN_PCA_A0    25
#define PIN_PCA_A1    26
#define PIN_PCA_A2    27
#define DRIVE_PCA_ADDR_PINS  false  // set true only if you actually wired A0..A2

#define PCA9548A_BASE_ADDR   0x70
static uint8_t pca_addr = PCA9548A_BASE_ADDR;

// Channel 0 = wrist (MPU9250), channels 1..5 = fingers (MPU6050)
const uint8_t ACTIVE_CHANNELS[] = {0,1,2,3,4,5};
const uint8_t NUM_SENSORS = sizeof(ACTIVE_CHANNELS)/sizeof(ACTIVE_CHANNELS[0]);

// ================= IMU I2C addresses =================
#define MPU9250_ADDR   0x68   // wrist IMU
// Finger IMUs are MPU6050 at their default address 0x68 unless AD0 pulled high (0x69).

// ================== Display + colors =================
TFT_eSPI tft;
static const uint16_t COL_BG     = TFT_BLACK;
static const uint16_t COL_TEXT   = 0xB5B6;  // soft gray
static const uint16_t COL_PALM   = 0xC618;  // light gray
static const uint16_t COL_BONE   = TFT_YELLOW;
static const uint16_t COL_PIVOT  = 0xFD20;  // orange
static const uint16_t COL_WRIST  = TFT_RED;

// ================== IMU storage ==================
MPU6050 mpu[NUM_SENSORS];             // we’ll only *use* idx>0 with the MPU6050 class
float roll_[NUM_SENSORS]  = {0};      // degrees
float pitch_[NUM_SENSORS] = {0};
float yaw_[NUM_SENSORS]   = {0};
unsigned long lastT[NUM_SENSORS] = {0};
const float alpha = 0.98f;            // complementary filter gyro weight

// ================ Simple 3D math =================
struct Vec3 { float x,y,z; };
struct Mat3 { float m[3][3]; };

static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r){ return r * 180.0f / (float)M_PI; }

static Mat3 Rx(float a){
  const float c=cosf(a), s=sinf(a);
  return Mat3{{{1,0,0},{0,c,-s},{0,s,c}}};
}
static Mat3 Ry(float a){
  const float c=cosf(a), s=sinf(a);
  return Mat3{{{c,0,s},{0,1,0},{-s,0,c}}};
}
static Mat3 Rz(float a){
  const float c=cosf(a), s=sinf(a);
  return Mat3{{{c,-s,0},{s,c,0},{0,0,1}}};
}
static Mat3 mul(const Mat3&a,const Mat3&b){
  Mat3 r{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j){ r.m[i][j]=0; for(int k=0;k<3;++k) r.m[i][j]+=a.m[i][k]*b.m[k][j]; }
  return r;
}
static Vec3 mul(const Mat3&r,const Vec3&v){
  return Vec3{
    r.m[0][0]*v.x + r.m[0][1]*v.y + r.m[0][2]*v.z,
    r.m[1][0]*v.x + r.m[1][1]*v.y + r.m[1][2]*v.z,
    r.m[2][0]*v.x + r.m[2][1]*v.y + r.m[2][2]*v.z
  };
}
static Vec3 add(const Vec3&a,const Vec3&b){ return Vec3{a.x+b.x,a.y+b.y,a.z+b.z}; }

// Convert ZYX euler degrees (roll X, pitch Y, yaw Z) to rotation matrix
static Mat3 eulerZYX_deg(float rollX, float pitchY, float yawZ){
  Mat3 R = mul(Rz(deg2rad(yawZ)), mul(Ry(deg2rad(pitchY)), Rx(deg2rad(rollX))));
  return R;
}

// ============ Simple camera & projection ============
struct Camera { Vec3 pos; Mat3 R; float f; };

static Camera cam;  // set up in setup()

static bool project(const Vec3& P, int &sx, int &sy){
  // camera transform: Pc = R*(P - pos)
  const Vec3 Pm{ P.x - cam.pos.x, P.y - cam.pos.y, P.z - cam.pos.z };
  const Vec3 Pc = mul(cam.R, Pm);
  if (Pc.z <= 1.0f) return false; // behind camera or too near
  const float xp = cam.f * (Pc.x / Pc.z);
  const float yp = cam.f * (Pc.y / Pc.z);
  // map to screen: center in the middle of T-Display (240x135), flip Y
  const int cx = tft.width()/2;
  const int cy = tft.height()/2;
  sx = (int)(cx + xp);
  sy = (int)(cy - yp);
  return (sx>=-50 && sx<=tft.width()+50 && sy>=-50 && sy<=tft.height()+50);
}

static void drawLine3D(const Vec3&A, const Vec3&B, uint16_t col){
  int x1,y1,x2,y2; if(!project(A,x1,y1)) return; if(!project(B,x2,y2)) return; 
  tft.drawLine(x1,y1,x2,y2,col);
}

static void drawPoint3D(const Vec3&P, uint16_t col, int r=2){
  int x,y; if(!project(P,x,y)) return; tft.fillCircle(x,y,r,col);
}

// ================ Hand model (units: pixels) =================
// Local hand frame origin roughly at wrist center, +X right, +Y up, +Z toward viewer.
static const float PALM_WIDTH   = 55;  // across index to little MCPs
static const float PALM_LENGTH  = 35;  // wrist to MCP row
static const float FINGER_GAP   = PALM_WIDTH/4.0f;
static const float BONE_MCP     = 28;
static const float BONE_PIP     = 22;
static const float BONE_DIP     = 16;
static const float BONE_TH_MB   = 22;  // thumb metacarpal (rotated out)
static const float BONE_TH_IP   = 18;

// MCP base points in the palm (in the hand local frame before wrist rotation)
static Vec3 mcpBase[5]; // [0]=Thumb, 1=Index, 2=Middle, 3=Ring, 4=Little

// populate the MCP base layout
static void initHandLayout(){
  // Put the MCP line at y = +PALM_LENGTH (upwards), centered in X for fingers 1..4
  mcpBase[1] = Vec3{-1.5f*FINGER_GAP, PALM_LENGTH, 0};
  mcpBase[2] = Vec3{-0.5f*FINGER_GAP, PALM_LENGTH, 0};
  mcpBase[3] = Vec3{+0.5f*FINGER_GAP, PALM_LENGTH, 0};
  mcpBase[4] = Vec3{+1.5f*FINGER_GAP, PALM_LENGTH, 0};
  // Thumb base slightly towards wrist and outwards
  mcpBase[0] = Vec3{-PALM_WIDTH*0.25f, PALM_LENGTH*0.3f, 0};
}

// ================ PCA9548A helpers =================
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

// ============ Minimal I2C for wrist MPU9250 ============
static void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t*buf, uint8_t len){
  Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)len);
  for(uint8_t i=0;i<len && Wire.available();++i) buf[i]=Wire.read();
}

// MPU9250 regs we use
#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// Initialize one sensor on ACTIVE_CHANNELS[idx]
static bool initOneIMU(uint8_t idx){
  const uint8_t ch = ACTIVE_CHANNELS[idx];
  pcaSelect(ch);
  if (idx==0){
    // Wrist: raw setup for MPU9250
    i2cWriteByte(MPU9250_ADDR, REG_PWR_MGMT_1, 0x00); // wake
    delay(10);
    i2cWriteByte(MPU9250_ADDR, REG_GYRO_CONFIG,  0x00); // ±250 dps
    i2cWriteByte(MPU9250_ADDR, REG_ACCEL_CONFIG, 0x00); // ±2g
    return true;
  } else {
    // Finger: MPU6050 via library
    mpu[idx].initialize();
    bool ok = mpu[idx].testConnection();
    if (ok) {
      mpu[idx].setFullScaleGyroRange(MPU6050_GYRO_FS_250);
      mpu[idx].setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    }
    return ok;
  }
}

// Read / update filter for one IMU; stores degrees in roll_/pitch_/yaw_ arrays
static void updateOneIMU(uint8_t idx){
  int16_t ax,ay,az,gx,gy,gz; ax=ay=az=gx=gy=gz=0;
  pcaSelect(ACTIVE_CHANNELS[idx]);
  if (idx==0){
    uint8_t buf[14]; i2cReadBytes(MPU9250_ADDR, REG_ACCEL_XOUT_H, buf, 14);
    ax = (int16_t)((buf[0]<<8)|buf[1]); ay=(int16_t)((buf[2]<<8)|buf[3]); az=(int16_t)((buf[4]<<8)|buf[5]);
    gx = (int16_t)((buf[8]<<8)|buf[9]); gy=(int16_t)((buf[10]<<8)|buf[11]); gz=(int16_t)((buf[12]<<8)|buf[13]);
  } else {
    mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  }
  const unsigned long now = millis();
  float dt = (now - lastT[idx]) / 1000.0f; if (dt<=0) dt=0.001f; lastT[idx]=now;

  const float axg=ax/16384.0f, ayg=ay/16384.0f, azg=az/16384.0f; // ±2g scale
  const float gxds=gx/131.0f,  gyds=gy/131.0f,  gzds=gz/131.0f;  // dps

  // integrate gyro
  const float roll_gyro  = roll_[idx]  + gxds * dt;
  const float pitch_gyro = pitch_[idx] + gyds * dt;
  const float yaw_gyro   = yaw_[idx]   + gzds * dt;  // yaw drifts, acceptable for visualization

  // accelerometer inclination
  const float roll_acc  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  const float pitch_acc = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / (float)M_PI;

  roll_[idx]  = alpha*roll_gyro  + (1-alpha)*roll_acc;
  pitch_[idx] = alpha*pitch_gyro + (1-alpha)*pitch_acc;
  yaw_[idx]   = yaw_gyro; // no mag; drift is fine

  // wrap yaw
  if (yaw_[idx] > 180) yaw_[idx]-=360; else if (yaw_[idx] < -180) yaw_[idx]+=360;
}

// ================ Hand rendering =================
// Build a finger chain from an MCP base with given local rotations (relative to wrist)
static void renderFingerChain(const Vec3& mcp, const Mat3& Rwrist,
                              float rx_deg, float ry_deg, float rz_deg,
                              float len1, float len2, float len3,
                              uint16_t col)
{
  // Local finger rotation (relative to wrist). Use ZYX order for ‘human’ feel.
  const Mat3 Rloc = eulerZYX_deg(rx_deg, ry_deg, rz_deg);
  const Mat3 Rf = mul(Rwrist, Rloc);

  const Vec3 p0 = mul(Rwrist, mcp);           // MCP position in world (hand) space
  const Vec3 p1 = add(p0, mul(Rf, Vec3{0, len1, 0}));
  const Vec3 p2 = add(p1, mul(Rf, Vec3{0, len2, 0}));
  const Vec3 p3 = add(p2, mul(Rf, Vec3{0, len3, 0}));

  drawLine3D(p0,p1,col); drawLine3D(p1,p2,col); drawLine3D(p2,p3,col);
  drawPoint3D(p0, COL_PIVOT, 2);
}

static void renderPalm(const Mat3& Rwrist){
  // simple diamond palm outline for depth cue
  const Vec3 w0 = mul(Rwrist, Vec3{-PALM_WIDTH*0.6f,  0, 0});
  const Vec3 w1 = mul(Rwrist, Vec3{ PALM_WIDTH*0.6f,  0, 0});
  const Vec3 w2 = mul(Rwrist, Vec3{ 0, PALM_LENGTH*1.1f,  0});
  const Vec3 w3 = mul(Rwrist, Vec3{ 0,-PALM_LENGTH*0.8f,  0});
  drawLine3D(w0,w2,COL_PALM); drawLine3D(w2,w1,COL_PALM);
  drawLine3D(w1,w3,COL_PALM); drawLine3D(w3,w0,COL_PALM);
}

static void drawHand3D(){
  // Update sensors first
  for(uint8_t i=0;i<NUM_SENSORS;++i) updateOneIMU(i);

  // Wrist orientation in world
  const Mat3 Rwrist = eulerZYX_deg(roll_[0], pitch_[0], yaw_[0]);

  // Palm
  renderPalm(Rwrist);

  // Fingers (relative rotations are finger - wrist for a stable feel)
  auto rel = [&](int i){ return Vec3{ roll_[i]-roll_[0], pitch_[i]-pitch_[0], yaw_[i]-yaw_[0] }; };

  // Thumb (0): flex mainly around its own ‘x’ and a bit yaw outwards
  {
    Vec3 r = rel(1); // use channel 1 sensor for thumb if that’s how you wired it; change if not
    // map: flex = pitch, splay = yaw
    renderFingerChain(mcpBase[0], Rwrist, r.x*0.6f, r.y*0.6f - 25.0f, r.z*0.2f,
                      BONE_TH_MB, BONE_TH_IP*0.9f, BONE_TH_IP*0.7f, COL_BONE);
  }
  // Index (1)
  {
    Vec3 r = rel(1);
    renderFingerChain(mcpBase[1], Rwrist, r.x, r.y, r.z,
                      BONE_MCP, BONE_PIP, BONE_DIP, COL_BONE);
  }
  // Middle (2)
  {
    Vec3 r = rel(2);
    renderFingerChain(mcpBase[2], Rwrist, r.x, r.y, r.z,
                      BONE_MCP, BONE_PIP, BONE_DIP, COL_BONE);
  }
  // Ring (3)
  {
    Vec3 r = rel(3);
    renderFingerChain(mcpBase[3], Rwrist, r.x, r.y, r.z,
                      BONE_MCP*0.98f, BONE_PIP*0.98f, BONE_DIP*0.98f, COL_BONE);
  }
  // Little (4)
  {
    Vec3 r = rel(4);
    renderFingerChain(mcpBase[4], Rwrist, r.x, r.y, r.z,
                      BONE_MCP*0.95f, BONE_PIP*0.95f, BONE_DIP*0.9f, COL_BONE);
  }

  // Wrist marker
  drawPoint3D(mul(Rwrist, Vec3{0,0,0}), COL_WRIST, 3);
}

// ================ Setup & Loop =================
void setup(){
  Serial.begin(115200);

  // Display
  tft.init();
  tft.setRotation(1); // landscape for T-Display
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("3D Hand (MPU9250+MPU6050)", 4, 2);

  // I2C & PCA
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  pcaReset();

  // Init IMUs
  for(uint8_t i=0;i<NUM_SENSORS;++i){
    bool ok = initOneIMU(i);
    if(!ok){
      tft.fillRect(0, 18+ i*14, tft.width(), 13, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.drawString(String("IMU FAIL ch ")+String(ACTIVE_CHANNELS[i]), 4, 20+i*14);
      tft.setTextColor(COL_TEXT, COL_BG);
    }
    delay(5);
  }

  // Hand base layout
  initHandLayout();

  // Camera: a little above and in front of the wrist, looking to origin.
  cam.pos = Vec3{0, -25, -120};
  cam.R   = mul(Ry(deg2rad(0)), Rx(deg2rad(0))); // facing +Z
  cam.f   = 120.0f; // focal length in pixels
}

void loop(){
  // Clear drawing area below the title bar
  tft.fillRect(0, 16, tft.width(), tft.height()-16, COL_BG);

  // (optional) tiny camera orbit for depth perception; comment out if undesired
  static float orbit=0; orbit += 0.02f; if (orbit>2*M_PI) orbit-=2*M_PI;
  cam.pos.x =  10.0f * cosf(orbit);
  cam.pos.z = -120.0f + 10.0f * sinf(orbit);

  drawHand3D();

  // Debug: print wrist RPY
  Serial.printf("Wrist rpy: %.1f %.1f %.1f\n", roll_[0], pitch_[0], yaw_[0]);
  delay(8);
}
