/*
 * ESP32 (LilyGO T-Display) — 3D Hand Skeleton from MPU9250 (wrist) + MPU6050 (fingers) via PCA9548A
 *
 * Enhanced version:
 *  - Fixes palm visualization (no more white square)
 *  - Adds finger mapping configuration (FINGER_MAP[])
 *  - Adjustable camera framing
 *  - Optional palm guide toggle and spoke-style palm
 *  - Fixes GCC/Arduino init-list ambiguity errors
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MPU6050.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =====================
// Pins & PCA9548A
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
const uint8_t ACTIVE_CHANNELS[] = {0,1,2,3,4,5}; // 0=wrist, 1..5=fingers
const uint8_t NUM_SENSORS = sizeof(ACTIVE_CHANNELS)/sizeof(ACTIVE_CHANNELS[0]);

// ============== Finger→channel mapping (indexes into ACTIVE_CHANNELS)
// Order: Thumb, Index, Middle, Ring, Little
uint8_t FINGER_MAP[5] = {1,1,2,3,4}; // change Thumb to 5 if it has its own sensor

// ============== Options
#define DRAW_PALM 1
#define PALM_STYLE_SPOKES 1

// ============== IMU addresses
#define MPU9250_ADDR   0x68  // wrist

// ============== Display colors
TFT_eSPI tft;
static const uint16_t COL_BG     = TFT_BLACK;
static const uint16_t COL_TEXT   = 0xB5B6;
static const uint16_t COL_PALM   = 0x8410;   // darker gray
static const uint16_t COL_BONE   = TFT_YELLOW;
static const uint16_t COL_PIVOT  = 0xFD20;   // orange
static const uint16_t COL_WRIST  = TFT_RED;

// ============== IMU state
MPU6050 mpu[NUM_SENSORS];
float roll_[NUM_SENSORS]  = {0};
float pitch_[NUM_SENSORS] = {0};
float yaw_[NUM_SENSORS]   = {0};
unsigned long lastT[NUM_SENSORS] = {0};
const float alpha = 0.98f; // complementary filter weight

// ================= Math =================
struct Vec3 { float x,y,z; };
struct Mat3 { float m[3][3]; };

static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }
static Mat3 Rx(float a){ float c=cosf(a), s=sinf(a); return Mat3{{{1,0,0},{0,c,-s},{0,s,c}}}; }
static Mat3 Ry(float a){ float c=cosf(a), s=sinf(a); return Mat3{{{c,0,s},{0,1,0},{-s,0,c}}}; }
static Mat3 Rz(float a){ float c=cosf(a), s=sinf(a); return Mat3{{{c,-s,0},{s,c,0},{0,0,1}}}; }
static Mat3 mul(const Mat3&a,const Mat3&b){ Mat3 r{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j){ r.m[i][j]=0; for(int k=0;k<3;++k) r.m[i][j]+=a.m[i][k]*b.m[k][j]; } return r; }
static Vec3 mul(const Mat3&r,const Vec3&v){ return Vec3{ r.m[0][0]*v.x + r.m[0][1]*v.y + r.m[0][2]*v.z, r.m[1][0]*v.x + r.m[1][1]*v.y + r.m[1][2]*v.z, r.m[2][0]*v.x + r.m[2][1]*v.y + r.m[2][2]*v.z }; }
static Vec3 add(const Vec3&a,const Vec3&b){ return Vec3{a.x+b.x,a.y+b.y,a.z+b.z}; }
static Mat3 eulerZYX_deg(float rollX,float pitchY,float yawZ){ return mul(Rz(deg2rad(yawZ)), mul(Ry(deg2rad(pitchY)), Rx(deg2rad(rollX)))); }

// ================= Camera & projection =================
struct Camera { Vec3 pos; Mat3 R; float f; };
static Camera cam;

static bool project(const Vec3& P, int &sx, int &sy){
  Vec3 Pm{ P.x - cam.pos.x, P.y - cam.pos.y, P.z - cam.pos.z };
  Vec3 Pc = mul(cam.R, Pm);
  if (Pc.z <= 1.0f) return false;
  float xp = cam.f * (Pc.x / Pc.z);
  float yp = cam.f * (Pc.y / Pc.z);
  int cx = tft.width()/2, cy = tft.height()/2;
  sx = (int)(cx + xp);
  sy = (int)(cy - yp);
  return true;
}

static void drawLine3D(const Vec3&A, const Vec3&B, uint16_t col){ int x1,y1,x2,y2; if(project(A,x1,y1) && project(B,x2,y2)) tft.drawLine(x1,y1,x2,y2,col); }
static void drawPoint3D(const Vec3&P, uint16_t col, int r=2){ int x,y; if(project(P,x,y)) tft.fillCircle(x,y,r,col); }

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
static void pcaSelect(uint8_t ch){ Wire.beginTransmission(pca_addr); Wire.write(1<<ch); Wire.endTransmission(); delayMicroseconds(200); }

// ================= Minimal I2C for wrist MPU9250 =================
#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

static void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val){ Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission(); }
static void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t*buf, uint8_t len){ Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false); Wire.requestFrom((int)addr,(int)len); for(uint8_t i=0;i<len && Wire.available();++i) buf[i]=Wire.read(); }

static bool initOneIMU(uint8_t idx){
  uint8_t ch = ACTIVE_CHANNELS[idx];
  pcaSelect(ch);
  if (idx==0){
    i2cWriteByte(MPU9250_ADDR, REG_PWR_MGMT_1, 0x00); delay(10);
    i2cWriteByte(MPU9250_ADDR, REG_GYRO_CONFIG,  0x00);
    i2cWriteByte(MPU9250_ADDR, REG_ACCEL_CONFIG, 0x00);
    return true;
  } else {
    mpu[idx].initialize();
    bool ok = mpu[idx].testConnection();
    if (ok){ mpu[idx].setFullScaleGyroRange(MPU6050_GYRO_FS_250); mpu[idx].setFullScaleAccelRange(MPU6050_ACCEL_FS_2); }
    return ok;
  }
}

static void updateOneIMU(uint8_t idx){
  int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  pcaSelect(ACTIVE_CHANNELS[idx]);
  if (idx==0){
    uint8_t buf[14]; i2cReadBytes(MPU9250_ADDR, REG_ACCEL_XOUT_H, buf, 14);
    ax=(int16_t)((buf[0]<<8)|buf[1]); ay=(int16_t)((buf[2]<<8)|buf[3]); az=(int16_t)((buf[4]<<8)|buf[5]);
    gx=(int16_t)((buf[8]<<8)|buf[9]); gy=(int16_t)((buf[10]<<8)|buf[11]); gz=(int16_t)((buf[12]<<8)|buf[13]);
  } else { mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz); }

  unsigned long now = millis();
  float dt = (now - lastT[idx]) / 1000.0f; if(dt<=0) dt=0.001f; lastT[idx]=now;

  float axg=ax/16384.0f, ayg=ay/16384.0f, azg=az/16384.0f; // ±2g
  float gxds=gx/131.0f,  gyds=gy/131.0f,  gzds=gz/131.0f;  // dps

  float roll_gyro  = roll_[idx]  + gxds*dt;
  float pitch_gyro = pitch_[idx] + gyds*dt;
  float yaw_gyro   = yaw_[idx]   + gzds*dt; // drift acceptable

  float roll_acc  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  float pitch_acc = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / (float)M_PI;

  roll_[idx]  = alpha*roll_gyro  + (1-alpha)*roll_acc;
  pitch_[idx] = alpha*pitch_gyro + (1-alpha)*pitch_acc;
  yaw_[idx]   = yaw_gyro;
  if (yaw_[idx] > 180) yaw_[idx]-=360; else if (yaw_[idx] < -180) yaw_[idx]+=360;
}

// ================= Rendering =================
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
  for (int i=0;i<5;++i){ Vec3 tip = mul(Rwrist, mcpBase[i]); drawLine3D(origin, tip, COL_PALM); }
#else
  Vec3 w0 = mul(Rwrist, Vec3{-PALM_WIDTH*0.6f,  PALM_LENGTH*0.3f, 0});
  Vec3 w1 = mul(Rwrist, Vec3{ PALM_WIDTH*0.6f,  PALM_LENGTH*0.3f, 0});
  Vec3 w2 = mul(Rwrist, Vec3{ 0, PALM_LENGTH*1.2f,  0});
  Vec3 w3 = mul(Rwrist, Vec3{ 0,-PALM_LENGTH*0.2f,  0});
  drawLine3D(w0,w2,COL_PALM); drawLine3D(w2,w1,COL_PALM); drawLine3D(w1,w3,COL_PALM); drawLine3D(w3,w0,COL_PALM);
#endif
#endif
}

static void drawHand3D(){
  for(uint8_t i=0;i<NUM_SENSORS;++i) updateOneIMU(i);

  Mat3 Rwrist = eulerZYX_deg(roll_[0], pitch_[0], yaw_[0]);
  renderPalm(Rwrist);

  auto relFinger = [&](int finger){ int idx = FINGER_MAP[finger]; return Vec3{ roll_[idx]-roll_[0], pitch_[idx]-pitch_[0], yaw_[idx]-yaw_[0] }; };

  Vec3 r;
  r = relFinger(0);
  renderFingerChain(mcpBase[0], Rwrist, r.x*0.6f, r.y*0.6f - 25.0f, r.z*0.2f,
                    BONE_TH_MB, BONE_TH_IP*0.9f, BONE_TH_IP*0.7f, COL_BONE);

  r = relFinger(1);
  renderFingerChain(mcpBase[1], Rwrist, r.x, r.y, r.z,
                    BONE_MCP, BONE_PIP, BONE_DIP, COL_BONE);

  r = relFinger(2);
  renderFingerChain(mcpBase[2], Rwrist, r.x, r.y, r.z,
                    BONE_MCP, BONE_PIP, BONE_DIP, COL_BONE);

  r = relFinger(3);
  renderFingerChain(mcpBase[3], Rwrist, r.x, r.y, r.z,
                    BONE_MCP*0.98f, BONE_PIP*0.98f, BONE_DIP*0.98f, COL_BONE);

  r = relFinger(4);
  renderFingerChain(mcpBase[4], Rwrist, r.x, r.y, r.z,
                    BONE_MCP*0.95f, BONE_PIP*0.95f, BONE_DIP*0.90f, COL_BONE);

  drawPoint3D(mul(Rwrist, Vec3{0,0,0}), COL_WRIST, 3);
}

// ================= Setup/Loop =================
void setup(){
  Serial.begin(115200);
  tft.init(); tft.setRotation(1); tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG); tft.setTextDatum(TL_DATUM);
  tft.drawString("3D Hand (MPU9250+MPU6050)", 4, 2);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  pcaReset();

  for(uint8_t i=0;i<NUM_SENSORS;++i){
    bool ok = initOneIMU(i);
    if(!ok){ tft.fillRect(0, 18+i*14, tft.width(), 13, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.drawString(String("IMU FAIL ch ")+String(ACTIVE_CHANNELS[i]), 4, 20+i*14);
      tft.setTextColor(COL_TEXT, COL_BG);
    }
    delay(5);
  }

  initHandLayout();
  cam.pos = Vec3{0, -20, -140};
  cam.R   = mul(Ry(deg2rad(0)), Rx(deg2rad(0)));
  cam.f   = 95.0f;
}

void loop(){
  // tft.fillRect(0, 16, tft.width(), tft.height()-16, COL_BG);
  tft.fillRect(0, 0, tft.width(), tft.height()-16, COL_BG);

  // Small orbit helps depth perception
  static float orbit=0; orbit += 0.02f; if (orbit > 2*(float)M_PI) orbit -= 2*(float)M_PI;
  cam.pos.x = 8.0f * cosf(orbit);
  cam.pos.z = -140.0f + 8.0f * sinf(orbit);
  // cam.pos.z = -140.0f + 8.0f * sinf(orbit);

  drawHand3D();

  // Debug
  Serial.printf("Wrist rpy: %.1f %.1f %.1f\n", roll_[0], pitch_[0], yaw_[0]);
  delay(10);
}
