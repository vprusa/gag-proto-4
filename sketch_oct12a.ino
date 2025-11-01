/**
 * Hand skeleton visualization from MPU6050 via PCA9548A on ESP32 (LilyGO T-Display)
 *
 * - Replaces cube views with a simple hand skeleton: 5 fingers + wrist (palm/top of hand)
 * - Supports 3 physical sensors on mux channels {0,1,2} by default and mirrors them to 6 parts
 *   (Wrist, Thumb, Index, Middle, Ring, Little). If you add more sensors, extend ACTIVE_CHANNELS.
 * - Feature flag WRIST_RELATIVE: when true, finger rotations are drawn relative to the wrist.
 *
 * Libraries (Arduino Library Manager):
 *   - MPU6050 by Electronic Cats (Jeff Rowberg fork)
 *   - TFT_eSPI (configure for LilyGO T-Display ST7789 135x240)
 */

#include <Wire.h>
#include <MPU6050.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

// ================== FEATURE FLAGS ==================
#define WRIST_RELATIVE true   // set to false to draw absolute finger rotations

// ================= PIN CONFIG =================
#define PIN_SCL 21
#define PIN_SDA 22
#define PIN_PCA_RST 33
#define PIN_PCA_A0 25
#define PIN_PCA_A1 26
#define PIN_PCA_A2 27
#define DRIVE_PCA_ADDR_PINS false  // set true ONLY if A0/A1/A2 are actually wired to these pins

#define PCA9548A_BASE_ADDR 0x70
uint8_t pca_addr = PCA9548A_BASE_ADDR;

// ================= PCA9548A HELPERS =================
void pcaReset() {
  pinMode(PIN_PCA_RST, OUTPUT);
  digitalWrite(PIN_PCA_RST, LOW);
  delay(2);
  digitalWrite(PIN_PCA_RST, HIGH);
  delay(2);
}

void pcaSelectChannel(uint8_t ch) {
  Wire.beginTransmission(pca_addr);
  Wire.write(1 << ch);  // one-hot select
  Wire.endTransmission();
  delayMicroseconds(200);
}

// ================= SENSORS =================
// List the active mux channels that actually have an MPU6050 connected.
// By default you said first three channels are used: 0,1,2.
const uint8_t ACTIVE_CHANNELS[] = {0,1,2};
const uint8_t NUM_SENSORS = sizeof(ACTIVE_CHANNELS)/sizeof(ACTIVE_CHANNELS[0]);
MPU6050 mpu[NUM_SENSORS];

float roll_[NUM_SENSORS], pitch_[NUM_SENSORS], yaw_[NUM_SENSORS];
unsigned long lastT[NUM_SENSORS];
const float alpha = 0.98f;  // complementary filter gyro weight

// ================= HAND PARTS / MAPPING =================
// 6 parts: wrist + 5 fingers
enum HandPart { WRIST=0, THUMB=1, INDEXF=2, MIDDLE=3, RING=4, LITTLE=5 };
const char* PART_NAME[6] = {"Wrist","Thumb","Index","Middle","Ring","Little"};

// Map each hand part to which physical sensor index to use
// Default: mirror 3 sensors across 6 parts (0,1,2,0,1,2).
uint8_t PART_TO_SENSOR[6] = {
  0 % NUM_SENSORS, // Wrist
  1 % NUM_SENSORS, // Thumb
  2 % NUM_SENSORS, // Index
  0 % NUM_SENSORS, // Middle
  1 % NUM_SENSORS, // Ring
  2 % NUM_SENSORS  // Little
};

// ================= DISPLAY =================
TFT_eSPI tft = TFT_eSPI();
uint16_t bgColor = TFT_BLACK;
uint16_t fgColor = TFT_WHITE;

// ================= 3D HELPERS (primitive-only signatures to avoid Arduino autoproto issues) =================
static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }

// Rotate vector v by roll/pitch/yaw (degrees), Z (yaw) -> Y (pitch) -> X (roll)
static inline void rotateRPY(
  float vx, float vy, float vz, float rDeg, float pDeg, float yDeg,
  float &ox, float &oy, float &oz
){
  float r = deg2rad(rDeg), p = deg2rad(pDeg), y = deg2rad(yDeg);
  float cr=cosf(r), sr=sinf(r), cp=cosf(p), sp=sinf(p), cy=cosf(y), sy=sinf(y);
  // yaw
  float x1 = cy*vx - sy*vy;
  float y1 = sy*vx + cy*vy;
  float z1 = vz;
  // pitch
  float x2 = cp*x1 + sp*z1;
  float y2 = y1;
  float z2 = -sp*x1 + cp*z1;
  // roll
  ox = x2;
  oy = cr*y2 - sr*z2;
  oz = sr*y2 + cr*z2;
}

// Simple pseudo-perspective orthographic projection
static inline void projectTo2D(
  float vx, float vy, float vz, int cx, int cy, float scale, int16_t &sx, int16_t &sy
){
  float depth = (vz + 3.0f);
  float persp = 1.0f / (0.6f + 0.2f*depth);
  sx = (int16_t)(cx + vx * scale * persp);
  sy = (int16_t)(cy - vy * scale * persp);
}

// ================= DRAW: HAND SKELETON =================
// Draw a single bone starting from (x0,y0) in screen coords, given orientation r/p/y and length in pixels
static inline void drawBone(int x0, int y0, float rDeg, float pDeg, float yDeg, float lengthPx, uint16_t color){
  // Base vector pointing "up" in model space
  float bx=0, by=1, bz=0;
  float rx, ry, rz;
  rotateRPY(bx, by, bz, rDeg, pDeg, yDeg, rx, ry, rz);
  // Project tip
  int16_t sx0=x0, sy0=y0, sx1, sy1;
  projectTo2D(rx*lengthPx, ry*lengthPx, rz*lengthPx, x0, y0, 1.0f, sx1, sy1);
  tft.drawLine(sx0, sy0, sx1, sy1, color);
  // small tip marker
  tft.fillCircle(sx1, sy1, 2, color);
}

// Draw the whole hand: one wrist bone + 2-segment fingers
static inline void drawHandSkeleton(int W, int H){
  tft.fillScreen(bgColor);

  // Center palm a bit left of center and lower in the screen
  const int cx = W/2 - 10;
  const int cy = H/2 + 25;

  // Visual lengths (pixels)
  const float wristLen = 28;
  const float baseLen  = 26; // MCP->PIP
  const float tipLen   = 20; // PIP->tip

  // Spread anchors for finger bases along a slight arc
  // Offsets relative to palm center (screen coords)
  const int anchorDX[5] = {-22, -8, 8, 22, 34};   // thumb .. little
  const int anchorDY[5] = {-6,  -10, -12, -10, -8};

  // Fetch wrist orientation (absolute)
  uint8_t wristS = PART_TO_SENSOR[WRIST];
  float rW = roll_[wristS];
  float pW = pitch_[wristS];
  float yW = yaw_[wristS];

  // Draw wrist from palm center upwards a bit
  drawBone(cx, cy, rW, pW, yW, wristLen, TFT_CYAN);

  // Finger parts: THUMB..LITTLE = 5 fingers
  for(int f = THUMB; f <= LITTLE; ++f){
    uint8_t s = PART_TO_SENSOR[f];
    float r = roll_[s];
    float p = pitch_[s];
    float y = yaw_[s];

    if(WRIST_RELATIVE){
      // Approximate relative orientation by subtracting Euler angles
      r -= rW; p -= pW; y -= yW;
    }

    // Anchor for this finger
    int ax = cx + anchorDX[f-1];
    int ay = cy + anchorDY[f-1];

    // Draw proximal phalanx
    // Compute first segment tip to use as next start
    float bx=0, by=1, bz=0; float rx, ry, rz;
    rotateRPY(bx, by, bz, r, p, y, rx, ry, rz);
    int16_t sxa, sya; projectTo2D(rx*baseLen, ry*baseLen, rz*baseLen, ax, ay, 1.0f, sxa, sya);
    tft.drawLine(ax, ay, sxa, sya, TFT_WHITE);

    // Distal phalanx – slightly curl by reducing pitch to hint a fingertip bend
    float r2 = r; float p2 = p * 0.7f; float y2 = y;
    float rx2, ry2, rz2; rotateRPY(bx, by, bz, r2, p2, y2, rx2, ry2, rz2);
    int16_t sxb, syb; projectTo2D(rx2*tipLen, ry2*tipLen, rz2*tipLen, sxa, sya, 1.0f, sxb, syb);
    tft.drawLine(sxa, sya, sxb, syb, TFT_WHITE);

    // fingertip marker & label
    tft.fillCircle(sxb, syb, 2, TFT_YELLOW);
    tft.setTextColor(TFT_WHITE, bgColor);
    tft.setTextFont(1);
    tft.drawString(PART_NAME[f], ax, ay-10);
  }

  // Legend
  tft.setTextFont(1);
  tft.setTextColor(TFT_GREEN, bgColor);
  tft.drawString(WRIST_RELATIVE ? "Hand: wrist-relative" : "Hand: absolute", W - 5, 10, 1);
}

// ================= SENSOR INIT/UPDATE =================
bool initOneMPU(uint8_t idx) {
  // idx indexes into ACTIVE_CHANNELS & arrays
  pcaSelectChannel(ACTIVE_CHANNELS[idx]);
  mpu[idx].initialize();
  if(!mpu[idx].testConnection()) return false;

  lastT[idx] = millis();
  int16_t ax,ay,az,gx,gy,gz;
  mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float axg=ax/16384.0f, ayg=ay/16384.0f, azg=az/16384.0f;

  roll_[idx]  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  pitch_[idx] = atan2f(-axg, sqrtf(ayg*ayg+azg*azg)) * 180.0f / (float)M_PI;
  yaw_[idx]=0.0f;
  return true;
}

void updateOneMPU(uint8_t idx) {
  pcaSelectChannel(ACTIVE_CHANNELS[idx]);
  int16_t ax,ay,az,gx,gy,gz;
  mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  unsigned long now = millis();
  float dt = (now - lastT[idx]) / 1000.0f;
  if(dt<=0) dt=0.001f;
  lastT[idx]=now;

  float axg=ax/16384.0f, ayg=ay/16384.0f, azg=az/16384.0f;
  float gxds=gx/131.0f,  gyds=gy/131.0f,  gzds=gz/131.0f;

  float roll_gyro =  roll_[idx] + gxds*dt;
  float pitch_gyro = pitch_[idx] + gyds*dt;
  float yaw_gyro =   yaw_[idx] + gzds*dt;

  float roll_acc  = atan2f(ayg, azg) * 180.0f / (float)M_PI;
  float pitch_acc = atan2f(-axg, sqrtf(ayg*ayg+azg*azg)) * 180.0f / (float)M_PI;

  roll_[idx]  = alpha*roll_gyro  + (1.0f-alpha)*roll_acc;
  pitch_[idx] = alpha*pitch_gyro + (1.0f-alpha)*pitch_acc;
  yaw_[idx]   = yaw_gyro; // yaw will drift without magnetometer

  if(yaw_[idx] > 180.0f)  yaw_[idx]-=360.0f;
  if(yaw_[idx] < -180.0f) yaw_[idx]+=360.0f;
}

// ================= ARDUINO SETUP/LOOP =================
void setup() {
  Serial.begin(115200);

  if(DRIVE_PCA_ADDR_PINS){
    pinMode(PIN_PCA_A0,OUTPUT);
    pinMode(PIN_PCA_A1,OUTPUT);
    pinMode(PIN_PCA_A2,OUTPUT);
    digitalWrite(PIN_PCA_A0,LOW);
    digitalWrite(PIN_PCA_A1,LOW);
    digitalWrite(PIN_PCA_A2,LOW);
    // If you change these HIGH/LOW, update pca_addr accordingly (0x70..0x77)
  }

  Wire.begin(PIN_SDA, PIN_SCL, 400000); // 400kHz I2C
  pcaReset();

  tft.init();
  tft.setRotation(1);              // 240x135 landscape
  tft.fillScreen(bgColor);
  tft.setTextColor(fgColor, bgColor);
  tft.setTextFont(2);

  // tft.drawString("MPU6050 Hand Skeleton", 120, 10);

  // Initialize sensors listed in ACTIVE_CHANNELS
  for(uint8_t i=0;i<NUM_SENSORS;i++){
    if(!initOneMPU(i)){
      // If a sensor fails, draw an error banner
      tft.fillRect(0, 30 + i*16, tft.width(), 14, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      // tft.drawString("MPU FAIL on CH" + String(ACTIVE_CHANNELS[i]), 5, 37 + i*16);
      tft.setTextColor(fgColor, bgColor);
    }
  }
}

void loop() {
  // Update all physical sensors once per frame
  for (uint8_t s = 0; s < NUM_SENSORS; ++s) {
    updateOneMPU(s);
  }

  // Draw the hand skeleton
  drawHandSkeleton(tft.width(), tft.height());

  delay(8);
}
