/**
 * Hand skeleton (pivot-based) visualization from MPU6050 via PCA9548A on ESP32 (LilyGO T-Display)
 *
 * Visualization logic:
 *  - The wrist is a RED point (pivot).
 *  - From the wrist, draw 5 radial guide lines, each separated by 20° and length 50 px.
 *    At the end of each guide, draw an ORANGE pivot (one per finger).
 *  - From each finger pivot, draw a finger line (initially parallel); if the assigned
 *    sensor rotates, the finger rotates around its pivot simulating finger motion.
 *
 * Feature flag:
 *  - WRIST_RELATIVE: if true, finger rotation uses (fingerYaw - wristYaw). If false, absolute yaw.
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
// You originally used the first three channels: 0,1,2. Extend if you add more.
const uint8_t ACTIVE_CHANNELS[] = {0,1,2,3,4,5,6};
const uint8_t NUM_SENSORS = sizeof(ACTIVE_CHANNELS)/sizeof(ACTIVE_CHANNELS[0]);
MPU6050 mpu[NUM_SENSORS];

float roll_[NUM_SENSORS], pitch_[NUM_SENSORS], yaw_[NUM_SENSORS];
unsigned long lastT[NUM_SENSORS];
const float alpha = 0.98f;  // complementary filter gyro weight

// ================= HAND PARTS / MAPPING =================
// Indices for parts
enum HandPart { WRIST=0, THUMB=1, INDEXF=2, MIDDLE=3, RING=4, LITTLE=5 };
const char* PART_NAME[6] = {"Wrist","Thumb","Index","Middle","Ring","Little"};

// Dynamic mapping: wrist uses sensor 0 (ACTIVE_CHANNELS[0]);
// Remaining sensors (1,2,3,4,5...) are assigned one-by-one to Thumb..Little.
// If fewer than 6 sensors are present, the finger sensors wrap among 1..(NUM_SENSORS-1).
// If only 1 sensor exists, all fingers fall back to sensor 0.
uint8_t PART_TO_SENSOR[6];

static inline void buildPartMapping(){
  PART_TO_SENSOR[WRIST] = 0; // first sensor -> wrist
  if(NUM_SENSORS <= 1){
    // no extra sensors; everything uses wrist sensor
    PART_TO_SENSOR[THUMB] = 0;
    PART_TO_SENSOR[INDEXF]= 0;
    PART_TO_SENSOR[MIDDLE]= 0;
    PART_TO_SENSOR[RING]  = 0;
    PART_TO_SENSOR[LITTLE]= 0;
    return;
  }
  // assign 1.. to fingers, wrapping across available finger sensors
  // available finger sensors count:
  uint8_t avail = NUM_SENSORS - 1; // sensors 1..NUM_SENSORS-1
  uint8_t order[5] = {THUMB, INDEXF, MIDDLE, RING, LITTLE};
  for(uint8_t i=0;i<5;i++){
    PART_TO_SENSOR[order[i]] = 1 + (i % avail);
  }
}

// ================= DISPLAY =================
TFT_eSPI tft = TFT_eSPI();
uint16_t bgColor = TFT_BLACK;

// Colors
#define COL_WRIST   TFT_RED
#define COL_PIVOT   0xFD20  // orange
#define COL_GUIDE   0xC618  // light grey
#define COL_FINGER  TFT_YELLOW
#define COL_TEXT    TFT_WHITE

// ================= MATH HELPERS (primitive-only signatures) =================
static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r){ return r * 180.0f / (float)M_PI; }

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

// 2D helper to draw a line by angle/length around a pivot
static inline void drawRayPolar(int x0, int y0, float angleDeg, float length, uint16_t color){
  float a = deg2rad(angleDeg);
  int x1 = x0 + (int)(cosf(a) * length);
  int y1 = y0 - (int)(sinf(a) * length);
  tft.drawLine(x0, y0, x1, y1, color);
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
  yaw_[idx]   = yaw_gyro; // yaw drifts without magnetometer

  if(yaw_[idx] > 180.0f)  yaw_[idx]-=360.0f;
  if(yaw_[idx] < -180.0f) yaw_[idx]+=360.0f;
}

// ================= DRAW: HAND (PIVOT MODEL) =================
void drawHandPivotModel(){
  const int W = tft.width();
  const int H = tft.height();
  tft.fillScreen(bgColor);

  // Wrist pivot in red
  const int wristX = W/2 - 10;
  const int wristY = H/2 + 10;
  tft.fillCircle(wristX, wristY, 4, COL_WRIST);

  // Wrist orientation (for relative mode)
  uint8_t wristS = PART_TO_SENSOR[WRIST];
  float yawW = yaw_[wristS];

  // Five guide rays: base angle and +20° steps
  const float baseAngleDeg = -40.0f; // relative to +X axis (0° to the right), positive CCW (screen y down)
  const float stepDeg = 20.0f;
  const float guideLen = 50.0f;
  const float fingerLen = 40.0f; // length of the rotating finger line

  // All finger lines start parallel at this default direction (before sensor rotation)
  const float fingerBaseDirDeg = -90.0f; // up on screen

  for(int i=0;i<5;i++){
    float angle = baseAngleDeg + i*stepDeg;

    // Draw guide from wrist to finger pivot
    drawRayPolar(wristX, wristY, angle, guideLen, COL_GUIDE);

    // Compute finger pivot coordinate (end of guide)
    float a = deg2rad(angle);
    int px = wristX + (int)(cosf(a) * guideLen);
    int py = wristY - (int)(sinf(a) * guideLen);

    // Draw pivot in orange
    tft.fillCircle(px, py, 3, COL_PIVOT);

    // Determine which sensor drives this finger
    HandPart part = (HandPart)(i+1); // THUMB..LITTLE
    uint8_t s = PART_TO_SENSOR[part];

    // Compute the finger line angle
    float yawF = yaw_[s];
    float relYaw = WRIST_RELATIVE ? (yawF - yawW) : yawF;
    // Keep within [-180,180] for sanity
    if(relYaw > 180.0f)  relYaw -= 360.0f;
    if(relYaw < -180.0f) relYaw += 360.0f;

    float fingerAngle = fingerBaseDirDeg + relYaw; // rotate around the pivot

    // Draw finger line from pivot
    drawRayPolar(px, py, fingerAngle, fingerLen, COL_FINGER);
  }

  // Labels
  tft.setTextColor(COL_TEXT, bgColor);
  tft.setTextFont(1);
  tft.drawString(WRIST_RELATIVE ? "Wrist-relative" : "Absolute", W-5, 10, 1);
  tft.drawString("Wrist", wristX+8, wristY-6, 1);
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

  // Build mapping: sensor0 -> wrist; sensor1.. -> fingers in order
  buildPartMapping();

  tft.init();
  tft.setRotation(1);              // 240x135 landscape
  tft.fillScreen(bgColor);
  tft.setTextColor(COL_TEXT, bgColor);
  tft.setTextFont(2);
  tft.drawString("MPU6050 Hand Pivots", 120, 10);

  // Initialize sensors listed in ACTIVE_CHANNELS
  for(uint8_t i=0;i<NUM_SENSORS;i++){
    if(!initOneMPU(i)){
      // Error banner per failed sensor
      tft.fillRect(0, 30 + i*16, tft.width(), 14, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.drawString("MPU FAIL on CH" + String(ACTIVE_CHANNELS[i]), 5, 37 + i*16);
      tft.setTextColor(COL_TEXT, bgColor);
    }
  }
}

void loop() {
  // Update all physical sensors once per frame
  for (uint8_t s = 0; s < NUM_SENSORS; ++s) {
    updateOneMPU(s);
  }

  // Draw hand pivot model
  drawHandPivotModel();

  delay(8);
}
