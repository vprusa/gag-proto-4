/**
 * Six-cube visualization from 3x MPU6050 via PCA9548A on ESP32 (LilyGO T-Display)
 *
 * - Uses Jeff Rowberg / Electronic Cats MPU6050 library (getMotion6)
 * - Draws SIX wireframe cubes across the T-Display (mirrors 3 sensors twice)
 * - I2C mux channels used: 0,1,2
 *
 * Libraries (Arduino Library Manager):
 *   - MPU6050 by Electronic Cats
 *   - TFT_eSPI (configure for LilyGO T-Display ST7789 135x240)
 */

#include <Wire.h>
#include <MPU6050.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

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
#define NUM_SENSORS 4
const uint8_t SENSOR_CH[NUM_SENSORS] = {0, 1, 2, 3};
MPU6050 mpu[NUM_SENSORS];

float roll_[NUM_SENSORS], pitch_[NUM_SENSORS], yaw_[NUM_SENSORS];
unsigned long lastT[NUM_SENSORS];
const float alpha = 0.98f;  // complementary filter gyro weight

// ================= DISPLAY / VIEWS =================
TFT_eSPI tft = TFT_eSPI();

struct Viewport { int x, y, w, h; };

#define NUM_VIEWS 6  // draw 6 cubes across the screen
Viewport views[NUM_VIEWS];
uint16_t bgColor = TFT_BLACK;
uint16_t fgColor = TFT_WHITE;

// map each view to a sensor index (0,1,2,0,1,2)
inline uint8_t viewToSensor(uint8_t i){ return i % NUM_SENSORS; }

// ================= 3D CUBE GEOMETRY =================
const float cubeVerts[8][3] = {
  {-1,-1,-1},{ 1,-1,-1},{ 1, 1,-1},{-1, 1,-1},
  {-1,-1, 1},{ 1,-1, 1},{ 1, 1, 1},{-1, 1, 1}
};
const uint8_t edges[12][2] = {
  {0,1},{1,2},{2,3},{3,0},
  {4,5},{5,6},{6,7},{7,4},
  {0,4},{1,5},{2,6},{3,7}
};

// ================= MATH / DRAW HELPERS =================
static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }

static inline void rotateRPY(
  float vx, float vy, float vz, float rDeg, float pDeg, float yDeg,
  float &ox, float &oy, float &oz
){
  // Rotation order: yaw(Z) -> pitch(Y) -> roll(X)
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

static inline void projectTo2D(
  float vx, float vy, float vz, int cx, int cy, float scale, int16_t &sx, int16_t &sy
){
  float depth = (vz + 3.0f);
  float persp = 1.0f / (0.6f + 0.2f*depth);
  sx = (int16_t)(cx + vx * scale * persp);
  sy = (int16_t)(cy - vy * scale * persp);
}

static inline void drawCube(int vx, int vy, int vw, int vh, float r, float p, float y) {
  tft.fillRect(vx, vy, vw, vh, bgColor);

  int cx = vx + vw/2;
  int cy = vy + vh/2;
  float scale = (float)min(vw, vh) * 0.32f;  // slightly smaller for narrow columns

  int16_t sx[8], sy[8];
  for (int i=0;i<8;i++){
    float rx, ry, rz;
    rotateRPY(cubeVerts[i][0], cubeVerts[i][1], cubeVerts[i][2], r, p, y, rx, ry, rz);
    projectTo2D(rx, ry, rz, cx, cy, scale, sx[i], sy[i]);
  }

  for (int e=0;e<12;e++){
    uint8_t a=edges[e][0], b=edges[e][1];
    tft.drawLine(sx[a], sy[a], sx[b], sy[b], fgColor);
  }
}

// ================= SENSOR INIT/UPDATE =================
bool initOneMPU(uint8_t idx) {
  pcaSelectChannel(SENSOR_CH[idx]);
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
  pcaSelectChannel(SENSOR_CH[idx]);
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

  // Lay out 6 columns with padding
  int pad = 3;                      // tighter padding so 6 fit nicely
  int W = tft.width();              // 240
  int H = tft.height();             // 135
  int colW = (W - pad * (NUM_VIEWS + 1)) / NUM_VIEWS;
  for (int i = 0; i < NUM_VIEWS; ++i) {
    int x = pad + i * (colW + pad);
    views[i] = { x, pad, colW, H - 2 * pad };
  }

  tft.drawString("MPU6050 x3 via PCA9548A", W/2, 10);

  // Initialize sensors on channels 0..2
  for(uint8_t i=0;i<NUM_SENSORS;i++){
    if(!initOneMPU(i)){
      // paint all views mapped to this sensor in red as an error
      for(uint8_t v=0; v<NUM_VIEWS; ++v){
        if(viewToSensor(v)==i){
          tft.fillRect(views[v].x, views[v].y, views[v].w, views[v].h, TFT_RED);
          tft.setTextColor(TFT_WHITE, TFT_RED);
          tft.drawString("MPU FAIL", views[v].x+views[v].w/2, views[v].y+views[v].h/2);
          tft.setTextColor(fgColor, bgColor);
        }
      }
    }
  }
}

void loop() {
  // Update each of the 3 sensors once per frame
  for (uint8_t s = 0; s < NUM_SENSORS; ++s) {
    updateOneMPU(s);
  }

  // Use smaller font for narrow headers
  tft.setTextFont(1);

  // Draw 6 mirrored views (0,1,2,0,1,2)
  for (uint8_t i = 0; i < NUM_VIEWS; ++i) {
    uint8_t s = viewToSensor(i);

    // Header bar per view
    tft.fillRect(views[i].x, views[i].y, views[i].w, 12, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    char buf[40];
    snprintf(buf, sizeof(buf), "V%u CH%u R%+.0f P%+.0f Y%+.0f", i, (unsigned)s, roll_[s], pitch_[s], yaw_[s]);
    tft.drawString(buf, views[i].x + views[i].w/2, views[i].y + 6);
    tft.setTextColor(fgColor, bgColor);

    // Cube area
    int vx = views[i].x;
    int vy = views[i].y + 14;
    int vw = views[i].w;
    int vh = views[i].h - 16;
    drawCube(vx, vy, vw, vh, roll_[s], pitch_[s], yaw_[s]);
  }

  delay(5);
}
