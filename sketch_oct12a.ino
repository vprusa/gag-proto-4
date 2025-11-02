/**
 * Hand skeleton visualization from MPU9250 (wrist) + MPU6050 (fingers) via PCA9548A
 * ESP32 LilyGO T-Display — with wrist data logging.
 */

#include <Wire.h>
#include <MPU6050.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

#define WRIST_RELATIVE true
#define PIN_SCL 21
#define PIN_SDA 22
#define PIN_PCA_RST 33
#define PIN_PCA_A0 25
#define PIN_PCA_A1 26
#define PIN_PCA_A2 27
#define DRIVE_PCA_ADDR_PINS false
#define PCA9548A_BASE_ADDR 0x70
uint8_t pca_addr = PCA9548A_BASE_ADDR;

void pcaReset(){ pinMode(PIN_PCA_RST,OUTPUT); digitalWrite(PIN_PCA_RST,LOW); delay(2); digitalWrite(PIN_PCA_RST,HIGH); delay(2);} 
void pcaSelectChannel(uint8_t ch){ Wire.beginTransmission(pca_addr); Wire.write(1<<ch); Wire.endTransmission(); delayMicroseconds(200);} 

// ========= SENSORS =========
const uint8_t ACTIVE_CHANNELS[] = {0,1,2,3,4,5,6}; // wrist + 6 fingers
const uint8_t NUM_SENSORS = sizeof(ACTIVE_CHANNELS)/sizeof(ACTIVE_CHANNELS[0]);
#define MPU9250_ADDR 0x68
MPU6050 mpu[NUM_SENSORS];
float roll_[NUM_SENSORS], pitch_[NUM_SENSORS], yaw_[NUM_SENSORS];
unsigned long lastT[NUM_SENSORS];
const float alpha = 0.98f;

// ========= DISPLAY =========
TFT_eSPI tft = TFT_eSPI();
uint16_t bgColor=TFT_BLACK;
#define COL_WRIST   TFT_RED
#define COL_PIVOT   0xFD20
#define COL_GUIDE   0xC618
#define COL_FINGER  TFT_YELLOW
#define COL_TEXT    TFT_WHITE

// ========= HAND PARTS =========
enum HandPart {WRIST=0,THUMB=1,INDEXF=2,MIDDLE=3,RING=4,LITTLE=5};
uint8_t PART_TO_SENSOR[6];
static inline void buildPartMapping(){PART_TO_SENSOR[WRIST]=0;uint8_t avail=NUM_SENSORS-1;uint8_t order[5]={THUMB,INDEXF,MIDDLE,RING,LITTLE};for(uint8_t i=0;i<5;i++){PART_TO_SENSOR[order[i]]=1+(i%avail);} }

// ========= MATH =========
static inline float deg2rad(float d){return d*M_PI/180.0f;}
static inline void drawRayPolar(int x0,int y0,float aDeg,float len,uint16_t col){float a=deg2rad(aDeg);int x1=x0+cosf(a)*len;int y1=y0-sinf(a)*len;tft.drawLine(x0,y0,x1,y1,col);} 

// ========= I2C WRIST (MPU9250) =========
static inline void i2cWriteByte(uint8_t addr,uint8_t reg,uint8_t val){Wire.beginTransmission(addr);Wire.write(reg);Wire.write(val);Wire.endTransmission();}
static inline void i2cReadBytes(uint8_t addr,uint8_t reg,uint8_t n,uint8_t* dst){Wire.beginTransmission(addr);Wire.write(reg);Wire.endTransmission(false);Wire.requestFrom((int)addr,(int)n);for(uint8_t i=0;i<n&&Wire.available();++i)dst[i]=Wire.read();}

bool initMPU9250_Wrist(){pcaSelectChannel(ACTIVE_CHANNELS[0]);i2cWriteByte(MPU9250_ADDR,0x6B,0x80);delay(100);i2cWriteByte(MPU9250_ADDR,0x6B,0x01);i2cWriteByte(MPU9250_ADDR,0x6C,0x00);i2cWriteByte(MPU9250_ADDR,0x1B,0x00);i2cWriteByte(MPU9250_ADDR,0x1C,0x00);i2cWriteByte(MPU9250_ADDR,0x1D,0x03);i2cWriteByte(MPU9250_ADDR,0x19,0x07);uint8_t buf[14];i2cReadBytes(MPU9250_ADDR,0x3B,14,buf);int16_t ax=(buf[0]<<8)|buf[1];int16_t ay=(buf[2]<<8)|buf[3];int16_t az=(buf[4]<<8)|buf[5];float axg=ax/16384.0f,ayg=ay/16384.0f,azg=az/16384.0f;roll_[0]=atan2f(ayg,azg)*180.0f/M_PI;pitch_[0]=atan2f(-axg,sqrtf(ayg*ayg+azg*azg))*180.0f/M_PI;yaw_[0]=0;lastT[0]=millis();Serial.println("MPU9250 (wrist) initialized.");return true;}

void readMPU9250_Wrist(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){pcaSelectChannel(ACTIVE_CHANNELS[0]);uint8_t buf[14];i2cReadBytes(MPU9250_ADDR,0x3B,14,buf);ax=(buf[0]<<8)|buf[1];ay=(buf[2]<<8)|buf[3];az=(buf[4]<<8)|buf[5];gx=(buf[8]<<8)|buf[9];gy=(buf[10]<<8)|buf[11];gz=(buf[12]<<8)|buf[13];}

bool initOneMPU(uint8_t idx){if(idx==0)return initMPU9250_Wrist();pcaSelectChannel(ACTIVE_CHANNELS[idx]);mpu[idx].initialize();if(!mpu[idx].testConnection())return false;lastT[idx]=millis();int16_t ax,ay,az,gx,gy,gz;mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);float axg=ax/16384.0f,ayg=ay/16384.0f,azg=az/16384.0f;roll_[idx]=atan2f(ayg,azg)*180.0f/M_PI;pitch_[idx]=atan2f(-axg,sqrtf(ayg*ayg+azg*azg))*180.0f/M_PI;yaw_[idx]=0;return true;}

void updateOneMPU(uint8_t idx){int16_t ax,ay,az,gx,gy,gz;if(idx==0)readMPU9250_Wrist(ax,ay,az,gx,gy,gz);else{pcaSelectChannel(ACTIVE_CHANNELS[idx]);mpu[idx].getMotion6(&ax,&ay,&az,&gx,&gy,&gz);}unsigned long now=millis();float dt=(now-lastT[idx])/1000.0f;if(dt<=0)dt=0.001f;lastT[idx]=now;float axg=ax/16384.0f,ayg=ay/16384.0f,azg=az/16384.0f;float gxds=gx/131.0f,gyds=gy/131.0f,gzds=gz/131.0f;float roll_gyro=roll_[idx]+gxds*dt;float pitch_gyro=pitch_[idx]+gyds*dt;float yaw_gyro=yaw_[idx]+gzds*dt;float roll_acc=atan2f(ayg,azg)*180.0f/M_PI;float pitch_acc=atan2f(-axg,sqrtf(ayg*ayg+azg*azg))*180.0f/M_PI;roll_[idx]=alpha*roll_gyro+(1-alpha)*roll_acc;pitch_[idx]=alpha*pitch_gyro+(1-alpha)*pitch_acc;yaw_[idx]=yaw_gyro;if(yaw_[idx]>180)yaw_[idx]-=360;if(yaw_[idx]<-180)yaw_[idx]+=360;if(idx==0){Serial.printf("Wrist data -> Roll:%6.1f  Pitch:%6.1f  Yaw:%6.1f\n",roll_[0],pitch_[0],yaw_[0]);}}

// ========= DRAW HAND =========
void drawHandPivotModel(){int W=tft.width();int H=tft.height();tft.fillScreen(bgColor);int wristX=W/2-10;int wristY=H/2+0;tft.fillCircle(wristX,wristY,4,COL_WRIST);uint8_t wristS=PART_TO_SENSOR[WRIST];float yawW=yaw_[wristS];float baseAngle=-40,step=20,guideLen=50,fingerLen=40,fingerBaseDir=-90;for(int i=0;i<5;i++){float ang=baseAngle+i*step;drawRayPolar(wristX,wristY,ang,guideLen,COL_GUIDE);float a=deg2rad(ang);int px=wristX+cosf(a)*guideLen;int py=wristY-sinf(a)*guideLen;tft.fillCircle(px,py,3,COL_PIVOT);uint8_t s=PART_TO_SENSOR[i+1];float yawF=yaw_[s];float relYaw=WRIST_RELATIVE?(yawF-yawW):yawF;if(relYaw>180)relYaw-=360;if(relYaw<-180)relYaw+=360;float fingerAngle=fingerBaseDir+relYaw;drawRayPolar(px,py,fingerAngle,fingerLen,COL_FINGER);}tft.setTextColor(COL_TEXT,bgColor);tft.setTextFont(1);tft.drawString(WRIST_RELATIVE?"Wrist-rel":"Abs",W-5,10,1);} 

void setup(){Serial.begin(115200);Wire.begin(PIN_SDA,PIN_SCL,400000);pcaReset();buildPartMapping();tft.init();tft.setRotation(1);tft.fillScreen(bgColor);tft.setTextColor(COL_TEXT,bgColor);tft.setTextFont(2);tft.drawString("MPU Hand Pivot",120,10);for(uint8_t i=0;i<NUM_SENSORS;i++){if(!initOneMPU(i)){tft.fillRect(0,30+i*16,tft.width(),14,TFT_RED);tft.setTextColor(TFT_WHITE,TFT_RED);tft.drawString("MPU FAIL CH"+String(ACTIVE_CHANNELS[i]),5,37+i*16);tft.setTextColor(COL_TEXT,bgColor);}}}

void loop(){for(uint8_t s=0;s<NUM_SENSORS;s++)updateOneMPU(s);drawHandPivotModel();delay(8);}
