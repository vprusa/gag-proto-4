// Include the MultiSDA_I2C class you pasted
#include "MultiSDA_I2C.h"

// Shared clock pin
#define SCL_PIN 22

// Two SDA lines for the two MPU6050 sensors
#define SDA1_PIN 21
#define SDA2_PIN 17

// MPU6050 I2C address (AD0 low)
#define MPU_ADDR 0x68

// MPU6050 register addresses
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

MultiSDA_I2C myI2C(SCL_PIN);

// Helper to write a single byte to MPU6050
bool mpuWrite(uint8_t sdaPin, uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {reg, data};
  return myI2C.writeBytes(sdaPin, MPU_ADDR, buf, 2, true);
}

// Helper to read N bytes from MPU6050
bool mpuRead(uint8_t sdaPin, uint8_t reg, uint8_t *buf, size_t len) {
  // Send register address first
  if (!myI2C.writeBytes(sdaPin, MPU_ADDR, &reg, 1, true)) return false;
  delayMicroseconds(10);
  return myI2C.readBytes(sdaPin, MPU_ADDR, buf, len, true);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nMPU6050 Dual I2C Reader Starting...");

  const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};
  myI2C.begin(sdaPins, 2);

  // Wake up both MPU6050s
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t sda = sdaPins[i];
    if (mpuWrite(sda, PWR_MGMT_1, 0x00)) {
      Serial.printf("Sensor %d initialized on SDA %d\n", i + 1, sda);
    } else {
      Serial.printf("Sensor %d (SDA %d) failed to init!\n", i + 1, sda);
    }
  }
}

void loop() {
  uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};

  for (uint8_t i = 0; i < 2; i++) {
    uint8_t sda = sdaPins[i];
    uint8_t buf[14];
    if (mpuRead(sda, ACCEL_XOUT_H, buf, 14)) {
      int16_t ax = (buf[0] << 8) | buf[1];
      int16_t ay = (buf[2] << 8) | buf[3];
      int16_t az = (buf[4] << 8) | buf[5];
      int16_t gx = (buf[8] << 8) | buf[9];
      int16_t gy = (buf[10] << 8) | buf[11];
      int16_t gz = (buf[12] << 8) | buf[13];

      Serial.printf("MPU%d (SDA=%d): AX=%6d AY=%6d AZ=%6d | GX=%6d GY=%6d GZ=%6d\t",
                    i + 1, sda, ax, ay, az, gx, gy, gz);
    } else {
      Serial.printf("MPU%d (SDA=%d): Read error\n", i + 1, sda);
    }
  }
  Serial.printf("\n");

  delay(500);
}
