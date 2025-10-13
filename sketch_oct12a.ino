#include "MultiSDA_I2C.h"

#define SCL_PIN 22
#define SDA1_PIN 21
#define SDA2_PIN 17

#define MPU6050_ADDR 0x68

MultiSDA_I2C multiI2C(SCL_PIN);

// MPU6050 register addresses
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Multi-SDA I2C MPU6050 test...");

  const uint8_t sdaPins[] = {SDA1_PIN, SDA2_PIN};
  multiI2C.begin(sdaPins, 2);

  // Wake up both MPU6050s (disable sleep)
  uint8_t data[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t sdaPin = sdaPins[i];
    if (!multiI2C.writeBytes(sdaPin, MPU6050_ADDR, data, 2, true)) {
      Serial.printf("MPU6050 on SDA pin %d not responding!\n", sdaPin);
    } else {
      Serial.printf("MPU6050 on SDA pin %d initialized.\n", sdaPin);
    }
    delay(50);
  }
}

void loop() {
  readAndPrintMPU(SDA1_PIN, "MPU1");
  readAndPrintMPU(SDA2_PIN, "MPU2");

  Serial.println();
  delay(500);
}

void readAndPrintMPU(uint8_t sdaPin, const char *label) {
  uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
  uint8_t rawData[14]; // accel (6), temp (2), gyro (6)

  // Write starting register address
  if (!multiI2C.writeBytes(sdaPin, MPU6050_ADDR, &reg, 1, true)) {
    Serial.printf("[%s] Write failed on SDA %d\n", label, sdaPin);
    return;
  }

  // Read 14 bytes of data
  if (!multiI2C.readBytes(sdaPin, MPU6050_ADDR, rawData, 14, true)) {
    Serial.printf("[%s] Read failed on SDA %d\n", label, sdaPin);
    return;
  }

  int16_t ax = (rawData[0] << 8) | rawData[1];
  int16_t ay = (rawData[2] << 8) | rawData[3];
  int16_t az = (rawData[4] << 8) | rawData[5];
  int16_t gx = (rawData[8] << 8) | rawData[9];
  int16_t gy = (rawData[10] << 8) | rawData[11];
  int16_t gz = (rawData[12] << 8) | rawData[13];

  Serial.printf("[%s] Accel: X=%6d Y=%6d Z=%6d | Gyro: X=%6d Y=%6d Z=%6d\n",
                label, ax, ay, az, gx, gy, gz);
}
