/* 
HARDWARE SET-UP (I2C PROTOCOL)

VCC --> 3.3V
GND --> GND
SCL --> pin A5
SDA --> pin A4
*/

#include <Wire.h>

#define MPU9250_ADDR   0x68     // standard I2C address 

// REGISTERS
#define PWR_MGMT_1     0x6B
#define ACCEL_CONFIG   0x1C
#define GYRO_CONFIG    0x1B
#define ACCEL_XOUT_H   0x3B
#define GYRO_XOUT_H    0x43

// Variable initialization
bool readAcc = true;
bool readGyro = false;

bool startStreaming = false;
unsigned long timestamp = 0;

float ax;
float ay;
float az;
float gx;
float gy;
float gz;


void setup() {

  Wire.begin();
  if(readAcc && readGyro) {Serial.begin(500000);}
  else {Serial.begin(230400);}

  // Software reset of the MPU-9250 (set DEVICE_RESET)
  writeRegister(PWR_MGMT_1, 0x80);
  delay(100);

  // Wake up the MPU-9250 (clear sleep bit)
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Accelerometer FSR ±2g (ACCEL_CONFIG: bit 4-3 = 00)
  if (readAcc) {writeRegister(ACCEL_CONFIG, 0x00);}

  // Gyroscope FSR ±250 °/s (GYRO_CONFIG: bit 4-3 = 00)
  if (readGyro) {writeRegister(GYRO_CONFIG, 0x00);}

  // Wait for 'S' command from Python
  while (!startStreaming) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'S') {
        startStreaming = true;
      }
    }
  }

}


void loop() {
  if (!startStreaming) return;

  if ((micros()-timestamp) >= 2000){
    timestamp = micros();

    if (readAcc) {
      uint8_t accel[6];
      readBytes(ACCEL_XOUT_H, 6, accel);

      // Accelerometer ±2g → 16384 LSB/g
      ax = convertBytes(accel[0], accel[1], 2.0);
      ay = convertBytes(accel[2], accel[3], 2.0);
      az = convertBytes(accel[4], accel[5], 2.0);
    }

    if (readGyro) {
      uint8_t gyro[6];
      readBytes(GYRO_XOUT_H, 6, gyro);

      // Gyroscope ±250 dps → 131 LSB/(°/s)
      gx = convertBytes(gyro[0], gyro[1], 250.0);
      gy = convertBytes(gyro[2], gyro[3], 250.0);
      gz = convertBytes(gyro[4], gyro[5], 250.0);
    }

    // Serial communication
    Serial.write(0xAA);
    if(readAcc){
      Serial.write((byte*)&ax, sizeof(float));
      Serial.write((byte*)&ay, sizeof(float));
      Serial.write((byte*)&az, sizeof(float));
    }
    if(readGyro){
      Serial.write((byte*)&gx, sizeof(float));
      Serial.write((byte*)&gy, sizeof(float));
      Serial.write((byte*)&gz, sizeof(float));
    }
    Serial.write((byte*)&timestamp, sizeof(unsigned long));
  }

}


// COMMUNICATION FUNCTIONS

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


void readBytes(uint8_t startReg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)count);
  for (uint8_t i = 0; i < count; i++) {
    if (Wire.available()) {
      dest[i] = Wire.read();
    }
  }
}


float convertBytes(uint8_t high, uint8_t low, float fsr) {
  int16_t value = (int16_t)((high << 8) | low);  
  return - (float)value * fsr / float(0x8000);
}
