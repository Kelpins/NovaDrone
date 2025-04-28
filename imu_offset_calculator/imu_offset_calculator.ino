#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("Calibrating... Keep the device VERY STILL!");
  delay(2000);  // Give yourself time to let go

  mpu.CalibrateAccel(6);  // 6 samples
  mpu.CalibrateGyro(6);

  Serial.println("Calibration done.");

  // Print out the offsets
  Serial.print("Accel offsets: ");
  Serial.print(mpu.getXAccelOffset()); Serial.print(", ");
  Serial.print(mpu.getYAccelOffset()); Serial.print(", ");
  Serial.print(mpu.getZAccelOffset()); Serial.println();

  Serial.print("Gyro offsets: ");
  Serial.print(mpu.getXGyroOffset()); Serial.print(", ");
  Serial.print(mpu.getYGyroOffset()); Serial.print(", ");
  Serial.print(mpu.getZGyroOffset()); Serial.println();
}

void loop() {
  // Nothing in loop
}
