#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

//all prop & receiver pins (digital)
const int ppm_pin = 7;
const int f_cw_pin = 9;
const int b_cw_pin = 10;
const int f_ccw_pin = 11;
const int b_ccw_pin = 12;

// timer to control serial monitor output
unsigned long timer = 0;

// variables all volatile because they are utilized in an interrupt
volatile unsigned long lastTime = 0;
volatile int channelIndex = 0;
volatile int channelValues[6] = {0};

// controller output declarations
int cout_roll = 1500;
int cout_pitch = 1500;
int cout_throttle = 1000;
int cout_yaw = 1500;
int cout_vra = 1000;
int cout_vrb = 1000;

// imu output declarations
double imu_roll = 0.0;
double imu_pitch = 0.0;
double imu_yaw = 0.0;

// scalar 0 -> 1; multuplies every output voltage
double vout_scalar = 0.0;

//scalar 0 -> 1; multiplies each motor voltage separately
double f_cw_scalar = 0.0;
double b_cw_scalar = 0.0;
double f_ccw_scalar = 0.0;
double b_ccw_scalar = 0.0;

// function runs on rising edge of PPM pulse (within interrupt)
void IRAM_ATTR ppmISR() {
  unsigned long now = micros();          // Current time in microseconds
  unsigned long pulse = now - lastTime;  // Time since last pulse
  lastTime = now;                        // Update last pulse time

  if (pulse > 3000) {
    // sync pulse detected (>3ms gap), start new frame
    channelIndex = 0;
  } else {
    // save pulse width for current channel
    if (channelIndex < 6) {
      channelValues[channelIndex] = pulse;
      channelIndex++;
    }
  }
}

void setup(){
  Serial.begin(19200);
  Wire.begin();

  byte status = mpu.begin();
  while (status!=0){Serial.println("IMU discnonected");} // stop if could not connect to MPU
  delay(1000);
  mpu.calcOffsets(true, true);

  pinMode(ppm_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ppm_pin), ppmISR, RISING); // Attach interrupt to read PPM
}

void updateStates() {

  mpu.update();
  cout_roll = channelValues[0]; // Channel 1 - tilting/moving left to right (roll control)
  cout_pitch = channelValues[1]; // Channel 2 - tilting/moving forward and back (pitch control)
  cout_throttle = channelValues[2]; // Channel 3 - power to motor (throttle control)
  cout_yaw = channelValues[3]; // Channel 4 - rotation (yaw control)
  cout_vra = channelValues[4]; // Channel 5 - first knob
  cout_vrb = channelValues[5]; // Channel 6 - second knob

  imu_roll = mpu.getAngleY();
  imu_pitch = mpu.getAngleX();
  imu_yaw = mpu.getAngleZ();

  if (millis() - timer > 1000) {
    Serial.print(F("TEMP: "));    Serial.println(mpu.getTemp());
    Serial.print(F("   X: "));       Serial.print(imu_pitch); // controls pitch
    Serial.print("\t | Y: ");        Serial.print(imu_roll); // controls roll
    Serial.print("\t| Z: ");        Serial.println(imu_yaw); // controlls yaw

    Serial.print("Roll: ");       Serial.print(cout_roll); // tilting and therefore moving side to side
    Serial.print("\t| Pitch: ");      Serial.print(cout_pitch); // tilting and therefore moving forward and backward
    Serial.print("\t| Yaw: ");        Serial.print(cout_yaw); // spinning in place
    Serial.print("\t| Throttle: ");   Serial.println(cout_throttle); // directly affecting power scalar
    Serial.print("Aux1: ");       Serial.print(cout_vra); //first auxilary knob on controller
    Serial.print("\t|  Aux2: ");       Serial.println(cout_vrb); //second auxilary knob on controller

    timer = millis();
  }

}

void adjustPitch() {

  // target angle (-20 -> 20) given a c_out (1000 -> 2000);
  // scaled on a polynomial
  double target_pitch = (1500.0 - cout_pitch) / 25.0;
  double current_pitch = imu_pitch;

  

}

void updateMotorVoltages() {

  adjustPitch();
  adjustRoll();
  adjustYaw();

}

void loop(){
  updateStates();
  updateMotorVoltages();
}