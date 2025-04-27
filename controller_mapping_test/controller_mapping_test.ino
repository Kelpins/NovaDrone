#include "Wire.h"
#include <MPU6050_light.h>
#include <ESP32Servo.h>
//#include <BluetoothSerial.h>

//BluetoothSerial //SerialBT;

MPU6050 mpu(Wire);

//all prop & receiver pins (digital)
const int ppm_pin = 7;
const int f_cw_pin = 9;
const int b_cw_pin = 10;
const int f_ccw_pin = 11;
const int b_ccw_pin = 12;

// timer to control serial monitor output
unsigned long timer = 0;
unsigned long timer2 = 0;
unsigned long timer3 = 0;
unsigned long timer4 = 0;

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
double v_scalar = 0.0;

// scalar 0 -> 1; multiplies each motor voltage separately
double f_cw_scalar = 0.5;
double b_cw_scalar = 0.5;
double f_ccw_scalar = 0.5;
double b_ccw_scalar = 0.5;

// scalar affecting smoothness of movements
const double scalar_change = 0.0025;

// prep Servo library for usage (wrapping the ledc pins)
Servo esc[4];

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

void setPWM(int pin, int pulse) {
  esc[pin-9].writeMicroseconds(pulse);
}

void setup(){
  Serial.begin(9600);
  //SerialBT.begin("ZippyZoom");
  Wire.begin();
  byte status = mpu.begin();

  delay(2000);

  if (status != 0){Serial.println("IMU disconnected");/*//SerialBT.println("IMU disconnected");*/}
  while (status!=0){Serial.println("IMU disconnected");delay(20);} // stop if could not connect to MPU

  if (pulseIn(ppm_pin, HIGH, 25000) < 1000) {Serial.println("RX disconnected");}
  while (pulseIn(ppm_pin, HIGH, 25000) < 1000) {Serial.println("RX disconnected");delay(20);}

  delay(1000);
  mpu.calcOffsets(true, true);

  Serial.println("imu connected - arming escs");
  //SerialBT.println("imu connected - arming escs");

  //"arming" each esc; basically setting it up to receive 1000 <= x <= 2000 microseconds of pwm, then sending it 1000 (motor off)
  esc[0].setPeriodHertz(50);
  esc[0].attach(f_cw_pin, 1000, 2000);
  esc[0].writeMicroseconds(1000);

  esc[1].setPeriodHertz(50);
  esc[1].attach(b_cw_pin, 1000, 2000);
  esc[1].writeMicroseconds(1000);

  esc[2].setPeriodHertz(50);
  esc[2].attach(f_ccw_pin, 1000, 2000);
  esc[2].writeMicroseconds(1000);

  esc[3].setPeriodHertz(50);
  esc[3].attach(b_ccw_pin, 1000, 2000);
  esc[3].writeMicroseconds(1000);

  setPWM(f_cw_pin, 1000);
  setPWM(b_cw_pin, 1000);
  setPWM(f_ccw_pin, 1000);
  setPWM(b_ccw_pin, 1000);

  pinMode(ppm_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ppm_pin), ppmISR, RISING); // Attach interrupt to read PPM

  delay(2500);
  Serial.println("ready!");
  //SerialBT.println("ready!");

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
    Serial.print(F("TEMP: ")); Serial.println(mpu.getTemp());
    Serial.print(F("   X: ")); Serial.print(imu_pitch); // controls pitch
    Serial.print("\t| Y: ");  Serial.print(imu_roll); // controls roll
    Serial.print("\t| Z: ");   Serial.println(imu_yaw); // controlls yaw

    Serial.print("Roll: ");         Serial.print(cout_roll); // tilting and therefore moving side to side
    Serial.print("\t| Pitch: ");    Serial.print(cout_pitch); // tilting and therefore moving forward and backward
    Serial.print("\t| Yaw: ");      Serial.print(cout_yaw); // spinning in place
    Serial.print("\t| Throttle: "); Serial.println(cout_throttle); // directly affecting power scalar
    Serial.print("Aux1: ");         Serial.print(cout_vra); //first auxilary knob on controller
    Serial.print("\t|  Aux2: ");    Serial.println(cout_vrb); //second auxilary knob on controller

    Serial.print("vscl: "); Serial.println(v_scalar);
    Serial.print(" fcw: "); Serial.println(f_cw_scalar);
    Serial.print(" bcw: "); Serial.println(b_cw_scalar);
    Serial.print("fccw: "); Serial.println(f_ccw_scalar);
    Serial.print("bccw: "); Serial.println(b_ccw_scalar);
    

    //SerialBT.print(F("TEMP: ")); Serial.println(mpu.getTemp());
    //SerialBT.print(F("   X: ")); Serial.print(imu_pitch); // controls pitch
    //SerialBT.print("\t| Y: ");  Serial.print(imu_roll); // controls roll
    //SerialBT.print("\t| Z: ");   Serial.println(imu_yaw); // controlls yaw

    //SerialBT.print("Roll: ");         Serial.print(cout_roll); // tilting and therefore moving side to side
    //SerialBT.print("\t| Pitch: ");    Serial.print(cout_pitch); // tilting and therefore moving forward and backward
    //SerialBT.print("\t| Yaw: ");      Serial.print(cout_yaw); // spinning in place
    //SerialBT.print("\t| Throttle: "); Serial.println(cout_throttle); // directly affecting power scalar
    //SerialBT.print("Aux1: ");         Serial.print(cout_vra); //first auxilary knob on controller
    //SerialBT.print("\t|  Aux2: ");    Serial.println(cout_vrb); //second auxilary knob on controller

    //SerialBT.print("vscl: "); Serial.println(v_scalar);
    //SerialBT.print(" fcw: "); Serial.println(f_cw_scalar);
    //SerialBT.print(" bcw: "); Serial.println(b_cw_scalar);
    //SerialBT.print("fccw: "); Serial.println(f_ccw_scalar);
    //SerialBT.print("bccw: "); Serial.println(b_ccw_scalar);

    timer = millis();
  }

}

double* adjustPitch() {

  // target angle (-20 -> 20) given a c_out (1000 -> 2000);
  double target_pitch = (1500.0 - cout_pitch) / 25.0;
  int adjusted_pitch = imu_pitch / 0.25;
  double current_pitch = adjusted_pitch * 0.25;

  //calculates how much power should go to each motor based on the difference in pitches
  //should use PID; direct voltage control is okay for now
  //essentially a scalar for the change in each scalar
  //-1: max difference forward
  //1: max difference backward
  double difference = (0.025) * (target_pitch - current_pitch);

  if (millis() - timer2 > 1000) {
    Serial.print("pitch difference: ");Serial.println(difference);
    timer2 = millis();
  }

  double f_cw = 1.0 - ((difference + 2.0) * 0.25);
  double b_cw = (difference + 2.0) * 0.25;
  double f_ccw = (difference + 2.0) * 0.25;
  double b_ccw = 1.0 - ((difference + 2.0) * 0.25);

  double* out_arr = new double[4]{f_cw, b_cw, f_ccw, b_ccw};
  return out_arr;

}

double* adjustRoll() {

  // target angle (-20 -> 20) given a c_out (1000 -> 2000);
  double target_roll = (1500.0 - cout_roll) / 25.0;
  int adjusted_roll = imu_roll / 0.25;
  double current_roll = adjusted_roll * 0.25;

  //calculates how much power should go to each motor based on the difference in pitches
  //should use PID; direct voltage control is okay for now
  //essentially a scalar for the change in each scalar
  //-1: max difference forward
  //1: max difference backward
  double difference = (0.025) * (target_roll - current_roll);

  if (millis() - timer3 > 1000) {
    Serial.print("roll difference2: ");Serial.println(difference);
    timer3 = millis();
  }

  double f_cw = (difference + 2.0) * 0.25;
  double b_cw = 1.0 - ((difference + 2.0) * 0.25);
  double f_ccw = (difference + 2.0) * 0.25;
  double b_ccw = 1.0 - ((difference + 2.0) * 0.25);

  double* out_arr = new double[4]{f_cw, b_cw, f_ccw, b_ccw};
  return out_arr;

}

//TODO: NEEDS ANGLE WRAPPING (can't go 360 lmao)
double* adjustYaw() {

  // target angle (-50 -> 50) given a c_out (1000 -> 2000);
  double target_yaw = (1500.0 - cout_yaw) / 10.0;
  int adjusted_yaw = imu_yaw / 0.25;
  double current_yaw = adjusted_yaw * 0.25;

  //calculates how much power should go to each motor based on the difference in pitches
  //should use PID; direct voltage control is okay for now
  //essentially a scalar for the change in each scalar
  //-1: max difference forward
  //1: max difference backward
  double difference = (0.025) * (target_yaw - current_yaw);

  if (millis() - timer4 > 1000) {
    Serial.print("yaw difference: ");Serial.println(difference);
    timer4 = millis();
  }

  double f_cw = (difference + 2.0) * 0.25;
  double b_cw = (difference + 2.0) * 0.25;
  double f_ccw = 1.0 - ((difference + 2.0) * 0.25);
  double b_ccw = 1.0 - ((difference + 2.0) * 0.25);

  double* out_arr = new double[4]{f_cw, b_cw, f_ccw, b_ccw};
  return out_arr;

}

void adjustThrottle() {}

void updateMotorVoltages() {

  //look more 
  //throttleVals = adjustThrottle();
  double* pitchVals = adjustPitch();
  double* rollVals = adjustRoll();
  //double* yawVals = adjustYaw();

  f_cw_scalar = (pitchVals[0] + rollVals[0]) / 2.0;
  b_cw_scalar = (pitchVals[1] + rollVals[1]) / 2.0;
  f_ccw_scalar = (pitchVals[2] + rollVals[2]) / 2.0;
  b_ccw_scalar = (pitchVals[3] + rollVals[3]) / 2.0;

  v_scalar = (cout_throttle - 1000.0) / 1000.0;
  if (cout_vra > 1050 || cout_vrb > 1050 || cout_throttle < 900) {
    v_scalar = 0.0;
  }

  //constrain motor scalars 0 <= scalar <= 1
  double f_cw_volt = 1000.0 * (f_cw_scalar * v_scalar + 1.0);
  double b_cw_volt = 1000.0 * (b_cw_scalar * v_scalar + 1.0);
  double f_ccw_volt = 1000.0 * (f_ccw_scalar * v_scalar + 1.0);
  double b_ccw_volt = 1000.0 * (b_ccw_scalar * v_scalar + 1.0);

  //setPWM(f_cw_pin, f_cw_volt);
  //setPWM(b_cw_pin, b_cw_volt);
  //setPWM(f_ccw_pin, f_ccw_volt);
  //setPWM(b_ccw_pin, b_ccw_volt);

}

void loop(){
  updateStates();
  updateMotorVoltages();
}