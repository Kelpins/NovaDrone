#include "Wire.h"
//#include <MPU6050_light.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h> // For hostname resolution

// WiFi credentials
const char* ssid = "SIMOMIFI_G0U0_2.4GHz";
const char* password = "DwAJi54E";

// MySecureOTApass


// 192.168.76.215 -- use this ip in putty to connect to telnet server
// telnet server
WiFiServer wifiServer(23);
WiFiClient telnetClient;

//MPU6050 mpu(Wire);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//all prop & receiver pins (digital)
const int ppm_pin = 8;
const int f_cw_pin = 9;
const int b_cw_pin = 10;
const int f_ccw_pin = 11;
const int b_ccw_pin = 12;

// timer to control serial monitor output
unsigned long timer = 0;
unsigned long timer2 = 0;
unsigned long timer3 = 0;
unsigned long timer4 = 0;
unsigned long timer5 = 0;

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

double x_offset = 0.0;
double y_offset = 0.0;
double z_offset = 0.0;

// prep Servo library for usage (wrapping the ledc pins)
Servo esc[4];

// helper function to print output to telnet if connected, otherwise to serial
void printOutput(const char* message) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(message);
  } else {
    Serial.print(message);
  }
}

void printOutput(String message) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(message);
  } else {
    Serial.print(message);
  }
}

void printOutput(double value) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(value);
  } else {
    Serial.print(value);
  }
}

void printlnOutput(const char* message) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(message);
  } else {
    Serial.println(message);
  }
}

void printlnOutput(String message) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(message);
  } else {
    Serial.println(message);
  }
}

void printlnOutput(double value) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(value);
  } else {
    Serial.println(value);
  }
}

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

// setup wifi, telnet server, and OTA
void setupWiFiTelnet() {
  Serial.begin(9600); // match baud rate from other sketch, ensure monitor matches
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  printlnOutput("Connecting to Wi-Fi...");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    wl_status_t current_status = WiFi.status();
    delay(500);
    printOutput("."); 
    printOutput("("); printOutput((int)current_status); printOutput(")"); // Print status
    wifi_attempts++;
    if (wifi_attempts > 20) {
        printlnOutput("\nFailed to connect after 10 seconds. Retrying...");
        WiFi.disconnect(true); // disconnect before retry
        delay(1000); // wait a second before retrying
        WiFi.begin(ssid, password); // try connecting again
        wifi_attempts = 0;
    }
  }

  printlnOutput("\nWi-Fi connected!");
  printOutput("IP address: ");
  printlnOutput(WiFi.localIP());
  
  // Start mDNS responder
  if (MDNS.begin("esp32-nova-drone")) { 
    printlnOutput("mDNS responder started. Hostname: esp32-nova-drone.local");
    // Advertise Telnet service
    MDNS.addService("telnet", "tcp", 23);
  } else {
    printlnOutput("Error starting mDNS");
  }
  delay(100);

  ArduinoOTA.setHostname("esp32-nova-drone");
  ArduinoOTA.begin();
  printlnOutput("OTA Initialized. Hostname: esp32-nova-drone");
  delay(100);

  wifiServer.begin();
  wifiServer.setNoDelay(true);
  printlnOutput("Telnet Server Started. Ready for client connection.");
  delay(100);
}

// handle telnet client connection and OTA updates
void handleTelnetClient() {
  ArduinoOTA.handle();

  if (wifiServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = wifiServer.available();
      printlnOutput("New Telnet client connected");
      printlnOutput("ESP32 Controller Mapping - Sending data...");
    } else {
      // allow only one client
      WiFiClient newClient = wifiServer.available();
      newClient.stop();
    }
  }

  // check if client has sent any data (optional, could be used for commands)
  if (telnetClient && telnetClient.connected()) {
    if (telnetClient.available()) {
      // read and discard data for now, or implement command handling
      while(telnetClient.available()) {
        telnetClient.read();
      }
    }
  }
}

void setup(){
  setupWiFiTelnet(); // initialize wifi and telnet first

  Wire.begin();
  //byte status = mpu.begin();

  // initialize BNO055 sensor - loop until successful
  printlnOutput("Attempting to initialize BNO055...");
  while (!bno.begin()) {
    // failed to connect
    printlnOutput("BNO055 not detected. Retrying in 1 second...");
    delay(1000); // wait 1 second before retrying
  }
  printlnOutput("BNO055 Detected!"); // confirmation after successful connection

  delay(2000);

  sensors_event_t event;
  bno.getEvent(&event);

  x_offset = event.orientation.x;
  z_offset = event.orientation.z;
  y_offset = event.orientation.y;

  //if (status != 0){printlnOutput("IMU disconnected");}
  //while (status!=0){printlnOutput("IMU disconnected");delay(20);}

  // check for receiver/controler connection
  printOutput("Checking RX on pin "); printlnOutput((double)ppm_pin);
  delay(100);
  unsigned long pulseValue = pulseIn(ppm_pin, HIGH, 25000); // Read initial value
  printOutput("Initial pulseIn() value: "); printlnOutput((double)pulseValue);
  delay(100);

  int rx_check_count = 0;
  while (pulseValue < 1000 && rx_check_count < 50) {
    printOutput("RX disconnected - pulseIn value: "); printlnOutput((double)pulseValue);
    delay(100);
    pulseValue = pulseIn(ppm_pin, HIGH, 25000); // Read again
    rx_check_count++;
  }

  if (pulseValue < 1000) {
      printlnOutput("RX still disconnected after checks. Check wiring/power/binding/signal type!");
      // Optional: halt execution if RX is critical
      // while(1) { delay(100); }
  } else {
      printlnOutput("RX Detected! Proceeding...");
  }
  delay(100);

  delay(1000);
  //mpu.calcOffsets(true, true);

  bno.setExtCrystalUse(true);
  printlnOutput("BNO055 IMU Initialized");
  delay(100);

  printlnOutput("imu connected - arming escs");

  // "arming" each esc; basically setting it up to receive 1000 <= x <= 2000 microseconds of pwm, then sending it 1000 (motor off)
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
  printlnOutput("ready!");

}

void updateStates() {

  //mpu.update();

  sensors_event_t event;
  bno.getEvent(&event); 
  
  cout_roll = channelValues[0]; // Channel 1 - tilting/moving left to right (roll control)
  cout_pitch = channelValues[1]; // Channel 2 - tilting/moving forward and back (pitch control)
  cout_throttle = channelValues[2]; // Channel 3 - power to motor (throttle control)
  cout_yaw = channelValues[3]; // Channel 4 - rotation (yaw control)
  cout_vra = channelValues[4]; // Channel 5 - first knob
  cout_vrb = channelValues[5]; // Channel 6 - second knob

  //imu_roll = mpu.getAngleY();
  //imu_pitch = mpu.getAngleX();
  //imu_yaw = mpu.getAngleZ();
  
  imu_roll = event.orientation.x - x_offset;
  imu_pitch = event.orientation.z - z_offset;
  imu_yaw = event.orientation.y - y_offset;

  if (millis() - timer > 1000) {
    //printOutput(F("TEMP: ")); printlnOutput(mpu.getTemp());
    printOutput(F("   X: ")); printOutput(imu_pitch); // controls pitch
    printOutput("\t| Y: ");   printOutput(imu_roll); // controls roll
    printOutput("\t| Z: ");   printlnOutput(imu_yaw); // controlls yaw

    printOutput("Roll: ");        printOutput(cout_roll);
    printOutput(" | Pitch: ");    printOutput(cout_pitch);
    printOutput(" | Throttle: "); printOutput(cout_throttle);
    printOutput(" | Yaw: ");      printOutput(cout_yaw);
    printOutput(" | Aux1: ");     printOutput(cout_vra);
    printOutput(" | Aux2: ");     printlnOutput(cout_vrb);

    printOutput("vscl: "); printlnOutput(v_scalar);
    printOutput(" fcw: "); printlnOutput(f_cw_scalar);
    printOutput(" bcw: "); printlnOutput(b_cw_scalar);
    printOutput("fccw: "); printlnOutput(f_ccw_scalar);
    printOutput("bccw: "); printlnOutput(b_ccw_scalar);
    

    timer = millis();
  }

}

double* adjustPitch() {

  // target angle (-20 -> 20) given a c_out (1000 -> 2000);
  double target_pitch = (1500.0 - cout_pitch) / 25.0;
  int adjusted_pitch = imu_pitch / 0.25;
  double current_pitch = adjusted_pitch * 0.25;

  // calculates how much power should go to each motor based on the difference in pitches
  // should use PID; direct voltage control is okay for now
  // essentially a scalar for the change in each scalar
  // -1: max difference forward
  // 1: max difference backward
  double difference = (0.025) * (target_pitch - current_pitch);

  if (millis() - timer2 > 1000) {
    printOutput("pitch difference: ");printlnOutput(difference);
    timer2 = millis();
  }

  double f_cw = 0.5 + difference;
  double b_cw = 0.5 - difference;
  double f_ccw = 0.5 + difference;
  double b_ccw = 0.5 - difference;

  double* out_arr = new double[4]{f_cw, b_cw, f_ccw, b_ccw};
  return out_arr;

}

double* adjustRoll() {

  // target angle (-20 -> 20) given a c_out (1000 -> 2000);
  double target_roll = (1500.0 - cout_roll) / 25.0;
  int adjusted_roll = imu_roll / 0.25;
  double current_roll = adjusted_roll * 0.25;

  // calculates how much power should go to each motor based on the difference in pitches
  // should use PID; direct voltage control is okay for now
  // essentially a scalar for the change in each scalar
  // -1: max difference forward
  // 1: max difference backward
  double difference = (0.025) * (target_roll - current_roll);

  if (millis() - timer3 > 1000) {
    printOutput("roll difference: ");printlnOutput(difference);
    timer3 = millis();
  }

  double f_cw = 0.5 - difference;
  double b_cw = 0.5 - difference;
  double f_ccw = 0.5 + difference;
  double b_ccw = 0.5 + difference;

  double* out_arr = new double[4]{f_cw, b_cw, f_ccw, b_ccw};
  return out_arr;

}

//TODO: NEEDS ANGLE WRAPPING (can't go 360 lmao)
double* adjustYaw() {

  // target angle (-50 -> 50) given a c_out (1000 -> 2000);
  double target_yaw = (1500.0 - cout_yaw) / 10.0;
  int adjusted_yaw = imu_yaw / 2.5;
  double current_yaw = adjusted_yaw * 2.5;

  // calculates how much power should go to each motor based on the difference in pitches
  // should use PID; direct voltage control is okay for now
  // essentially a scalar for the change in each scalar
  // -1: max difference forward
  // 1: max difference backward
  double difference = (0.025) * (target_yaw - current_yaw);

  if (millis() - timer4 > 1000) {
    printOutput("yaw difference: ");printlnOutput(difference);
    timer4 = millis();
  }

  double f_cw = 0.5 + difference;
  double b_cw = 0.5 + difference;
  double f_ccw = 0.5 - difference;
  double b_ccw = 0.5 - difference;

  double* out_arr = new double[4]{f_cw, b_cw, f_ccw, b_ccw};
  return out_arr;

}

void adjustThrottle() {}

void updateMotorVoltages() {

  // look more 
  // throttleVals = adjustThrottle();
  double* pitchVals = adjustPitch();
  double* rollVals = adjustRoll();
  double* yawVals = adjustYaw();

  f_cw_scalar = (pitchVals[0] + rollVals[0] + yawVals[0]) / 3.0;
  b_cw_scalar = (pitchVals[1] + rollVals[1] + yawVals[1]) / 3.0;
  f_ccw_scalar = (pitchVals[2] + rollVals[2] + yawVals[2]) / 3.0;
  b_ccw_scalar = (pitchVals[3] + rollVals[3] + yawVals[3]) / 3.0;

  v_scalar = (cout_throttle - 1000.0) / 1000.0;
  if (cout_vra > 1050 || cout_vrb > 1050 || cout_throttle < 900) {
    v_scalar = 0.0;
  }

  if (millis() - timer5 > 1000) {
    printOutput("fcw pre: ");printlnOutput(f_cw_scalar);
    printOutput("bcw pre: ");printlnOutput(b_cw_scalar);
    printOutput("fccw pre: ");printlnOutput(f_ccw_scalar);
    printOutput("bccw pre: ");printlnOutput(b_ccw_scalar);
    timer5 = millis();
  }

  // normalize motor scalars
  double max_val = max(max(f_cw_scalar, b_cw_scalar), max(f_ccw_scalar, b_ccw_scalar));
  if (max_val > 1.0) {
    f_cw_scalar /= max_val;
    b_cw_scalar /= max_val;
    f_ccw_scalar /= max_val;
    b_ccw_scalar /= max_val;
  }

  // calculate final motor voltages with base offset
  double f_cw_volt = 1000.0 + (f_cw_scalar * v_scalar * 1000.0);
  double b_cw_volt = 1000.0 + (b_cw_scalar * v_scalar * 1000.0);
  double f_ccw_volt = 1000.0 + (f_ccw_scalar * v_scalar * 1000.0);
  double b_ccw_volt = 1000.0 + (b_ccw_scalar * v_scalar * 1000.0);

  // constrain motor voltages 1000 <= volt <= 2000
  f_cw_volt = constrain(f_cw_volt, 1000.0, 2000.0);
  b_cw_volt = constrain(b_cw_volt, 1000.0, 2000.0);
  f_ccw_volt = constrain(f_ccw_volt, 1000.0, 2000.0);
  b_ccw_volt = constrain(b_ccw_volt, 1000.0, 2000.0);

  setPWM(f_cw_pin, f_cw_volt);
  setPWM(b_cw_pin, b_cw_volt);
  setPWM(f_ccw_pin, f_ccw_volt);
  setPWM(b_ccw_pin, b_ccw_volt);

  // clean up dynamically allocated memory
  delete[] pitchVals;
  delete[] rollVals;
  delete[] yawVals;

}

void loop(){
  handleTelnetClient(); // handle telnet client connection and OTA
  updateStates();
  updateMotorVoltages();
}