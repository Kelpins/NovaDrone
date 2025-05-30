#include "Wire.h"
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

// PID Configuration
// Pitch Axis PID Gains (NEEDS CAREFUL TUNING!)
double Kp_pitch = 0.025; // Initial Kp based on user's original proportional scaling
double Ki_pitch = 0.0;   // Start Ki at 0
double Kd_pitch = 0.0;   // Start Kd at 0
// Roll Axis PID Gains (NEEDS CAREFUL TUNING!)
double Kp_roll = 0.025;  // Initial Kp based on user's original proportional scaling
double Ki_roll = 0.0;    // Start Ki at 0
double Kd_roll = 0.0;    // Start Kd at 0
// Yaw Axis PID Gains (NEEDS CAREFUL TUNING!)
// User's original yaw 'scaled_difference' was error/90.0. Kp_yaw could be ~1/90 = 0.011
double Kp_yaw = 0.015;   // Start Kp for yaw
double Ki_yaw = 0.0;     // Start Ki at 0
double Kd_yaw = 0.0;     // Start Kd at 0

// PID Output Scaling Factors
// These can be used to adjust the magnitude of the PID output if Kp alone doesn't provide the desired range.
// For now, assuming Kp values are chosen to give appropriate output magnitudes, so scaling is 1.0.
const double PID_OUTPUT_SCALING_PITCH = 1.0;
const double PID_OUTPUT_SCALING_ROLL = 1.0;
const double PID_OUTPUT_SCALING_YAW = 1.0;

// PID Limits and Safety
const double MAX_PID_INTEGRAL = 50.0;          // Anti-windup limit for integral term (tune this)
const double MAX_TARGET_PITCH_ROLL = 20.0;     // Max target pitch/roll angle command for safety
const double MAX_ABS_PITCH_ROLL_SAFETY = 35.0; // Safety cutoff for actual pitch/roll angle

// PID State Variables
double pitch_error_integral = 0.0;
double pitch_previous_error = 0.0;
double roll_error_integral = 0.0;
double roll_previous_error = 0.0;
double yaw_error_integral = 0.0;
double yaw_previous_error = 0.0;

unsigned long last_pid_time = 0; // For calculating dt

// Original global variables
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

const int ppm_pin = 8;
const int f_cw_pin = 9;
const int b_cw_pin = 10;
const int f_ccw_pin = 11;
const int b_ccw_pin = 12;

unsigned long timer = 0; // For general serial output
unsigned long timer_pid_debug = 0; // For PID specific debug output

volatile unsigned long lastTime = 0; // For PPM ISR
volatile int channelIndex = 0;       // For PPM ISR
volatile int channelValues[6] = {0}; // For PPM ISR

int cout_roll = 1500;
int cout_pitch = 1500;
int cout_throttle = 1000;
int cout_yaw = 1500;
int cout_vra = 1000;
int cout_vrb = 1000;

double current_yaw_target = 0.0; // Target yaw heading, updated incrementally

double imu_roll = 0.0;
double imu_pitch = 0.0;
double imu_yaw = 0.0;

double v_scalar = 0.0;

double f_cw_scalar = 0.5;
double b_cw_scalar = 0.5;
double f_ccw_scalar = 0.5;
double b_ccw_scalar = 0.5;

const double scalar_change = 0.035; // User's original factor for yaw target update rate

// Global variables for differences calculated in adjust functions (for telnet output)
double pitch_diff = 0.0;
double roll_diff = 0.0;
double yaw_diff = 0.0;

double x_offset = 0.0;
double y_offset = 0.0;
double z_offset = 0.0;

Servo esc[4];

WiFiServer wifiServer(23);
WiFiClient telnetClient;

// Helper function to print output to telnet if connected, otherwise to serial
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

// Function runs on rising edge of PPM pulse (within interrupt)
void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long pulse = now - lastTime;
  lastTime = now;

  if (pulse > 3000) {
    channelIndex = 0;
  } else {
    if (channelIndex < 6) {
      channelValues[channelIndex] = pulse;
      channelIndex++;
    }
  }
}

void setPWM(int pin, int pulse) {
  esc[pin-9].writeMicroseconds(pulse);
}

void setupWiFiTelnet() {
  Serial.begin(9600); // Changed from 115200 to match updated_drone_movement.ino
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  printlnOutput("Connecting to Wi-Fi...");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    wl_status_t current_status = WiFi.status();
    delay(500);
    printOutput("."); 
    printOutput("("); printOutput((int)current_status); printOutput(")");
    wifi_attempts++;
    if (wifi_attempts > 20) {
        printlnOutput("Failed to connect after 10 seconds. Retrying...");
        WiFi.disconnect(true);
        delay(1000);
        WiFi.begin(ssid, password);
        wifi_attempts = 0;
    }
  }

  printlnOutput("Wi-Fi connected!");
  printOutput("IP address: ");
  printlnOutput(WiFi.localIP().toString()); // Use toString() for IPAddress
  
  if (MDNS.begin("esp32-nova-drone")) { 
    printlnOutput("mDNS responder started. Hostname: esp32-nova-drone.local");
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

void handleTelnetClient() {
  ArduinoOTA.handle();

  if (wifiServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = wifiServer.available();
      printlnOutput("New Telnet client connected");
      printlnOutput("ESP32 PID Drone Controller - Sending CSV data...");
      printlnOutput("Format: imu_yaw,imu_pitch,imu_roll,cout_roll,cout_pitch,cout_throttle,cout_yaw,pitch_diff,roll_diff,current_yaw_target,yaw_diff,v_scalar,f_cw_scalar,b_cw_scalar,f_ccw_scalar,b_ccw_scalar");
    } else {
      WiFiClient newClient = wifiServer.available();
      newClient.stop();
    }
  }

  if (telnetClient && telnetClient.connected()) {
    if (telnetClient.available()) {
      while(telnetClient.available()) {
        telnetClient.read(); // Discard data for now
      }
    }
  }
}

// Helper function to normalize an angle to the -180 to +180 degree range
double normalizeAngle180(double angle) {
  angle = fmod(angle, 360.0);
  if (angle > 180.0) {
    angle -= 360.0;
  } else if (angle <= -180.0) { // Use <= for -180 to become +180 if desired, or < to keep -180 as is. Let's keep -180.
    angle += 360.0;
  }
  return angle;
}

// Generic PID calculation function
double calculatePID(double current_value, double target_value,
                    double Kp, double Ki, double Kd,
                    double &integral_term, double &previous_error, double dt) {
  if (dt <= 1e-6) dt = 1e-6; // Prevent division by zero or extreme values if dt is too small

  double error = target_value - current_value;

  // Proportional term
  double P_out = Kp * error;

  // Integral term (with anti-windup)
  integral_term += error * dt;
  integral_term = constrain(integral_term, -MAX_PID_INTEGRAL, MAX_PID_INTEGRAL);
  double I_out = Ki * integral_term;

  // Derivative term
  double derivative = (error - previous_error) / dt;
  double D_out = Kd * derivative;

  previous_error = error; // Update previous error for the next iteration

  return P_out + I_out + D_out; // Total PID output
}


void setup(){
  setupWiFiTelnet(); 

  Wire.begin();
  
  printlnOutput("Attempting to initialize BNO055...");
  while (!bno.begin()) {
    printlnOutput("BNO055 not detected. Retrying in 1 second...");
    delay(1000);
  }
  printlnOutput("BNO055 Detected!"); 

  delay(1000); // Allow BNO055 to settle
  bno.setExtCrystalUse(true); // Recommended for stability
  printlnOutput("BNO055 External Crystal Enabled.");

  // Capture initial offsets
  sensors_event_t event;
  for (int i=0; i<10; i++) { // Take a few readings to stabilize
      bno.getEvent(&event);
      delay(50);
  }
  x_offset = event.orientation.x; // Yaw offset
  y_offset = event.orientation.y; // Pitch offset
  z_offset = event.orientation.z; // Roll offset
  current_yaw_target = 0; // Initialize target yaw relative to initial orientation (which becomes 0 after offset)
  printlnOutput("IMU offsets captured.");
  printOutput("X_offset (Yaw): "); printlnOutput(x_offset);
  printOutput("Y_offset (Pitch): "); printlnOutput(y_offset);
  printOutput("Z_offset (Roll): "); printlnOutput(z_offset);

  printOutput("Checking RX on pin "); printlnOutput((double)ppm_pin);
  delay(100);
  unsigned long pulseValue = pulseIn(ppm_pin, HIGH, 25000); 
  printOutput("Initial pulseIn() value: "); printlnOutput((double)pulseValue);
  delay(100);

  int rx_check_count = 0;
  while (pulseValue < 1000 && rx_check_count < 50) {
    printOutput("RX disconnected - pulseIn value: "); printlnOutput((double)pulseValue);
    delay(100);
    pulseValue = pulseIn(ppm_pin, HIGH, 25000); 
    rx_check_count++;
  }

  if (pulseValue < 1000) {
      printlnOutput("RX still disconnected after checks. Check wiring/power/binding/signal type!");
  } else {
      printlnOutput("RX Detected! Proceeding...");
  }
  delay(100);

  printlnOutput("Arming ESCs...");
  esc[0].setPeriodHertz(50); esc[0].attach(f_cw_pin, 1000, 2000); esc[0].writeMicroseconds(1000);
  esc[1].setPeriodHertz(50); esc[1].attach(b_cw_pin, 1000, 2000); esc[1].writeMicroseconds(1000);
  esc[2].setPeriodHertz(50); esc[2].attach(f_ccw_pin, 1000, 2000); esc[2].writeMicroseconds(1000);
  esc[3].setPeriodHertz(50); esc[3].attach(b_ccw_pin, 1000, 2000); esc[3].writeMicroseconds(1000);
  delay(1000); // Wait for ESCs to arm

  pinMode(ppm_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ppm_pin), ppmISR, RISING); 

  last_pid_time = micros(); // Initialize time for first PID calculation
  delay(1000); // Additional delay before starting main loop
  printlnOutput("Ready for flight!");
}

void updateStates() {
  sensors_event_t event;
  bno.getEvent(&event); 
  
  cout_roll = channelValues[0]; 
  cout_pitch = channelValues[1]; 
  cout_throttle = channelValues[2]; 
  cout_yaw = channelValues[3]; 
  cout_vra = channelValues[4]; 
  cout_vrb = channelValues[5]; 
  
  // Apply offsets and map to drone's coordinate system
  // User: x is yaw, y is pitch, z is roll
  imu_yaw = normalizeAngle180(event.orientation.x); // No offset applied, matching updated_drone_movement
  imu_pitch = event.orientation.y - y_offset; // BNO055 pitch is typically -90 to +90
  imu_roll = event.orientation.z - z_offset;   // BNO055 roll is typically -180 to +180 (or -90 to +90 if only one axis used)
                                             // Ensure roll is also handled correctly if it wraps. For now, direct subtraction.

  if (millis() - timer > 100) {  // Changed from 1000ms to 100ms
    // Construct the CSV string (matching updated_drone_movement format)
    String dataString = String(imu_yaw) + "," +
                        String(imu_pitch) + "," +
                        String(imu_roll) + "," +
                        String(cout_roll) + "," +
                        String(cout_pitch) + "," +
                        String(cout_throttle) + "," +
                        String(cout_yaw) + "," +
                        String(pitch_diff) + "," +
                        String(roll_diff) + "," +
                        String(current_yaw_target) + "," +
                        String(yaw_diff) + "," +
                        String(v_scalar) + "," +
                        String(f_cw_scalar) + "," +
                        String(b_cw_scalar) + "," +
                        String(f_ccw_scalar) + "," +
                        String(b_ccw_scalar);
    printlnOutput(dataString);
    
    timer = millis();
  }
}

// Calculates pitch motor contributions based on PID
void adjustPitchPID(double current_pitch_angle, int controller_pitch_input, double dt, double* contributions) {
    // 1. Calculate Target Pitch (user's existing logic)
    double target_pitch = (1500.0 - controller_pitch_input) / 25.0;
    // Constrain target_pitch to safety limits
    target_pitch = constrain(target_pitch, -MAX_TARGET_PITCH_ROLL, MAX_TARGET_PITCH_ROLL);

    // 2. Calculate PID Output
    double pid_output = calculatePID(current_pitch_angle, target_pitch,
                                     Kp_pitch, Ki_pitch, Kd_pitch,
                                     pitch_error_integral, pitch_previous_error, dt);

    // 3. Scale PID output
    double scaled_pid_output = pid_output * PID_OUTPUT_SCALING_PITCH;
    
    // Capture pitch error for telnet output (matching original's logic)
    pitch_diff = scaled_pid_output; // This represents the correction being applied

    // 4. Calculate motor contributions for pitch
    // Positive scaled_pid_output commands nose down (front motors decrease, rear increase)
    contributions[0] = 0.5 + scaled_pid_output; // f_cw (front)
    contributions[1] = 0.5 - scaled_pid_output; // b_cw (rear)
    contributions[2] = 0.5 + scaled_pid_output; // f_ccw (front)
    contributions[3] = 0.5 - scaled_pid_output; // b_ccw (rear)
}

// Calculates roll motor contributions based on PID
void adjustRollPID(double current_roll_angle, int controller_roll_input, double dt, double* contributions) {
    // 1. Calculate Target Roll
    double target_roll = (1500.0 - controller_roll_input) / 25.0; // Positive input = roll left
    target_roll = constrain(target_roll, -MAX_TARGET_PITCH_ROLL, MAX_TARGET_PITCH_ROLL);

    // 2. Calculate PID Output
    double pid_output = calculatePID(current_roll_angle, target_roll,
                                     Kp_roll, Ki_roll, Kd_roll,
                                     roll_error_integral, roll_previous_error, dt);
    // 3. Scale PID output
    double scaled_pid_output = pid_output * PID_OUTPUT_SCALING_ROLL;
    
    // Capture roll error for telnet output
    roll_diff = scaled_pid_output; // This represents the correction being applied

    // 4. Calculate motor contributions for roll
    // Positive scaled_pid_output commands roll left (left motors decrease, right increase)
    // User's stick: 1000 (right) -> 2000 (left). (1500 - cout_roll) means positive is left.
    // So positive scaled_pid_output should make left motors decrease.
    contributions[0] = 0.5 + scaled_pid_output; // f_cw (front left)
    contributions[1] = 0.5 + scaled_pid_output; // b_cw (rear left)
    contributions[2] = 0.5 - scaled_pid_output; // f_ccw (front right)
    contributions[3] = 0.5 - scaled_pid_output; // b_ccw (rear right)
}

// Updates yaw target and calculates yaw motor contributions based on PID
void updateYawTargetAndContributionsPID(double current_yaw_angle, int controller_yaw_input, double dt, double* contributions) {
    // 1. Update Target Yaw (user's existing logic, interpreted as per-loop rate adjustment)
    double adjusted_yaw_stick_input = (1500.0 - controller_yaw_input) / 1000.0; // Range approx -0.5 to 0.5
    current_yaw_target -= (scalar_change * adjusted_yaw_stick_input); // scalar_change is per-loop rate factor
    current_yaw_target = normalizeAngle180(current_yaw_target);

    // 2. Calculate Yaw Error (shortest angle)
    double yaw_error = current_yaw_target - current_yaw_angle;
    yaw_error = normalizeAngle180(yaw_error); // Ensure error is shortest path
    
    // Capture raw yaw difference for telnet output (before PID processing)
    yaw_diff = yaw_error;

    // 3. Calculate PID Output (target for PID is current + shortest_error)
    double effective_target_yaw = current_yaw_angle + yaw_error;
    double pid_output = calculatePID(current_yaw_angle, effective_target_yaw,
                                     Kp_yaw, Ki_yaw, Kd_yaw,
                                     yaw_error_integral, yaw_previous_error, dt);
    // 4. Scale PID output
    double scaled_pid_output = pid_output * PID_OUTPUT_SCALING_YAW;

    // 5. Calculate motor contributions for yaw
    // Positive scaled_pid_output commands yaw right/clockwise
    // CW motors (e.g., FR, BL) decrease power; CCW motors (e.g., FL, BR) increase power.
    // Assuming motor layout: FL (f_cw, CCW), FR (f_ccw, CW), BL (b_cw, CW), BR (b_ccw, CCW) - check this
    // User's pinout: f_cw_pin (9), b_cw_pin (10), f_ccw_pin (11), b_ccw_pin (12)
    // Let's assume f_cw and b_ccw are CCW (increase for CW yaw)
    // And b_cw and f_ccw are CW (decrease for CW yaw)
    contributions[0] = 0.5 - scaled_pid_output; // f_cw (CCW) - User had this as 0.5 - scaled_difference for CW yaw
    contributions[1] = 0.5 + scaled_pid_output; // b_cw (CW)   - User had this as 0.5 - scaled_difference for CW yaw
    contributions[2] = 0.5 + scaled_pid_output; // f_ccw (CW)  - User had this as 0.5 + scaled_difference for CW yaw
    contributions[3] = 0.5 - scaled_pid_output; // b_ccw (CCW) - User had this as 0.5 + scaled_difference for CW yaw
    // Re-checking user's original adjustYaw:
    // f_cw = 0.5 - scaled_difference; b_cw = 0.5 - scaled_difference;
    // f_ccw = 0.5 + scaled_difference; b_ccw = 0.5 + scaled_difference;
    // This means CW props (f_ccw, b_ccw) speed up for positive scaled_difference (CW yaw)
    // And CCW props (f_cw, b_cw) slow down for positive scaled_difference (CW yaw)
    // This is slightly unusual. Typically for CW yaw, CW props slow down, CCW speed up.
    // Let's stick to user's original logic interpretation for now.
    // If positive scaled_pid_output means "yaw right/clockwise":
    contributions[0] = 0.5 - scaled_pid_output; // f_cw
    contributions[1] = 0.5 - scaled_pid_output; // b_cw
    contributions[2] = 0.5 + scaled_pid_output; // f_ccw
    contributions[3] = 0.5 + scaled_pid_output; // b_ccw
}


void updateMotorVoltagesPID(double dt) {
  double pitch_contributions[4];
  double roll_contributions[4];
  double yaw_contributions[4];

  adjustPitchPID(imu_pitch, cout_pitch, dt, pitch_contributions);
  adjustRollPID(imu_roll, cout_roll, dt, roll_contributions);
  updateYawTargetAndContributionsPID(imu_yaw, cout_yaw, dt, yaw_contributions);

  // Average the contributions from each axis (user's original method)
  f_cw_scalar = (pitch_contributions[0] + roll_contributions[0] + yaw_contributions[0]) / 3.0;
  b_cw_scalar = (pitch_contributions[1] + roll_contributions[1] + yaw_contributions[1]) / 3.0;
  f_ccw_scalar = (pitch_contributions[2] + roll_contributions[2] + yaw_contributions[2]) / 3.0;
  b_ccw_scalar = (pitch_contributions[3] + roll_contributions[3] + yaw_contributions[3]) / 3.0;

  // Calculate overall throttle scalar
  v_scalar = (cout_throttle - 1000.0) / 1000.0;
  v_scalar = constrain(v_scalar, 0.0, 1.0); // Ensure v_scalar is not negative or overly large

  // Original safety cutoffs
  if (cout_vra > 1050 || cout_vrb > 1050 || cout_throttle < 950) { // Adjusted throttle cutoff slightly
    v_scalar = 0.0;
  }

  // New safety cutoff based on actual drone attitude
  if (abs(imu_pitch) > MAX_ABS_PITCH_ROLL_SAFETY || abs(imu_roll) > MAX_ABS_PITCH_ROLL_SAFETY) {
    v_scalar = 0.0;
    if(millis() - timer_pid_debug > 1000) { // Avoid spamming
        printlnOutput("!!! SAFETY CUTOFF: Pitch/Roll angle exceeded limits !!!");
    }
  }
  
  // Normalize motor scalars (user's original method)
  double max_s = 0.0;
  max_s = max(f_cw_scalar, max_s);
  max_s = max(b_cw_scalar, max_s);
  max_s = max(f_ccw_scalar, max_s);
  max_s = max(b_ccw_scalar, max_s);

  if (max_s > 1.0) { // Only normalize if any scalar exceeds 1.0 to preserve ratios below 1.0
    f_cw_scalar /= max_s;
    b_cw_scalar /= max_s;
    f_ccw_scalar /= max_s;
    b_ccw_scalar /= max_s;
  }
  // Also ensure no scalar is negative after averaging if PID outputs are large
  f_cw_scalar = max(0.0, f_cw_scalar);
  b_cw_scalar = max(0.0, b_cw_scalar);
  f_ccw_scalar = max(0.0, f_ccw_scalar);
  b_ccw_scalar = max(0.0, b_ccw_scalar);


  // Debug output commented out to prevent interference with CSV data stream
  /*
  if (millis() - timer_pid_debug > 1000) {
    printOutput("PID Targets P: "); printOutput(constrain((1500.0 - cout_pitch) / 25.0, -MAX_TARGET_PITCH_ROLL, MAX_TARGET_PITCH_ROLL));
    printOutput(" R: "); printOutput(constrain((1500.0 - cout_roll) / 25.0, -MAX_TARGET_PITCH_ROLL, MAX_TARGET_PITCH_ROLL));
    printOutput(" Y: "); printlnOutput(current_yaw_target);
    
    printOutput("Motor Scalars Pre-V: fcw:"); printOutput(f_cw_scalar);
    printOutput(" bcw:"); printOutput(b_cw_scalar);
    printOutput(" fccw:"); printOutput(f_ccw_scalar);
    printOutput(" bccw:"); printlnOutput(b_ccw_scalar);
    printOutput("V_Scalar: "); printlnOutput(v_scalar);
    timer_pid_debug = millis();
  }
  */

  // Calculate final motor voltages with base offset
  double f_cw_volt = 1000.0 + (f_cw_scalar * v_scalar * 1000.0);
  double b_cw_volt = 1000.0 + (b_cw_scalar * v_scalar * 1000.0);
  double f_ccw_volt = 1000.0 + (f_ccw_scalar * v_scalar * 1000.0);
  double b_ccw_volt = 1000.0 + (b_ccw_scalar * v_scalar * 1000.0);

  // Constrain motor voltages 1000 <= volt <= 2000
  f_cw_volt = constrain(f_cw_volt, 1000.0, 2000.0);
  b_cw_volt = constrain(b_cw_volt, 1000.0, 2000.0);
  f_ccw_volt = constrain(f_ccw_volt, 1000.0, 2000.0);
  b_ccw_volt = constrain(b_ccw_volt, 1000.0, 2000.0);

  setPWM(f_cw_pin, (int)f_cw_volt);
  setPWM(b_cw_pin, (int)b_cw_volt);
  setPWM(f_ccw_pin, (int)f_ccw_volt);
  setPWM(b_ccw_pin, (int)b_ccw_volt);
}

void loop(){
  handleTelnetClient(); 

  unsigned long current_time = micros();
  double dt = (current_time - last_pid_time) / 1000000.0; // dt in seconds
  last_pid_time = current_time;
  if (dt <= 0) dt = 1e-5; // guard against zero or negative dt if micros wraps or timer resolution issue

  updateStates();
  updateMotorVoltagesPID(dt);
  
  // Small delay to prevent overly fast loop if not much processing happens
  // This also sets a maximum loop frequency. Adjust as needed.
  // Example: for a ~400Hz loop: delayMicroseconds(2500 - (micros() - current_time));
  // For now, let it run as fast as possible, dt will handle PID scaling.
  // Consider if a fixed loop rate is desired for PID consistency.
} 