#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

// WiFi credentials
const char* ssid = "SIMOMIFI_G0U0_2.4GHz";
const char* password = "DwAJi54E";
// MySecureOTApass

// === ANGLE PID GAINS (Outer Loop) ===
double PAngleRoll = -2.0;   // NEGATIVE for inverted roll
double IAngleRoll = 0.0;    
double DAngleRoll = 0.0;    
double PAnglePitch = 2.0;
double IAnglePitch = 0.0;
double DAnglePitch = 0.0;

// === RATE PID GAINS (Inner Loop) ===
// Optimal values from PID_TUNING_RESULTS_SUMMARY.md simulation
double Kp_pitch = 0.025;
double Ki_pitch = 0.0004;   // Updated from simulation optimal
double Kd_pitch = 0.0;      // Derivative not needed
double Kp_roll = -0.025;    // NEGATIVE for inverted roll
double Ki_roll = -0.0004;   // NEGATIVE for inverted roll
double Kd_roll = 0.0;       // Derivative not needed
double Kp_yaw = 0.015;   
double Ki_yaw = 0.0;        // P-only control sufficient
double Kd_yaw = 0.0;        // Derivative not needed

// PID Output Scaling Factors
const double PID_OUTPUT_SCALING_PITCH = 1.0;
const double PID_OUTPUT_SCALING_ROLL = 1.0;
const double PID_OUTPUT_SCALING_YAW = 1.0;

// Safety Limits - INCREASED
const double MAX_PID_INTEGRAL = 100.0;        // Increased from 50.0
const double MAX_TARGET_PITCH_ROLL = 30.0;    // Increased from 20.0
const double MAX_ABS_PITCH_ROLL_SAFETY = 45.0; // Increased from 35.0

// PID State Variables
double angle_pitch_error_integral = 0.0;
double angle_pitch_previous_error = 0.0;
double angle_roll_error_integral = 0.0;
double angle_roll_previous_error = 0.0;

double pitch_error_integral = 0.0;
double pitch_previous_error = 0.0;
double roll_error_integral = 0.0;
double roll_previous_error = 0.0;
double yaw_error_integral = 0.0;
double yaw_previous_error = 0.0;

unsigned long last_pid_time = 0;

// Original global variables
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Motor pin assignments (updated after testing)
const int ppm_pin = 8;
const int b_cw_pin = 9;   // Swapped: was f_cw
const int f_cw_pin = 10;  // Swapped: was b_cw
const int f_ccw_pin = 11; 
const int b_ccw_pin = 12;

unsigned long timer = 0;
unsigned long timer_pid_debug = 0;

volatile unsigned long lastTime = 0;
volatile int channelIndex = 0;
volatile int channelValues[6] = {0};

int cout_roll = 1500;
int cout_pitch = 1500;
int cout_throttle = 1000;
int cout_yaw = 1500;
int cout_vra = 1000;
int cout_vrb = 1000;

double current_yaw_target = 0.0;

double imu_roll = 0.0;
double imu_pitch = 0.0;
double imu_yaw = 0.0;

double v_scalar = 0.0;

double f_cw_scalar = 0.5;
double b_cw_scalar = 0.5;
double f_ccw_scalar = 0.5;
double b_ccw_scalar = 0.5;

const double scalar_change = 0.035;

double pitch_diff = 0.0;
double roll_diff = 0.0;
double yaw_diff = 0.0;

double x_offset = 0.0;
double y_offset = 0.0;
double z_offset = 0.0;

Servo esc[4];

WiFiServer wifiServer(23);
WiFiClient telnetClient;

double DesiredAngleRoll = 0.0;
double DesiredAnglePitch = 0.0;
double DesiredRateRoll = 0.0;
double DesiredRatePitch = 0.0;
double DesiredRateYaw = 0.0;

double RateRoll = 0.0;
double RatePitch = 0.0;
double RateYaw = 0.0;

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
  Serial.begin(115200); 

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
  printlnOutput(WiFi.localIP().toString());
  
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
        telnetClient.read();
      }
    }
  }
}

double normalizeAngle180(double angle) {
  angle = fmod(angle, 360.0);
  if (angle > 180.0) {
    angle -= 360.0;
  } else if (angle <= -180.0) {
    angle += 360.0;
  }
  return angle;
}

double calculatePID(double current_value, double target_value,
                    double Kp, double Ki, double Kd,
                    double &integral_term, double &previous_error, double dt) {
  double error = target_value - current_value;

  double P_out = Kp * error;

  integral_term += error * dt;
  integral_term = constrain(integral_term, -MAX_PID_INTEGRAL, MAX_PID_INTEGRAL);
  double I_out = Ki * integral_term;

  double derivative = (error - previous_error) / dt;
  double D_out = Kd * derivative;

  previous_error = error;

  return P_out + I_out + D_out;
}

double PIDReturn[] = {0, 0, 0};

void pid_equation(double Error, double P, double I, double D, double &PrevError, double &PrevIterm) {
  double dt = 0.004;
  
  double Pterm = P * Error;
  double Iterm = PrevIterm + I * (Error + PrevError) * dt / 2;
  
  if (Iterm > 600) Iterm = 600;      // Increased from 400
  else if (Iterm < -600) Iterm = -600; // Increased from 400
  
  double Dterm = D * (Error - PrevError) / dt;
  double PIDOutput = Pterm + Iterm + Dterm;
  
  if (PIDOutput > 600) PIDOutput = 600;      // Increased from 400
  else if (PIDOutput < -600) PIDOutput = -600; // Increased from 400
  
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
  
  PrevError = Error;
  PrevIterm = Iterm;
}

void reset_pid(void) {
  angle_pitch_previous_error = 0; angle_pitch_error_integral = 0;
  angle_roll_previous_error = 0; angle_roll_error_integral = 0;
  
  pitch_previous_error = 0; pitch_error_integral = 0;
  roll_previous_error = 0; roll_error_integral = 0;
  yaw_previous_error = 0; yaw_error_integral = 0;
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

  delay(1000);
  bno.setExtCrystalUse(true);
  printlnOutput("BNO055 External Crystal Enabled.");

  sensors_event_t event;
  for (int i=0; i<10; i++) {
      bno.getEvent(&event);
      delay(50);
  }
  x_offset = event.orientation.x;
  y_offset = event.orientation.y;
  z_offset = event.orientation.z;
  current_yaw_target = 0;
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
      printlnOutput("RX still disconnected after checks.");
  } else {
      printlnOutput("RX Detected! Proceeding...");
  }
  delay(100);

  printlnOutput("Arming ESCs...");
  esc[0].setPeriodHertz(50); esc[0].attach(b_cw_pin, 1000, 2000); esc[0].writeMicroseconds(1000);
  esc[1].setPeriodHertz(50); esc[1].attach(f_cw_pin, 1000, 2000); esc[1].writeMicroseconds(1000);
  esc[2].setPeriodHertz(50); esc[2].attach(f_ccw_pin, 1000, 2000); esc[2].writeMicroseconds(1000);
  esc[3].setPeriodHertz(50); esc[3].attach(b_ccw_pin, 1000, 2000); esc[3].writeMicroseconds(1000);
  delay(1000);

  pinMode(ppm_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ppm_pin), ppmISR, RISING); 

  last_pid_time = micros();
  delay(1000);
  printlnOutput("Ready for flight!");
}

void updateStates() {
  sensors_event_t event;
  bno.getEvent(&event); 
  
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  RateRoll = gyro.y();
  RatePitch = gyro.x();
  RateYaw = gyro.z();
  
  cout_roll = channelValues[0]; 
  cout_pitch = channelValues[1]; 
  cout_throttle = channelValues[2]; 
  cout_yaw = channelValues[3]; 
  cout_vra = channelValues[4]; 
  cout_vrb = channelValues[5]; 
  
  imu_yaw = normalizeAngle180(event.orientation.x);
  imu_pitch = event.orientation.y - y_offset;
  imu_roll = event.orientation.z - z_offset;

  if (millis() - timer > 100) {
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
    DesiredAnglePitch = 0.10 * (controller_pitch_input - 1500);
    DesiredAnglePitch = constrain(DesiredAnglePitch, -MAX_TARGET_PITCH_ROLL, MAX_TARGET_PITCH_ROLL);
    
    double ErrorAnglePitch = DesiredAnglePitch - current_pitch_angle;
    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, angle_pitch_previous_error, angle_pitch_error_integral);
    DesiredRatePitch = PIDReturn[0];
    
    double ErrorRatePitch = DesiredRatePitch - RatePitch;
    pid_equation(ErrorRatePitch, Kp_pitch, Ki_pitch, Kd_pitch, pitch_previous_error, pitch_error_integral);
    double InputPitch = PIDReturn[0];
    
    double scaled_pid_output = InputPitch / 1200.0;
    pitch_diff = scaled_pid_output;

    contributions[0] = 0.5 + scaled_pid_output;
    contributions[1] = 0.5 - scaled_pid_output;
    contributions[2] = 0.5 + scaled_pid_output;
    contributions[3] = 0.5 - scaled_pid_output;
}

// Calculates roll motor contributions based on PID
void adjustRollPID(double current_roll_angle, int controller_roll_input, double dt, double* contributions) {
    // 1. Calculate Desired Angle (outer loop setpoint)
    DesiredAngleRoll = 0.10 * (controller_roll_input - 1500);  // ±50° at full stick
    DesiredAngleRoll = constrain(DesiredAngleRoll, -MAX_TARGET_PITCH_ROLL, MAX_TARGET_PITCH_ROLL);
    
    // 2. Angle PID (outer loop) - calculates desired rate
    double ErrorAngleRoll = DesiredAngleRoll - current_roll_angle;
    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, angle_roll_previous_error, angle_roll_error_integral);
    DesiredRateRoll = PIDReturn[0];
    
    // 3. Rate PID (inner loop) - calculates motor command
    double ErrorRateRoll = DesiredRateRoll - RateRoll;
    pid_equation(ErrorRateRoll, Kp_roll, Ki_roll, Kd_roll, roll_previous_error, roll_error_integral);
    double InputRoll = PIDReturn[0];
    
    // Scale to 0-1 range for motor mixing
    double scaled_pid_output = InputRoll / 1200.0; // Scale from ±600 to ±0.5
    roll_diff = scaled_pid_output; // For telnet output

    // 4. Calculate motor contributions for roll
    contributions[0] = 0.5 + scaled_pid_output; // f_cw (front left)
    contributions[1] = 0.5 + scaled_pid_output; // b_cw (rear left)
    contributions[2] = 0.5 - scaled_pid_output; // f_ccw (front right)
    contributions[3] = 0.5 - scaled_pid_output; // b_ccw (rear right)
}

// Updates yaw target and calculates yaw motor contributions based on PID
void updateYawTargetAndContributionsPID(double current_yaw_angle, int controller_yaw_input, double dt, double* contributions) {
    // 1. Yaw is rate control only (no angle loop)
    DesiredRateYaw = 0.15 * (controller_yaw_input - 1500);  // ±225°/s at full stick
    
    // Update yaw target for telnet output (keep existing logic)
    double adjusted_yaw_stick_input = (1500.0 - controller_yaw_input) / 1000.0;
    current_yaw_target -= (scalar_change * adjusted_yaw_stick_input);
    current_yaw_target = normalizeAngle180(current_yaw_target);
    
    // 2. Rate PID for yaw
    double ErrorRateYaw = DesiredRateYaw - RateYaw;
    yaw_diff = ErrorRateYaw / 90.0; // For telnet output compatibility
    
    pid_equation(ErrorRateYaw, Kp_yaw, Ki_yaw, Kd_yaw, yaw_previous_error, yaw_error_integral);
    double InputYaw = PIDReturn[0];
    
    // Scale to 0-1 range
    double scaled_pid_output = InputYaw / 1200.0;

    // 3. Calculate motor contributions for yaw (keep existing mixing)
    contributions[0] = 0.5 - scaled_pid_output; // f_cw
    contributions[1] = 0.5 - scaled_pid_output; // b_cw
    contributions[2] = 0.5 + scaled_pid_output; // f_ccw
    contributions[3] = 0.5 + scaled_pid_output; // b_ccw
}


void updateMotorVoltagesPID(double dt) {
  // Motor power constants - tested values for straight flight
  // These compensate for individual motor/prop differences
  double f_cw_const = 0.96725;  // Front clockwise
  double b_cw_const = 0.9825;   // Back clockwise  
  double f_ccw_const = 0.9575;  // Front counter-clockwise
  double b_ccw_const = 0.95;    // Back counter-clockwise (weakest motor)
  
  double pitch_contributions[4];
  double roll_contributions[4];
  double yaw_contributions[4];

  adjustPitchPID(imu_pitch, cout_pitch, dt, pitch_contributions);
  adjustRollPID(imu_roll, cout_roll, dt, roll_contributions);
  updateYawTargetAndContributionsPID(imu_yaw, cout_yaw, dt, yaw_contributions);

  f_cw_scalar = (pitch_contributions[0] + roll_contributions[0] + yaw_contributions[0]) / 3.0;
  b_cw_scalar = (pitch_contributions[1] + roll_contributions[1] + yaw_contributions[1]) / 3.0;
  f_ccw_scalar = (pitch_contributions[2] + roll_contributions[2] + yaw_contributions[2]) / 3.0;
  b_ccw_scalar = (pitch_contributions[3] + roll_contributions[3] + yaw_contributions[3]) / 3.0;

  v_scalar = (cout_throttle - 1000.0) / 1000.0;
  v_scalar = constrain(v_scalar, 0.0, 1.0);

  if (cout_vra > 1050 || cout_vrb > 1050 || cout_throttle < 950) {
    v_scalar = 0.0;
    reset_pid();
  }

  if (abs(imu_pitch) > MAX_ABS_PITCH_ROLL_SAFETY || abs(imu_roll) > MAX_ABS_PITCH_ROLL_SAFETY) {
    v_scalar = 0.0;
    reset_pid();
    if(millis() - timer_pid_debug > 1000) {
        printlnOutput("!!! SAFETY CUTOFF: Pitch/Roll angle exceeded limits !!!");
    }
  }
  
  double max_s = 0.0;
  max_s = max(f_cw_scalar, max_s);
  max_s = max(b_cw_scalar, max_s);
  max_s = max(f_ccw_scalar, max_s);
  max_s = max(b_ccw_scalar, max_s);

  if (max_s > 1.0) {
    f_cw_scalar /= max_s;
    b_cw_scalar /= max_s;
    f_ccw_scalar /= max_s;
    b_ccw_scalar /= max_s;
  }
  
  f_cw_scalar = max(0.0, f_cw_scalar);
  b_cw_scalar = max(0.0, b_cw_scalar);
  f_ccw_scalar = max(0.0, f_ccw_scalar);
  b_ccw_scalar = max(0.0, b_ccw_scalar);

  // Calculate base voltages
  double f_cw_volt = 1000.0 + (f_cw_scalar * v_scalar * 1000.0);
  double b_cw_volt = 1000.0 + (b_cw_scalar * v_scalar * 1000.0);
  double f_ccw_volt = 1000.0 + (f_ccw_scalar * v_scalar * 1000.0);
  double b_ccw_volt = 1000.0 + (b_ccw_scalar * v_scalar * 1000.0);

  // Apply motor constants to final voltages (after PID and mixing)
  f_cw_volt = 1000.0 + ((f_cw_volt - 1000.0) * f_cw_const);
  b_cw_volt = 1000.0 + ((b_cw_volt - 1000.0) * b_cw_const);
  f_ccw_volt = 1000.0 + ((f_ccw_volt - 1000.0) * f_ccw_const);
  b_ccw_volt = 1000.0 + ((b_ccw_volt - 1000.0) * b_ccw_const);

  // Final constraining
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

  updateStates();
  
  unsigned long current_time = micros();
  double dt = (current_time - last_pid_time) / 1000000.0;
  
  updateMotorVoltagesPID(dt);
  
  while (micros() - last_pid_time < 4000);
  last_pid_time = micros();
} 