// Combined NovaDrone Flight Controller Code
// Base: transm.ino (Receiver Input, ESC Output)
// Sensors: sensors/sketch_apr21b.ino (MPU6050)
// Added: Basic PID Structure for Stabilization

#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

// --- Pin Definitions ---
// !! IMPORTANT: Verify these pins match your ESP32 wiring !!
// Receiver Input Pins (Using standard PWM)
#define CH1_PIN 13 // Roll
#define CH2_PIN 12 // Pitch
#define CH3_PIN 14 // Throttle
#define CH4_PIN 27 // Yaw

// Motor Output Pins
#define MOTOR1_PIN 18 // Example: Front Right (CW rotation assumed)
#define MOTOR2_PIN 19 // Example: Rear Right (CCW rotation assumed)
#define MOTOR3_PIN 21 // Example: Rear Left (CW rotation assumed)
#define MOTOR4_PIN 22 // Example: Front Left (CCW rotation assumed)

// MPU6050 I2C Pins (Default for many ESP32 boards)
// SDA: 21, SCL: 22 - NOTE: These might conflict with MOTOR3/4_PIN above!
// If using default Wire pins, ensure MOTOR pins are different.
// If using non-default I2C pins, initialize Wire like: Wire.begin(SDA_PIN, SCL_PIN);

// --- Constants ---
#define MIN_PWM 1000       // Minimum PWM signal (microseconds) for ESCs/Receiver
#define MAX_PWM 2000       // Maximum PWM signal (microseconds) for ESCs/Receiver
#define MID_PWM 1500       // Midpoint PWM signal (microseconds)
#define ARM_THRESHOLD 1050 // Throttle value below which motors can be armed
#define MAX_CONTROL_ANGLE 25.0 // Max tilt angle (degrees) controllable by sticks
#define MAX_YAW_RATE 100.0     // Max yaw rate (degrees/sec) controllable by sticks

// --- PID Constants ---
// !! IMPORTANT: These need careful tuning! Start low. !!
// Roll Axis
float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 18.0;
// Pitch Axis (Often similar to Roll)
float pid_p_gain_pitch = 1.3; // Same as roll
float pid_i_gain_pitch = 0.04;// Same as roll
float pid_d_gain_pitch = 18.0;// Same as roll
// Yaw Axis
float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;

// Maximum PID output values (constrain the correction)
float pid_max_roll = 400;
float pid_max_pitch = 400;
float pid_max_yaw = 400;

// --- Global Variables ---
// ESC Objects
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// MPU6050 Sensor Object
MPU6050 mpu(Wire);

// Raw receiver values (microseconds)
unsigned long ch1_value, ch2_value, ch3_value, ch4_value;
unsigned long loop_timer; // To measure loop time for PID calculations

// Mapped control inputs (desired states)
int throttle;
float desired_angle_roll, desired_angle_pitch, desired_rate_yaw;

// Calculated motor speeds (microseconds)
int motor1_speed, motor2_speed, motor3_speed, motor4_speed;

// Arming status
bool armed = false;

// Sensor Data
float angle_roll, angle_pitch, angle_yaw; // Current angles from MPU
float rate_roll, rate_pitch, rate_yaw;     // Current rates from MPU (might be needed for D term)

// PID Variables
float pid_error_temp;
float pid_i_mem_roll, pid_i_mem_pitch, pid_i_mem_yaw;
float pid_last_roll_d_error, pid_last_pitch_d_error, pid_last_yaw_d_error;
float pid_output_roll, pid_output_pitch, pid_output_yaw;

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    Serial.println("NovaDrone Combined Controller Initializing...");
    Serial.println("Ensure propellers are removed for safety!");
    Serial.println("Ensure ESCs are calibrated (1000-2000us).");
    Serial.println("Verifying MPU6050 connection...");

    // Initialize I2C for MPU6050
    Wire.begin(); // Use default SDA/SCL pins unless specified: Wire.begin(SDA_PIN, SCL_PIN);
    byte status = mpu.begin();
    if (status != 0) {
        Serial.println("MPU6050 connection failed! Check wiring.");
        while (1) { delay(100); } // Halt execution
    }
    Serial.println("MPU6050 connected. Calibrating...");
    delay(1000);
    mpu.calcOffsets(true, true); // Calculate gyro and accelerometer offsets
    Serial.println("MPU6050 calibration complete.");

    // Set receiver pin modes
    pinMode(CH1_PIN, INPUT); // Use INPUT_PULLUP if your receiver requires it
    pinMode(CH2_PIN, INPUT);
    pinMode(CH3_PIN, INPUT);
    pinMode(CH4_PIN, INPUT);

    // Allow allocation of all timers for ESP32Servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Attach ESCs to pins, setting the pulse width range.
    esc1.attach(MOTOR1_PIN, MIN_PWM, MAX_PWM);
    esc2.attach(MOTOR2_PIN, MIN_PWM, MAX_PWM);
    esc3.attach(MOTOR3_PIN, MIN_PWM, MAX_PWM);
    esc4.attach(MOTOR4_PIN, MIN_PWM, MAX_PWM);

    // Arming sequence part 1: Send min throttle to ESCs
    Serial.println("Sending minimum throttle signal to ESCs...");
    esc1.writeMicroseconds(MIN_PWM);
    esc2.writeMicroseconds(MIN_PWM);
    esc3.writeMicroseconds(MIN_PWM);
    esc4.writeMicroseconds(MIN_PWM);
    delay(3000); // Give ESCs time to initialize (adjust if needed)

    Serial.println("Setup complete. Lower throttle stick below ARM_THRESHOLD to arm.");
    loop_timer = micros(); // Initialize loop timer
}

// --- Loop Function ---
void loop() {
    // --- Read Receiver ---
    // Read receiver channels using pulseIn (blocking)
    // Timeout set to 25000 microseconds (25ms) - ensures we don't hang if signal lost
    ch1_value = pulseIn(CH1_PIN, HIGH, 25000); // Roll
    ch2_value = pulseIn(CH2_PIN, HIGH, 25000); // Pitch
    ch3_value = pulseIn(CH3_PIN, HIGH, 25000); // Throttle
    ch4_value = pulseIn(CH4_PIN, HIGH, 25000); // Yaw

    // --- Failsafe Check ---
    if (ch1_value == 0 || ch2_value == 0 || ch3_value == 0 || ch4_value == 0) {
        // Signal lost or invalid - Stop motors immediately
        if (armed) { // Only print once on signal loss
             Serial.println("Receiver signal lost! DISARMING.");
        }
        motor1_speed = MIN_PWM;
        motor2_speed = MIN_PWM;
        motor3_speed = MIN_PWM;
        motor4_speed = MIN_PWM;
        armed = false; // Disarm on signal loss
        reset_pid();   // Reset PID integrals
    } else {
        // --- Map Receiver Inputs to Desired States ---
        throttle = map(ch3_value, MIN_PWM, MAX_PWM, MIN_PWM, MAX_PWM); // 1000-2000us
        // Map Roll/Pitch sticks to desired angles (+/- MAX_CONTROL_ANGLE degrees)
        desired_angle_roll = map(ch1_value, MIN_PWM, MAX_PWM, -MAX_CONTROL_ANGLE, MAX_CONTROL_ANGLE);
        desired_angle_pitch = map(ch2_value, MIN_PWM, MAX_PWM, -MAX_CONTROL_ANGLE, MAX_CONTROL_ANGLE);
        // Map Yaw stick to desired rate (+/- MAX_YAW_RATE degrees/sec)
        desired_rate_yaw = map(ch4_value, MIN_PWM, MAX_PWM, -MAX_YAW_RATE, MAX_YAW_RATE);

        // --- Read Sensor Data ---
        mpu.update();
        angle_roll = mpu.getAngleX();
        angle_pitch = mpu.getAngleY();
        // Using gyro rate directly for yaw control is common
        rate_yaw = mpu.getGyroZ();
        // Get roll/pitch rates if needed for D-term calculation based on rate error
        rate_roll = mpu.getGyroX();
        rate_pitch = mpu.getGyroY();


        // --- Arming Logic ---
        if (!armed) {
            if (throttle < ARM_THRESHOLD) {
                // Only arm if throttle is low
                armed = true;
                Serial.println("System Armed!");
                reset_pid(); // Reset PID when arming
            } else {
                // Keep motors at minimum while waiting to arm
                motor1_speed = MIN_PWM;
                motor2_speed = MIN_PWM;
                motor3_speed = MIN_PWM;
                motor4_speed = MIN_PWM;
            }
        }

        // --- Flight Control (Only if Armed) ---
        if (armed) {
            // Disarm if throttle is brought low again
            if (throttle < ARM_THRESHOLD) {
                armed = false;
                motor1_speed = MIN_PWM;
                motor2_speed = MIN_PWM;
                motor3_speed = MIN_PWM;
                motor4_speed = MIN_PWM;
                Serial.println("Disarmed (throttle low).");
                reset_pid();
            } else {
                // --- Calculate PID Corrections ---
                calculate_pid();

                // --- Mix Motor Speeds ---
                // Basic Quadcopter Mixing ('X' configuration)
                // Incorporate PID outputs to stabilize
                // Adjust signs (+/-) based on motor rotation and placement relative to PID output meaning
                // Assumed: +pid_roll corrects right roll, +pid_pitch corrects forward pitch, +pid_yaw corrects right yaw

                // M1: Front Right (CW) - Needs +pitch, -roll, -yaw
                // M2: Rear Right  (CCW) - Needs -pitch, -roll, +yaw
                // M3: Rear Left   (CW)  - Needs -pitch, +roll, -yaw
                // M4: Front Left  (CCW) - Needs +pitch, +roll, +yaw

                motor1_speed = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
                motor2_speed = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;
                motor3_speed = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
                motor4_speed = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;

                // --- Constrain Motor Speeds ---
                // Ensure speeds are within the valid ESC range (MIN_PWM to MAX_PWM)
                // Also ensure motors don't stop completely if throttle is high enough
                int min_throttle_speed = MIN_PWM + 50; // Small idle speed above min PWM when armed & throttle up
                if (throttle > ARM_THRESHOLD + 50) { // Only apply min throttle speed if stick is up a bit
                   motor1_speed = constrain(motor1_speed, min_throttle_speed, MAX_PWM);
                   motor2_speed = constrain(motor2_speed, min_throttle_speed, MAX_PWM);
                   motor3_speed = constrain(motor3_speed, min_throttle_speed, MAX_PWM);
                   motor4_speed = constrain(motor4_speed, min_throttle_speed, MAX_PWM);
                } else { // If throttle is very low but still armed, allow motors to go to MIN_PWM
                   motor1_speed = constrain(motor1_speed, MIN_PWM, MAX_PWM);
                   motor2_speed = constrain(motor2_speed, MIN_PWM, MAX_PWM);
                   motor3_speed = constrain(motor3_speed, MIN_PWM, MAX_PWM);
                   motor4_speed = constrain(motor4_speed, MIN_PWM, MAX_PWM);
                }
            }
        } // end if(armed)
    } // end if(signal ok)

    // --- Write to Motors ---
    // Write calculated speeds to ESCs only if armed OR if signal was lost (to ensure motors stop)
    if (armed || (ch1_value == 0 || ch2_value == 0 || ch3_value == 0 || ch4_value == 0)) {
        esc1.writeMicroseconds(motor1_speed);
        esc2.writeMicroseconds(motor2_speed);
        esc3.writeMicroseconds(motor3_speed);
        esc4.writeMicroseconds(motor4_speed);
    } else {
        // If not armed and signal is present, ensure motors stay at MIN_PWM
        // (Handles the case where it's waiting for arming)
         esc1.writeMicroseconds(MIN_PWM);
         esc2.writeMicroseconds(MIN_PWM);
         esc3.writeMicroseconds(MIN_PWM);
         esc4.writeMicroseconds(MIN_PWM);
    }


    // --- Loop Timing ---
    // Wait until the desired loop time has passed (e.g., 4000us for 250Hz)
    // This helps keep PID calculations consistent.
    while(micros() - loop_timer < 4000);
    loop_timer = micros(); // Reset timer for next loop

    // Optional: Print values for debugging (uncomment/modify as needed)
    /*
    static unsigned long last_print_time = 0;
    if (millis() - last_print_time > 100) { // Print every 100ms
        Serial.print("Armed: "); Serial.print(armed);
        Serial.print(" | Thr:"); Serial.print(throttle);
        // Serial.print(" | Roll D:"); Serial.print(desired_angle_roll);
        // Serial.print(" P:"); Serial.print(desired_angle_pitch);
        // Serial.print(" Y:"); Serial.print(desired_rate_yaw);
        Serial.print(" | Roll A:"); Serial.print(angle_roll);
        Serial.print(" P:"); Serial.print(angle_pitch);
        // Serial.print(" Y:"); Serial.print(rate_yaw); // Yaw angle isn't directly used here
        Serial.print(" | PID R:"); Serial.print(pid_output_roll);
        Serial.print(" P:"); Serial.print(pid_output_pitch);
        Serial.print(" Y:"); Serial.print(pid_output_yaw);
        Serial.print(" | M1:"); Serial.print(motor1_speed);
        Serial.print(" M2:"); Serial.print(motor2_speed);
        Serial.print(" M3:"); Serial.print(motor3_speed);
        Serial.print(" M4:"); Serial.println(motor4_speed);
        last_print_time = millis();
    }
    */
}

// --- PID Calculation Function ---
void calculate_pid() {
    // Calculate Roll PID Error (Desired Angle vs Actual Angle)
    pid_error_temp = angle_roll - desired_angle_roll;
    // Calculate I term
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    // Constrain I term to prevent windup
    pid_i_mem_roll = constrain(pid_i_mem_roll, -pid_max_roll, pid_max_roll);
    // Calculate D term (using change in error, or directly using gyro rate)
    // Option 1: D term based on change in angle error (more sensitive to noise)
    // pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    // pid_last_roll_d_error = pid_error_temp;
    // Option 2: D term based on gyro rate (often preferred) - assumes D gain is tuned for rate
    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * rate_roll; // Use rate_roll directly

    // Calculate Pitch PID Error
    pid_error_temp = angle_pitch - desired_angle_pitch;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    pid_i_mem_pitch = constrain(pid_i_mem_pitch, -pid_max_pitch, pid_max_pitch);
    // pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    // pid_last_pitch_d_error = pid_error_temp;
    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * rate_pitch; // Use rate_pitch directly

    // Calculate Yaw PID Error (Desired Rate vs Actual Rate)
    pid_error_temp = rate_yaw - desired_rate_yaw; // Error is difference in rotation rates
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    pid_i_mem_yaw = constrain(pid_i_mem_yaw, -pid_max_yaw, pid_max_yaw);
    // D term for yaw is often zero or based on change in rate error
    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    pid_last_yaw_d_error = pid_error_temp;
    // pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw; // Simpler Yaw PID if D=0

    // Constrain total PID output
    pid_output_roll = constrain(pid_output_roll, -pid_max_roll, pid_max_roll);
    pid_output_pitch = constrain(pid_output_pitch, -pid_max_pitch, pid_max_pitch);
    pid_output_yaw = constrain(pid_output_yaw, -pid_max_yaw, pid_max_yaw);
}

// --- Reset PID Variables ---
void reset_pid() {
    pid_i_mem_roll = 0;
    pid_i_mem_pitch = 0;
    pid_i_mem_yaw = 0;
    pid_last_roll_d_error = 0;
    pid_last_pitch_d_error = 0;
    pid_last_yaw_d_error = 0;
    pid_output_roll = 0;
    pid_output_pitch = 0;
    pid_output_yaw = 0;
}
