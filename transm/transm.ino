#include <ESP32Servo.h>

// --- Pin Definitions ---
// !! IMPORTANT: Change these pins to match your ESP32 wiring !!
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

// --- Constants ---
#define MIN_PWM 1000       // Minimum PWM signal (microseconds) for ESCs/Receiver
#define MAX_PWM 2000       // Maximum PWM signal (microseconds) for ESCs/Receiver
#define MID_PWM 1500       // Midpoint PWM signal (microseconds)
#define ARM_THRESHOLD 1050 // Throttle value below which motors can be armed

// --- Global Variables ---
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// Raw receiver values (microseconds)
unsigned long ch1_value, ch2_value, ch3_value, ch4_value;

// Mapped control inputs
int throttle, roll, pitch, yaw;

// Calculated motor speeds (microseconds)
int motor1_speed, motor2_speed, motor3_speed, motor4_speed;

bool armed = false; // Arming status flag

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    Serial.println("Quadcopter Basic Controller Initializing...");
    Serial.println("Ensure propellers are removed for safety!");
    Serial.println("Ensure ESCs are calibrated (1000-2000us).");

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
    // We'll use writeMicroseconds() for precise control.
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
    delay(2000); // Give ESCs time to initialize (adjust if needed)

    Serial.println("Setup complete. Lower throttle stick to arm.");
}

// --- Loop Function ---
void loop() {
    // Read receiver channels using pulseIn (blocking)
    // Timeout set to 25000 microseconds (25ms)
    ch1_value = pulseIn(CH1_PIN, HIGH, 25000); // Roll
    ch2_value = pulseIn(CH2_PIN, HIGH, 25000); // Pitch
    ch3_value = pulseIn(CH3_PIN, HIGH, 25000); // Throttle
    ch4_value = pulseIn(CH4_PIN, HIGH, 25000); // Yaw

    // Basic Failsafe: Check if any channel timed out (value is 0)
    if (ch1_value == 0 || ch2_value == 0 || ch3_value == 0 || ch4_value == 0) {
        // Signal lost or invalid - Stop motors immediately
        Serial.println("Receiver signal lost! Stopping motors.");
        motor1_speed = MIN_PWM;
        motor2_speed = MIN_PWM;
        motor3_speed = MIN_PWM;
        motor4_speed = MIN_PWM;
        armed = false; // Disarm on signal loss
    } else {
        // Map raw PWM values (1000-2000) to control ranges
        // Throttle is mapped directly to output range (1000-2000)
        throttle = map(ch3_value, MIN_PWM, MAX_PWM, MIN_PWM, MAX_PWM);
        // Roll, Pitch, Yaw mapped to a +/- range centered around 0
        roll     = map(ch1_value, MIN_PWM, MAX_PWM, -500, 500);
        pitch    = map(ch2_value, MIN_PWM, MAX_PWM, -500, 500);
        yaw      = map(ch4_value, MIN_PWM, MAX_PWM, -500, 500);

        // Arming Logic
        if (!armed) {
            if (throttle < ARM_THRESHOLD) {
                armed = true;
                Serial.println("System Armed!");
            } else {
                // Keep motors at minimum while waiting to arm
                motor1_speed = MIN_PWM;
                motor2_speed = MIN_PWM;
                motor3_speed = MIN_PWM;
                motor4_speed = MIN_PWM;
                // Optional: Print waiting message periodically
                // Serial.println("Waiting for arm (throttle low)...");
            }
        }

        // If armed, calculate motor speeds based on inputs
        if (armed) {
            // Check if throttle is low enough to disarm or idle
            if (throttle < ARM_THRESHOLD) {
                // Option 1: Disarm completely
                // armed = false;
                // motor1_speed = MIN_PWM;
                // motor2_speed = MIN_PWM;
                // motor3_speed = MIN_PWM;
                // motor4_speed = MIN_PWM;
                // Serial.println("Disarmed (throttle low).");

                // Option 2: Keep motors at minimum idle speed (or just stop)
                motor1_speed = MIN_PWM;
                motor2_speed = MIN_PWM;
                motor3_speed = MIN_PWM;
                motor4_speed = MIN_PWM;
            } else {
                // Calculate motor speeds using basic quadcopter mixing ('X' config)
                // Adjust signs (+/-) based on your specific motor rotation and placement
                // M1: Front Right (CW)
                // M2: Rear Right  (CCW)
                // M3: Rear Left   (CW)
                // M4: Front Left  (CCW)
                motor1_speed = throttle + pitch - roll - yaw;
                motor2_speed = throttle - pitch - roll + yaw;
                motor3_speed = throttle - pitch + roll - yaw;
                motor4_speed = throttle + pitch + roll + yaw;

                // Constrain motor speeds to the valid ESC PWM range
                motor1_speed = constrain(motor1_speed, MIN_PWM, MAX_PWM);
                motor2_speed = constrain(motor2_speed, MIN_PWM, MAX_PWM);
                motor3_speed = constrain(motor3_speed, MIN_PWM, MAX_PWM);
                motor4_speed = constrain(motor4_speed, MIN_PWM, MAX_PWM);
            }
        }
    }

    // Write calculated speeds to ESCs only if armed or if signal was lost (to ensure motors stop)
    if (armed || (ch1_value == 0 || ch2_value == 0 || ch3_value == 0 || ch4_value == 0)) {
        esc1.writeMicroseconds(motor1_speed);
        esc2.writeMicroseconds(motor2_speed);
        esc3.writeMicroseconds(motor3_speed);
        esc4.writeMicroseconds(motor4_speed);
    }

    // Optional: Print values for debugging (uncomment to use)
    /*
    Serial.print("Armed: "); Serial.print(armed);
    Serial.print(" | Thr:"); Serial.print(throttle);
    Serial.print(" | Roll:"); Serial.print(roll);
    Serial.print(" | Pitch:"); Serial.print(pitch);
    Serial.print(" | Yaw:"); Serial.print(yaw);
    Serial.print(" | M1:"); Serial.print(motor1_speed);
    Serial.print(" | M2:"); Serial.print(motor2_speed);
    Serial.print(" | M3:"); Serial.print(motor3_speed);
    Serial.print(" | M4:"); Serial.println(motor4_speed);
    */

    delay(20); // Loop delay - Adjust for desired control loop frequency (50Hz here)
}
