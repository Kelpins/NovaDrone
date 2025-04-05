// Drone Control System with Physical Controller for ESP32 Nano

// Motor pin definitions for ESP32
#define MOTOR_FRONT_LEFT 5   // Changed to ESP32 PWM capable pins
#define MOTOR_FRONT_RIGHT 18
#define MOTOR_BACK_LEFT 19
#define MOTOR_BACK_RIGHT 21

// PWM properties for ESP32
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_FL 0     // PWM channels for each motor
#define PWM_CHANNEL_FR 1
#define PWM_CHANNEL_BL 2
#define PWM_CHANNEL_BR 3

// Controller input pins - ESP32 analog pins
#define THROTTLE_PIN 36      // VP (ADC1_CH0)
#define ROLL_PIN 39          // VN (ADC1_CH3)
#define PITCH_PIN 34         // ADC1_CH6
#define YAW_PIN 35           // ADC1_CH7 
#define TAKEOFF_BUTTON 23    // Digital pin for takeoff button
#define LAND_BUTTON 22       // Digital pin for land button

// Default motor speed values
#define DEFAULT_SPEED 150
#define MIN_SPEED 0
#define MAX_SPEED 255
#define CONTROL_DEADZONE 10  // Ignore small joystick movements

// Control variables
int baseSpeed = DEFAULT_SPEED;
bool isFlying = false;
int throttle, roll, pitch, yaw;
bool takeoffButtonPressed = false;
bool landButtonPressed = false;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);  // Higher baud rate for ESP32
  
  // Initialize PWM for motor control
  ledcSetup(PWM_CHANNEL_FL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_FR, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BR, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Attach pins to PWM channels
  ledcAttachPin(MOTOR_FRONT_LEFT, PWM_CHANNEL_FL);
  ledcAttachPin(MOTOR_FRONT_RIGHT, PWM_CHANNEL_FR);
  ledcAttachPin(MOTOR_BACK_LEFT, PWM_CHANNEL_BL);
  ledcAttachPin(MOTOR_BACK_RIGHT, PWM_CHANNEL_BR);
  
  // Initialize controller input pins
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(ROLL_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);
  pinMode(YAW_PIN, INPUT);
  pinMode(TAKEOFF_BUTTON, INPUT_PULLUP);
  pinMode(LAND_BUTTON, INPUT_PULLUP);
  
  // Set all motors to off initially
  stopAllMotors();
  
  Serial.println("ESP32 Nano drone control system initialized with physical controller");
}

void loop() {
  // Read controller inputs
  readControllerInputs();
  
  // Handle takeoff/land buttons
  handleFlightStateButtons();
  
  // If flying, control the drone's movement
  if (isFlying) {
    controlDrone();
  }
  
  // Small delay for stability
  delay(20);
}

void readControllerInputs() {
  // Read analog values from controller (0-4095 for ESP32 ADC)
  throttle = analogRead(THROTTLE_PIN);
  roll = analogRead(ROLL_PIN);
  pitch = analogRead(PITCH_PIN);
  yaw = analogRead(YAW_PIN);
  
  // Map analog values to appropriate ranges
  throttle = map(throttle, 0, 4095, MIN_SPEED, MAX_SPEED);
  
  // For directional controls: Map to -100 to +100 range
  roll = map(roll, 0, 4095, -100, 100);
  pitch = map(pitch, 0, 4095, -100, 100);
  yaw = map(yaw, 0, 4095, -100, 100);
  
  // Apply deadzone to control inputs
  roll = applyDeadzone(roll);
  pitch = applyDeadzone(pitch);
  yaw = applyDeadzone(yaw);
  
  // Read digital buttons (active LOW with pullup)
  takeoffButtonPressed = !digitalRead(TAKEOFF_BUTTON);
  landButtonPressed = !digitalRead(LAND_BUTTON);
}

int applyDeadzone(int value) {
  // Ignore small movements around center
  if (abs(value) < CONTROL_DEADZONE) {
    return 0;
  }
  return value;
}

void handleFlightStateButtons() {
  // Handle takeoff button
  if (takeoffButtonPressed && !isFlying) {
    takeOff();
  }
  
  // Handle land button
  if (landButtonPressed && isFlying) {
    land();
  }
}

void controlDrone() {
  // Calculate motor speeds based on controller inputs
  int frontLeftSpeed = throttle;
  int frontRightSpeed = throttle;
  int backLeftSpeed = throttle;
  int backRightSpeed = throttle;
  
  // Apply pitch control (forward/backward tilt)
  frontLeftSpeed -= pitch;
  frontRightSpeed -= pitch;
  backLeftSpeed += pitch;
  backRightSpeed += pitch;
  
  // Apply roll control (left/right tilt)
  frontLeftSpeed -= roll;
  frontRightSpeed += roll;
  backLeftSpeed -= roll;
  backRightSpeed += roll;
  
  // Apply yaw control (rotation)
  frontLeftSpeed -= yaw;
  frontRightSpeed += yaw;
  backLeftSpeed += yaw;
  backRightSpeed -= yaw;
  
  // Constrain values to valid range
  frontLeftSpeed = constrain(frontLeftSpeed, MIN_SPEED, MAX_SPEED);
  frontRightSpeed = constrain(frontRightSpeed, MIN_SPEED, MAX_SPEED);
  backLeftSpeed = constrain(backLeftSpeed, MIN_SPEED, MAX_SPEED);
  backRightSpeed = constrain(backRightSpeed, MIN_SPEED, MAX_SPEED);
  
  // Apply motor speeds using ESP32 PWM
  ledcWrite(PWM_CHANNEL_FL, frontLeftSpeed);
  ledcWrite(PWM_CHANNEL_FR, frontRightSpeed);
  ledcWrite(PWM_CHANNEL_BL, backLeftSpeed);
  ledcWrite(PWM_CHANNEL_BR, backRightSpeed);
  
  // Debug output - print only occasionally
  if (millis() % 1000 < 20) {
    Serial.print("Motors: ");
    Serial.print(frontLeftSpeed); Serial.print(", ");
    Serial.print(frontRightSpeed); Serial.print(", ");
    Serial.print(backLeftSpeed); Serial.print(", ");
    Serial.println(backRightSpeed);
  }
}

void takeOff() {
  Serial.println("Taking off...");
  // Gradually increase all motors to base speed
  for (int i = MIN_SPEED; i <= baseSpeed; i += 5) {
    setAllMotors(i);
    delay(100);
  }
  isFlying = true;
  Serial.println("Hovering");
}

void land() {
  Serial.println("Landing...");
  // Gradually decrease all motors to zero
  for (int i = baseSpeed; i >= MIN_SPEED; i -= 5) {
    setAllMotors(i);
    delay(100);
  }
  stopAllMotors();
  isFlying = false;
  Serial.println("Landed");
}

void setAllMotors(int speed) {
  ledcWrite(PWM_CHANNEL_FL, speed);
  ledcWrite(PWM_CHANNEL_FR, speed);
  ledcWrite(PWM_CHANNEL_BL, speed);
  ledcWrite(PWM_CHANNEL_BR, speed);
}

void stopAllMotors() {
  setAllMotors(MIN_SPEED);
}
