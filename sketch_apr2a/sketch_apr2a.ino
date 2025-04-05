// Drone Control System with Physical Controller

// Motor pin definitions
#define MOTOR_FRONT_LEFT 9
#define MOTOR_FRONT_RIGHT 10
#define MOTOR_BACK_LEFT 11
#define MOTOR_BACK_RIGHT 12

// Controller input pins
#define THROTTLE_PIN A0    // Up/Down control (analog)
#define ROLL_PIN A1        // Left/Right control (analog)
#define PITCH_PIN A2       // Forward/Backward control (analog)
#define YAW_PIN A3         // Rotation control (analog)
#define TAKEOFF_BUTTON 2   // Digital pin for takeoff button
#define LAND_BUTTON 3      // Digital pin for land button

// Default motor speed values
#define DEFAULT_SPEED 150
#define MIN_SPEED 0
#define MAX_SPEED 255
#define CONTROL_DEADZONE 10 // Ignore small joystick movements

// Control variables
int baseSpeed = DEFAULT_SPEED;
bool isFlying = false;
int throttle, roll, pitch, yaw;
bool takeoffButtonPressed = false;
bool landButtonPressed = false;

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  
  // Initialize motor control pins
  pinMode(MOTOR_FRONT_LEFT, OUTPUT);
  pinMode(MOTOR_FRONT_RIGHT, OUTPUT);
  pinMode(MOTOR_BACK_LEFT, OUTPUT);
  pinMode(MOTOR_BACK_RIGHT, OUTPUT);
  
  // Initialize controller input pins
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(ROLL_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);
  pinMode(YAW_PIN, INPUT);
  pinMode(TAKEOFF_BUTTON, INPUT_PULLUP);
  pinMode(LAND_BUTTON, INPUT_PULLUP);
  
  // Set all motors to off initially
  stopAllMotors();
  
  Serial.println("Drone control system initialized with physical controller");
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
  // Read analog values from controller (0-1023)
  throttle = analogRead(THROTTLE_PIN);
  roll = analogRead(ROLL_PIN);
  pitch = analogRead(PITCH_PIN);
  yaw = analogRead(YAW_PIN);
  
  // Map analog values to appropriate ranges
  throttle = map(throttle, 0, 1023, MIN_SPEED, MAX_SPEED);
  
  // For directional controls: Map to -100 to +100 range
  roll = map(roll, 0, 1023, -100, 100);
  pitch = map(pitch, 0, 1023, -100, 100);
  yaw = map(yaw, 0, 1023, -100, 100);
  
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
  
  // Apply motor speeds
  analogWrite(MOTOR_FRONT_LEFT, frontLeftSpeed);
  analogWrite(MOTOR_FRONT_RIGHT, frontRightSpeed);
  analogWrite(MOTOR_BACK_LEFT, backLeftSpeed);
  analogWrite(MOTOR_BACK_RIGHT, backRightSpeed);
  
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
  analogWrite(MOTOR_FRONT_LEFT, speed);
  analogWrite(MOTOR_FRONT_RIGHT, speed);
  analogWrite(MOTOR_BACK_LEFT, speed);
  analogWrite(MOTOR_BACK_RIGHT, speed);
}

void stopAllMotors() {
  setAllMotors(MIN_SPEED);
}
