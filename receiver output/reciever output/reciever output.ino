// === PPM Reader for FS-iA6B on ESP32 Nano ===
// Reads PPM signal on D9 and translates 6 channels into named outputs
// Useful for controlling devices (e.g., ESCs, servos) via RC transmitter

const int ppmPin = 9; // Digital pin D9 receives PPM signal from FS-iA6B receiver

// Timing variables
volatile unsigned long lastTime = 0;     // Last rising edge time
volatile int channelIndex = 0;           // Current channel index while reading
volatile int channelValues[6] = {0};     // Stores pulse widths for each of the 6 channels (in microseconds)

// Interrupt Service Routine (ISR) triggered on rising edge of PPM pulse
void IRAM_ATTR ppmISR() {
  unsigned long now = micros();          // Current time in microseconds
  unsigned long pulse = now - lastTime;  // Time since last pulse
  lastTime = now;                        // Update last pulse time

  if (pulse > 3000) {
    // Sync pulse detected (>3ms gap), start new frame
    channelIndex = 0;
  } else {
    // Save pulse width for current channel
    if (channelIndex < 6) {
      channelValues[channelIndex] = pulse;
      channelIndex++;
    }
  }
}

void setup() {
  Serial.begin(115200);           // Start serial communication
  pinMode(ppmPin, INPUT);         // Set PPM pin as input
  attachInterrupt(digitalPinToInterrupt(ppmPin), ppmISR, RISING); // Attach interrupt to read PPM
}

void loop() {
  // Assign channel values to common RC names
  int roll     = channelValues[0]; // Channel 1 tilting/moving left to right
  int pitch    = channelValues[1]; // Channel 2 tilting/moving forward and back
  int throttle = channelValues[2]; // Channel 3 power to motor
  int yaw      = channelValues[3]; // Channel 4 rotation
  int vra     = channelValues[4]; // Channel 5 nobs
  int vrb     = channelValues[5]; // Channel 6 nobs

  // Print to serial monitor
  Serial.print("Roll: ");     Serial.print(roll);
  Serial.print(" | Pitch: ");  Serial.print(pitch);
  Serial.print(" | Throttle: "); Serial.print(throttle);
  Serial.print(" | Yaw: ");    Serial.print(yaw);
  Serial.print(" | Aux1: ");   Serial.print(aux1);
  Serial.print(" | Aux2: ");   Serial.println(aux2);

  delay(50); // Short delay to avoid flooding the serial monitor
}
