/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  5 October 2022 -  initial release
*/

/*
Then, I ran it through ChatGPT given I'm using a Nano ESP32 and it fixed some issues.
*/

#include <Wire.h>
#include <stdint.h>

TwoWire I2C = TwoWire(0);

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

void gyro_signals(void) {
  I2C.beginTransmission(0x68);
  I2C.write(0x1A);
  I2C.write(0x05);
  I2C.endTransmission(); 

  I2C.beginTransmission(0x68);
  I2C.write(0x1B);
  I2C.write(0x08);
  I2C.endTransmission();

  I2C.beginTransmission(0x68);
  I2C.write(0x43);
  I2C.endTransmission(); 

  I2C.requestFrom(0x68,6);
  int16_t GyroX=I2C.read()<<8 | I2C.read();
  int16_t GyroY=I2C.read()<<8 | I2C.read();
  int16_t GyroZ=I2C.read()<<8 | I2C.read();
  
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  I2C.begin(8, 9, 400000); // SDA, SCL
  delay(250);
  I2C.beginTransmission(0x68); 
  I2C.write(0x6B);
  I2C.write(0x00);
  I2C.endTransmission();

  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }

  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;   
}

void loop() {
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  Serial.print("Roll rate [°/s]= ");
  Serial.print(RateRoll); 
  Serial.print(" Pitch Rate [°/s]= ");
  Serial.print(RatePitch);
  Serial.print(" Yaw Rate [°/s]= ");
  Serial.println(RateYaw);
  delay(50);
}
