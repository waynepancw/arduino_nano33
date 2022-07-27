/*
  Arduino LSM9DS1 - Simple Gyroscope

  This example reads the gyroscope values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>
//#include <MadgwickAHRS.h>
//
//Madgwick filter;

void setup() {
  Serial.begin(9600);
  while (!Serial);
//  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
//  Serial.print("Gyroscope sample rate = ");
//  Serial.print(IMU.gyroscopeSampleRate());
//  Serial.println(" Hz");
//  Serial.println();
//  Serial.println("Gyroscope in degrees/second");
//  Serial.println("X\tY\tZ");
}

void loop() {
  float gx, gy, gz, ax, ay, az;
  float dt = 1/119;
  float roll, pitch, yaw;
  float Lroll, Lpitch, Lyaw;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    IMU.readAcceleration(ax, ay, az);

    roll = gx * dt + Lroll - Lpitch * (3.1416/180) * yaw * dt;         
    // Integrate the gyro roll rate and add the last computed roll angle
    // This line adds yaw coupling - using small angle approximation
    //lastFilteredPitchAngle * sin(DEGREES_TO_RADIANS * gyroYawRate * dt);     // Formula without small angle approximaion
    pitch = gy * dt + Lpitch + Lroll * (3.1416/180) * yaw * dt;             
    // Integrate the gyro pitch rate and add the last computed pitch angle
    // This line adds yaw coupling - using small angle approximation
    //lastFilteredRollAngle * sin(DEGREES_TO_RADIANS * gyroYawRate * dt);      // Formula without small angle approximation

//    Serial.print(gx);
//    Serial.print('\t');
//    Serial.print(gy);
//    Serial.print('\t');
//    Serial.print(gz);
//    Serial.print('\t');
//    Serial.print(ax);
//    Serial.print('\t');
//    Serial.print(ay);
//    Serial.print('\t');
//    Serial.println(az);
//    delay(500);

//    roll = filter.getRoll();
//    pitch = filter.getPitch();
//    yaw = filter.getYaw();
    
//    Serial.print(yaw);
//    Serial.print(",  ");
    Serial.print(pitch);
    Serial.print(",  ");
    Serial.println(roll);

    Lroll = roll;
    Lpitch = pitch;
  }
}
