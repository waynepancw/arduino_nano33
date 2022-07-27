#include <Arduino_LSM9DS1.h>
#include <Servo.h>
#include "math.h"

Servo myservo;  // create servo object to control a servo

float ax, ay, az;
float gx, gy, gz;
float anglex=0, angley=0, anglez=0;
int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}


void loop() {

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    
    //Read accelerometer and gyroscope as degree
    
    float atanx = atan2((double)ay,(double)az)*180/PI;
    float gyrox = fmod(double(anglex+float(gx)*0.03/0.762), 360.0);

    float atany = atan2((double)ax,(double)az)*180/PI;
    float gyroy = fmod(double(angley+float(gy)*0.03/0.762), 360.0);

    float atanz = atan2((double)ay,(double)ax)*180/PI;
    float gyroz = fmod(double(angley+float(gy)*0.03/0.762), 360.0);

    int val;


    //Change value to get a value from 180 to 180°
    
    if (atanx < 0) {
      atanx = atanx + 360;
    }
    if (gyrox < 0) {
      gyrox = gyrox + 360;
    }
    
    if (atany < 0) {
      atany = atany + 360;
    }
    if (gyroy < 0) {
      gyroy = gyroy + 360;
    }

    if (atanz < 0) {
      atanz = atanz + 360;
    }
    if (gyroz < 0) {
      gyroz = gyroz + 360;
    }

    /*
     * Get angle smmoth from gyro and accel. 
     * Tried different way to change coef before gyro and accel.
     * All time coef gyro + coef accel = 1
    */
    anglex=0.6*gyrox + 0.4*atanx;
    angley=0.6*gyroy + 0.4*atany;
    anglez=0.6*gyroz + 0.4*atanz;

    delay(1000);

    val = map(anglez, 0, 1023, 0, 180); 
    myservo.write(val);              // tell servo to go to position in variable 'val'

    //print values
    Serial.print(anglex);
    Serial.print("/");
    Serial.print(angley);
    Serial.print("/");
    Serial.print(anglez);
    Serial.print("/");
    Serial.println(val);
    
  }
  delay(500);
}
