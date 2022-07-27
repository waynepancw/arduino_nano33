#include <Arduino_LSM9DS1.h>
#include <Servo.h>
#include "math.h"

Servo myservo;  // create servo object to control a servo

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float anglex=0, angley=0, anglez=0;
int pos = 0;    // variable to store the servo position

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  if (IMU.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }   
}


void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float yaw;
  if (my == 0)
    yaw = (mx < 0) ? PI : 0;
  else
    yaw = atan2(mx, my);

  yaw -= DECLINATION * PI / 180;

  if (yaw > PI) yaw -= (2 * PI);
  else if (yaw < -PI) yaw += (2 * PI);

  // Convert everything from radians to degrees:
  yaw *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch:");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print("Roll:");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print("Yaw:"); 
  Serial.println(yaw, 2);
}


void loop()
{
  // Update the sensor values whenever new data is available
  if ( IMU.gyroAvailable() )
  {
    IMU.readGyro();
  }
  if ( IMU.accelAvailable() )
  {
    IMU.readAccel();
  }
  if ( IMU.magAvailable() )
  {
    IMU.readMag();
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    // Print the yaw and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    printAttitude(ax, ay, az,
                  -my, -mx, mz);
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }
}
