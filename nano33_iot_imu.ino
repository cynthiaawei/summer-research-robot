/*
Get the axis accelerations x, y z
Get the turns in x/y as degreesx, degreesy.
*/


#include <Arduino_LSM6DS3.h>

float x, y, z;
//float gx, gy, gz;

int degreesX = 0;
int degreesY = 0;

int plusThreshold = 30, minusThreshold = -30;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");  
  Serial.println();
  Serial.println("Gyroscope in degrees/second");

}

void loop() {


  //acceleration
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

  }

  Serial.print("x: ");
  Serial.print(x*9.80665);
  Serial.print(" m/s^2 \n");

  Serial.print("y: ");
  Serial.print(y*9.80665);
  Serial.print(" m/s^2 \n");

  //z values prob ~9.81 bc of gravity
  Serial.print("z: ");
  Serial.print(z*9.80665);
  Serial.print(" m/s^2 \n");
  delay(1000);


  //angle turns using accelerometer
  if (x > 0.1) {
    x = 100 * x;
    degreesX = map(x, 0, 97, 0, 90);
    Serial.print("Tilting up ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (x < -0.1) {
    x = 100 * x;
    degreesX = map(x, 0, -100, 0, 90);
    Serial.print("Tilting down ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (y > 0.1) {
    y = 100 * y;
    degreesY = map(y, 0, 97, 0, 90);
    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  if (y < -0.1) {
    y = 100 * y;
    degreesY = map(y, 0, -100, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  delay(1000);

}
  //turns
//   if (IMU.gyroscopeAvailable()) {
//     IMU.readGyroscope(gx, gy, gz);
//   }
//   if(gy > plusThreshold)
//   {
//     Serial.println("Collision front");
//     delay(500);
//   }
//   if(gy < minusThreshold)
//   {
//     Serial.println("Collision back");
//     delay(500);
//   }
//   if(gx < minusThreshold)
//   {
//     Serial.println("Collision right");
//     delay(500);
//   }
//     if(gx > plusThreshold)
//   {
//     Serial.println("Collision left");
//     delay(500);
//   }
// }