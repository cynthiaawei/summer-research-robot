// robot_odometry.ino - Arduino sketch for WSL2
#include <Arduino_LSM6DS3.h>

float gx, gy, gz;
float pos_x = 0.0, pos_y = 0.0, heading = 0.0;
float vel_x = 0.0, vel_y = 0.0, vel_angular = 0.0;
unsigned long last_time;

// Calibration values
float gyro_offset_z = 0.0;
bool calibrated = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
 
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU failed");
    while (1);
  }
 
  // Calibrate gyroscope
  calibrate_gyro();
 
  Serial.println("ARDUINO_READY");
  last_time = millis();
}

void calibrate_gyro() {
  Serial.println("Calibrating gyro... keep robot still");
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      sum += gz;
    }
    delay(10);
  }
  gyro_offset_z = sum / 100.0;
  calibrated = true;
  Serial.println("Calibration complete");
}

void loop() {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
 
  // Read gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
   
    // Remove bias and convert to rad/s
    vel_angular = (gz - gyro_offset_z) * DEG_TO_RAD;
   
    // Only update if significant rotation
    if (abs(vel_angular) > 0.02) {
      heading += vel_angular * dt;
    }
   
    // Normalize heading to [-π, π]
    while (heading > PI) heading -= 2*PI;
    while (heading < -PI) heading += 2*PI;
  }
 
  // Placeholder velocities (replace with encoder data)
  vel_x = 0.0;
  vel_y = 0.0;
 
  // Update position
  pos_x += vel_x * cos(heading) * dt;
  pos_y += vel_x * sin(heading) * dt;
 
  // Send data every 50ms
  if (current_time - last_time >= 50) {
    Serial.print("ODOM:");
    Serial.print(pos_x, 4); Serial.print(",");
    Serial.print(pos_y, 4); Serial.print(",");
    Serial.print(heading, 4); Serial.print(",");
    Serial.print(vel_x, 4); Serial.print(",");
    Serial.print(vel_y, 4); Serial.print(",");
    Serial.println(vel_angular, 4);
   
    last_time = current_time;
  }
 
  // Handle commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
   
    if (cmd == "RESET") {
      pos_x = 0; pos_y = 0; heading = 0;
      vel_x = 0; vel_y = 0;
      Serial.println("RESET_OK");
    }
    else if (cmd == "PING") {
      Serial.println("PONG");
    }
    else if (cmd.startsWith("VEL:")) {
      int comma = cmd.indexOf(',');
      if (comma > 0) {
        vel_x = cmd.substring(4, comma).toFloat();
        vel_angular = cmd.substring(comma + 1).toFloat();
      }
    }
  }
 
  delay(10);
}