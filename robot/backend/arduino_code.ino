#include <Arduino_LSM6DS3.h>

// State variables
float pos_x = 0.0, pos_y = 0.0, heading = 0.0;
float vel_x = 0.0, vel_angular = 0.0;
unsigned long last_time = 0;

// Calibration offsets
float gyro_offset_z = 0.0;
float accel_offset_x = 0.0;
bool calibrated = false;

// Simple parameters
const float ACCEL_THRESHOLD = 0.2;  // m/s² threshold
const float VEL_DECAY = 0.95;       // Velocity decay factor
const float MAX_VEL = 2.0;          // Max velocity limit

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== Working Arduino Odometry ===");
  
  if (!IMU.begin()) {
    Serial.println("❌ IMU failed");
    while (1) delay(1000);
  }
  
  Serial.println("✅ IMU initialized");
  
  // Simple calibration
  calibrate_imu();
  
  Serial.println("🚀 Starting odometry...");
  last_time = millis();
}

void calibrate_imu() {
  Serial.println("📏 Calibrating... keep robot still!");
  
  float gyro_sum = 0, accel_sum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      gyro_sum += gz;
      accel_sum += ax;
    }
    
    delay(20);
    if (i % 20 == 0) Serial.print(".");
  }
  
  gyro_offset_z = gyro_sum / samples;
  accel_offset_x = accel_sum / samples;
  calibrated = true;
  
  Serial.println();
  Serial.print("Gyro offset: "); Serial.println(gyro_offset_z, 4);
  Serial.print("Accel offset: "); Serial.println(accel_offset_x, 4);
  Serial.println("✅ Calibration complete!");
}

void loop() {
  unsigned long current_time = millis();
  
  // Calculate time step (convert to seconds)
  float dt = (current_time - last_time) / 1000.0;
  
  // Skip if time step is too small or too large
  if (dt < 0.02 || dt > 0.2) {
    return;  // Don't update last_time yet
  }
  
  // Read IMU
  float ax, ay, az, gx, gy, gz;
  bool imu_ok = false;
  
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    imu_ok = true;
  }
  
  if (!imu_ok) {
    Serial.println("⚠️ IMU read failed");
    delay(50);
    return;
  }
  
  // === ANGULAR VELOCITY (Gyroscope) ===
  vel_angular = (gz - gyro_offset_z) * DEG_TO_RAD;
  
  // Apply deadband
  if (abs(vel_angular) < 0.02) {
    vel_angular = 0.0;
  }
  
  // Update heading
  heading += vel_angular * dt;
  
  // Normalize heading to [-π, π]
  if (heading > PI) heading -= 2.0 * PI;
  if (heading < -PI) heading += 2.0 * PI;
  
  // === LINEAR VELOCITY (Accelerometer) ===
  float accel_x_clean = ax - accel_offset_x;
  float accel_x_ms2 = accel_x_clean * 9.81;  // Convert g to m/s²
  
  // Apply threshold and integrate
  if (abs(accel_x_ms2) > ACCEL_THRESHOLD) {
    vel_x += accel_x_ms2 * dt;
    
    // Debug output when acceleration detected
    Serial.print("🚀 Accel: "); Serial.print(accel_x_ms2, 2);
    Serial.print(" m/s², new vel_x: "); Serial.println(vel_x, 3);
  } else {
    // Apply decay when no significant acceleration
    vel_x *= VEL_DECAY;
  }
  
  // Limit velocity
  vel_x = constrain(vel_x, -MAX_VEL, MAX_VEL);
  
  // Stop very small velocities
  if (abs(vel_x) < 0.02) {
    vel_x = 0.0;
  }
  
  // === POSITION INTEGRATION ===
  pos_x += vel_x * cos(heading) * dt;
  pos_y += vel_x * sin(heading) * dt;
  
  // Send odometry data every 100ms
  if (current_time - last_time >= 100) {
    Serial.print("ODOM:");
    Serial.print(pos_x, 6); Serial.print(",");
    Serial.print(pos_y, 6); Serial.print(",");
    Serial.print(heading, 4); Serial.print(",");
    Serial.print(vel_x, 4); Serial.print(",");
    Serial.print("0.0000,"); // vel_y always 0
    Serial.println(vel_angular, 4);
    
    last_time = current_time;
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "RESET") {
      pos_x = 0.0; pos_y = 0.0; heading = 0.0;
      vel_x = 0.0; vel_angular = 0.0;
      Serial.println("RESET_OK");
    }
    else if (cmd == "STATUS") {
      Serial.print("STATUS: pos("); Serial.print(pos_x, 3);
      Serial.print(","); Serial.print(pos_y, 3); Serial.print(") ");
      Serial.print("heading("); Serial.print(heading, 3); Serial.print(") ");
      Serial.print("vel("); Serial.print(vel_x, 3); Serial.println(")");
    }
    else if (cmd == "DEBUG") {
      Serial.print("DEBUG: raw_ax="); Serial.print(ax, 4);
      Serial.print(" clean_ax="); Serial.print(accel_x_clean, 4);
      Serial.print(" accel_ms2="); Serial.println(accel_x_ms2, 3);
    }
    else if (cmd == "PING") {
      Serial.println("PONG");
    }
  }
  
  delay(10);  // Small delay for stability
}