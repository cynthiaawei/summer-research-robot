// #include <Arduino_LSM6DS3.h>

// float gx, gy, gz, ax, ay, az;
// float pos_x = 0.0, pos_y = 0.0, heading = 0.0;
// float vel_x = 0.0, vel_y = 0.0, vel_angular = 0.0;
// unsigned long last_integration_time = 0;
// unsigned long last_print_time = 0;

// // Calibration offsets
// float gyro_offset_z = 0.0;
// float accel_offset_x = 0.0, accel_offset_y = 0.0;

// // Low-pass filter for accelerometer
// float alpha = 0.2; // Filter constant
// float filtered_ax = 0.0, filtered_ay = 0.0;

// // CRITICAL: Much more aggressive thresholds to stop drift
// const float VELOCITY_DECAY = 0.85;        // Was 0.98 - much more aggressive!
// const float MIN_VELOCITY_THRESHOLD = 0.02; // Was 0.005 - higher to stop drift
// const float ACCEL_THRESHOLD = 0.25;       // Was 0.12 - higher to ignore small accelerations
// const float MAX_VELOCITY = 0.5;           // Was 1.5 - much lower max velocity
// const float MIN_DT = 0.015;
// const float MAX_DT = 0.08;

// // ZERO VELOCITY UPDATE - if robot is stationary, force velocities to zero
// bool is_stationary = true;
// unsigned long stationary_start_time = 0;
// const unsigned long STATIONARY_TIME_MS = 200; // 200ms of low accel = stationary

// void setup() {
//   Serial.begin(115200);
//   delay(2000);
  
//   Serial.println("=== Arduino OMNI DRIVE Odometry (DRIFT FIXED) ===");
  
//   if (!IMU.begin()) {
//     Serial.println("ERROR: IMU failed!");
//     while (1) {
//       delay(1000);
//       Serial.println("IMU still failed...");
//     }
//   }
  
//   Serial.println("✅ IMU OK!");
  
//   // EXTENDED calibration - more samples, better filtering
//   Serial.println("📏 Calibrating... keep robot PERFECTLY STILL on LEVEL surface!");
//   delay(3000); // Longer delay
  
//   float gyro_sum = 0, accel_x_sum = 0, accel_y_sum = 0;
//   int valid_readings = 0;
  
//   // More calibration samples with outlier rejection
//   for (int i = 0; i < 1000; i++) { // More samples!
//     if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
//       IMU.readAcceleration(ax, ay, az);
//       IMU.readGyroscope(gx, gy, gz);
      
//       // Reject outliers - only use reasonable readings
//       if (abs(gz) < 10 && abs(ax) < 1.5 && abs(ay) < 1.5 && abs(az - 1.0) < 0.5) {
//         gyro_sum += gz;
//         accel_x_sum += ax;
//         accel_y_sum += ay;
//         valid_readings++;
//       }
//     }
//     delay(5);
//     if (i % 100 == 0) Serial.print(".");
//   }
  
//   if (valid_readings < 500) {
//     Serial.println("\nERROR: Too few valid calibration readings! Check IMU mounting.");
//     while(1) delay(1000);
//   }
  
//   gyro_offset_z = gyro_sum / valid_readings;
//   accel_offset_x = accel_x_sum / valid_readings;
//   accel_offset_y = accel_y_sum / valid_readings;
  
//   // Initialize filtered values
//   filtered_ax = accel_offset_x;
//   filtered_ay = accel_offset_y;
  
//   Serial.println();
//   Serial.print("Valid readings: "); Serial.println(valid_readings);
//   Serial.print("Gyro Z offset: "); Serial.println(gyro_offset_z, 6);
//   Serial.print("Accel X offset: "); Serial.println(accel_offset_x, 6);
//   Serial.print("Accel Y offset: "); Serial.println(accel_offset_y, 6);
  
//   // WARN if offsets seem wrong
//   if (abs(accel_offset_x) > 0.3 || abs(accel_offset_y) > 0.3) {
//     Serial.println("⚠️ WARNING: Large accelerometer offsets detected!");
//     Serial.println("⚠️ Robot may not be level or IMU may be miscalibrated!");
//   }
  
//   Serial.println("✅ Calibration done!");
//   Serial.println("🚀 Starting DRIFT-RESISTANT odometry...");
  
//   unsigned long now = millis();
//   last_integration_time = now;
//   last_print_time = now;
//   stationary_start_time = now;
// }

// void loop() {
//   unsigned long current_time = millis();
  
//   // Handle millis() rollover
//   if (current_time < last_integration_time) {
//     last_integration_time = current_time;
//     return;
//   }
  
//   float dt = (current_time - last_integration_time) / 1000.0;
  
//   if (dt < MIN_DT) {
//     return;
//   }
  
//   last_integration_time = current_time;
  
//   if (dt > MAX_DT) {
//     dt = MAX_DT;
//   }
  
//   // Read sensors
//   if (!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable())) {
//     delay(2);
//     return;
//   }
  
//   if (!IMU.readAcceleration(ax, ay, az) || !IMU.readGyroscope(gx, gy, gz)) {
//     Serial.println("⚠️ Sensor read failed");
//     delay(5);
//     return;
//   }
  
//   // Sanity check
//   if (abs(ax) > 4.0 || abs(ay) > 4.0 || abs(az) > 4.0 || 
//       abs(gx) > 500 || abs(gy) > 500 || abs(gz) > 500) {
//     return;
//   }
  
//   // Angular velocity processing - SINGLE DEADZONE FILTER
//   float gz_corrected = gz - gyro_offset_z;
//   vel_angular = gz_corrected * DEG_TO_RAD;
  
//   // Single, more aggressive deadzone filter
//   if (fabs(vel_angular) < 0.035) {  // Increased from 0.02 to 0.035
//     vel_angular = 0.0;
//   }
  
//   heading += vel_angular * dt;
//   while (heading > PI) heading -= 2*PI;
//   while (heading < -PI) heading += 2*PI;
  
//   // Apply stronger low-pass filter
//   filtered_ax = alpha * ax + (1.0 - alpha) * filtered_ax;
//   filtered_ay = alpha * ay + (1.0 - alpha) * filtered_ay;
  
//   // Linear acceleration processing
//   float accel_x_clean = filtered_ax - accel_offset_x;
//   float accel_y_clean = filtered_ay - accel_offset_y;
//   float accel_x_ms2 = accel_x_clean * 9.81;
//   float accel_y_ms2 = accel_y_clean * 9.81;
  
//   // ZERO VELOCITY UPDATE - detect if robot is stationary
//   float total_accel_magnitude = sqrt(accel_x_ms2*accel_x_ms2 + accel_y_ms2*accel_y_ms2);
  
//   if (total_accel_magnitude < ACCEL_THRESHOLD && abs(vel_angular) < 0.035) {
//     // Robot appears stationary
//     if (!is_stationary) {
//       stationary_start_time = current_time;
//       is_stationary = true;
//     }
    
//     // If stationary for long enough, force velocities to zero
//     if (current_time - stationary_start_time > STATIONARY_TIME_MS) {
//       vel_x = 0.0;
//       vel_y = 0.0;
//       // Don't print debug messages when stationary
//     }
//   } else {
//     is_stationary = false;
    
//     // Process X acceleration ONLY if above threshold
//     if (fabs(accel_x_ms2) > ACCEL_THRESHOLD) {
//       vel_x += accel_x_ms2 * dt;
//       if (fabs(accel_x_ms2) > 0.5) { // Only log significant accelerations
//         Serial.print("🚀 X: "); Serial.print(accel_x_ms2, 2);
//         Serial.print(" → vel_x="); Serial.println(vel_x, 3);
//       }
//     }
    
//     // Process Y acceleration ONLY if above threshold  
//     if (fabs(accel_y_ms2) > ACCEL_THRESHOLD) {
//       vel_y += accel_y_ms2 * dt;
//       if (fabs(accel_y_ms2) > 0.5) { // Only log significant accelerations
//         Serial.print("🔄 Y: "); Serial.print(accel_y_ms2, 2);
//         Serial.print(" → vel_y="); Serial.println(vel_y, 3);
//       }
//     }
//   }
  
//   // ALWAYS apply velocity decay (even when stationary)
//   vel_x *= VELOCITY_DECAY;
//   vel_y *= VELOCITY_DECAY;
  
//   // Clamp velocities to reasonable limits
//   vel_x = constrain(vel_x, -MAX_VELOCITY, MAX_VELOCITY);
//   vel_y = constrain(vel_y, -MAX_VELOCITY, MAX_VELOCITY);
  
//   // Stop very small velocities
//   if (fabs(vel_x) < MIN_VELOCITY_THRESHOLD) vel_x = 0.0;
//   if (fabs(vel_y) < MIN_VELOCITY_THRESHOLD) vel_y = 0.0;
  
//   // Position integration
//   pos_x += vel_x * dt;
//   pos_y += vel_y * dt;
  
//   // Output telemetry every 60ms
//   if (current_time - last_print_time >= 60) {
//     Serial.print("ODOM:");
//     Serial.print(pos_x, 4); Serial.print(",");
//     Serial.print(pos_y, 4); Serial.print(",");
//     Serial.print(heading, 3); Serial.print(",");
//     Serial.print(vel_x, 3); Serial.print(",");
//     Serial.print(vel_y, 3); Serial.print(",");
//     Serial.println(vel_angular, 3);
    
//     last_print_time = current_time;
//   }
  
//   // Commands
//   if (Serial.available()) {
//     String cmd = Serial.readStringUntil('\n');
//     cmd.trim();
//     cmd.toUpperCase();
    
//     if (cmd == "RESET") {
//       pos_x = 0; pos_y = 0; heading = 0; 
//       vel_x = 0; vel_y = 0; vel_angular = 0;
//       is_stationary = true;
//       stationary_start_time = current_time;
//       Serial.println("RESET_OK");
//     }
//     else if (cmd == "RECALIBRATE") {
//       Serial.println("🔄 EMERGENCY RECALIBRATION - keep perfectly still!");
//       delay(1000);
      
//       // Quick recalibration
//       float gyro_sum = 0, accel_x_sum = 0, accel_y_sum = 0;
//       int samples = 0;
      
//       for (int i = 0; i < 200; i++) {
//         if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
//           IMU.readAcceleration(ax, ay, az);
//           IMU.readGyroscope(gx, gy, gz);
//           gyro_sum += gz;
//           accel_x_sum += ax;
//           accel_y_sum += ay;
//           samples++;
//         }
//         delay(10);
//       }
      
//       if (samples > 100) {
//         gyro_offset_z = gyro_sum / samples;
//         accel_offset_x = accel_x_sum / samples;
//         accel_offset_y = accel_y_sum / samples;
        
//         filtered_ax = accel_offset_x;
//         filtered_ay = accel_offset_y;
        
//         Serial.print("New offsets - X: "); Serial.print(accel_offset_x, 6);
//         Serial.print(" Y: "); Serial.print(accel_offset_y, 6);
//         Serial.print(" Gyro: "); Serial.println(gyro_offset_z, 6);
//         Serial.println("RECALIBRATE_OK");
//       } else {
//         Serial.println("RECALIBRATE_FAILED");
//       }
//     }
//     else if (cmd == "TEST") {
//       Serial.println("=== DEBUG INFO ===");
//       Serial.print("Stationary: "); Serial.println(is_stationary ? "YES" : "NO");
//       Serial.print("Total accel magnitude: "); Serial.println(total_accel_magnitude, 4);
//       Serial.print("Raw Y accel: "); Serial.println(ay, 6);
//       Serial.print("Filtered Y accel: "); Serial.println(filtered_ay, 6);
//       Serial.print("Y offset: "); Serial.println(accel_offset_y, 6);
//       Serial.print("Clean Y accel: "); Serial.println(accel_y_clean, 6);
//       Serial.print("Y accel m/s²: "); Serial.println(accel_y_ms2, 4);
//       Serial.print("Current velocities: vx="); Serial.print(vel_x, 4);
//       Serial.print(" vy="); Serial.println(vel_y, 4);
//     }
//     else if (cmd == "STATUS") {
//       Serial.print("Position: ("); Serial.print(pos_x, 3); Serial.print(","); Serial.print(pos_y, 3);
//       Serial.print(") Stationary: "); Serial.print(is_stationary ? "YES" : "NO");
//       Serial.print(" Total accel: "); Serial.println(total_accel_magnitude, 3);
//     }
//     else if (cmd == "PING") {
//       Serial.println("PONG");
//     }
//     else if (cmd == "HELP") {
//       Serial.println("Commands: RESET, RECALIBRATE, TEST, STATUS, PING, HELP");
//     }
//   }
  
//   delay(3);
// }
#include <Arduino_LSM6DS3.h>

float gx, gy, gz, ax, ay, az;
float pos_x = 0.0, pos_y = 0.0, heading = 0.0;
float vel_x = 0.0, vel_y = 0.0, vel_angular = 0.0;
unsigned long last_integration_time = 0;
unsigned long last_print_time = 0;

// Calibration offsets
float gyro_offset_z = 0.0;
float accel_offset_x = 0.0, accel_offset_y = 0.0;

// Low-pass filter for accelerometer
float alpha = 0.2; // Filter constant
float filtered_ax = 0.0, filtered_ay = 0.0;

// CRITICAL: Much more aggressive thresholds to stop drift
const float VELOCITY_DECAY = 0.85;        // Was 0.98 - much more aggressive!
const float MIN_VELOCITY_THRESHOLD = 0.02; // Was 0.005 - higher to stop drift
const float ACCEL_THRESHOLD = 0.25;       // Was 0.12 - higher to ignore small accelerations
const float MAX_VELOCITY = 0.5;           // Was 1.5 - much lower max velocity
const float MIN_DT = 0.015;
const float MAX_DT = 0.08;

// ZERO VELOCITY UPDATE - if robot is stationary, force velocities to zero
bool is_stationary = true;
unsigned long stationary_start_time = 0;
const unsigned long STATIONARY_TIME_MS = 200; // 200ms of low accel = stationary

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== Arduino OMNI DRIVE Odometry (RAW DATA TO ODOM) ===");
  
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU failed!");
    while (1) {
      delay(1000);
      Serial.println("IMU still failed...");
    }
  }
  
  Serial.println("✅ IMU OK!");
  
  // EXTENDED calibration - more samples, better filtering
  Serial.println("📏 Calibrating... keep robot PERFECTLY STILL on LEVEL surface!");
  delay(3000); // Longer delay
  
  float gyro_sum = 0, accel_x_sum = 0, accel_y_sum = 0;
  int valid_readings = 0;
  
  // More calibration samples with outlier rejection
  for (int i = 0; i < 1000; i++) { // More samples!
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      // Reject outliers - only use reasonable readings
      if (abs(gz) < 10 && abs(ax) < 1.5 && abs(ay) < 1.5 && abs(az - 1.0) < 0.5) {
        gyro_sum += gz;
        accel_x_sum += ax;
        accel_y_sum += ay;
        valid_readings++;
      }
    }
    delay(5);
    if (i % 100 == 0) Serial.print(".");
  }
  
  if (valid_readings < 500) {
    Serial.println("\nERROR: Too few valid calibration readings! Check IMU mounting.");
    while(1) delay(1000);
  }
  
  gyro_offset_z = gyro_sum / valid_readings;
  accel_offset_x = accel_x_sum / valid_readings;
  accel_offset_y = accel_y_sum / valid_readings;
  
  // Initialize filtered values
  filtered_ax = accel_offset_x;
  filtered_ay = accel_offset_y;
  
  Serial.println();
  Serial.print("Valid readings: "); Serial.println(valid_readings);
  Serial.print("Gyro Z offset: "); Serial.println(gyro_offset_z, 6);
  Serial.print("Accel X offset: "); Serial.println(accel_offset_x, 6);
  Serial.print("Accel Y offset: "); Serial.println(accel_offset_y, 6);
  
  // WARN if offsets seem wrong
  if (abs(accel_offset_x) > 0.3 || abs(accel_offset_y) > 0.3) {
    Serial.println("⚠️ WARNING: Large accelerometer offsets detected!");
    Serial.println("⚠️ Robot may not be level or IMU may be miscalibrated!");
  }
  
  Serial.println("✅ Calibration done!");
  Serial.println("🚀 Starting DRIFT-RESISTANT odometry...");
  Serial.println("📡 Sending RAW angular velocity to odometry node");
  
  unsigned long now = millis();
  last_integration_time = now;
  last_print_time = now;
  stationary_start_time = now;
}

void loop() {
  unsigned long current_time = millis();
  
  // Handle millis() rollover
  if (current_time < last_integration_time) {
    last_integration_time = current_time;
    return;
  }
  
  float dt = (current_time - last_integration_time) / 1000.0;
  
  if (dt < MIN_DT) {
    return;
  }
  
  last_integration_time = current_time;
  
  if (dt > MAX_DT) {
    dt = MAX_DT;
  }
  
  // Read sensors
  if (!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable())) {
    delay(2);
    return;
  }
  
  if (!IMU.readAcceleration(ax, ay, az) || !IMU.readGyroscope(gx, gy, gz)) {
    Serial.println("⚠️ Sensor read failed");
    delay(5);
    return;
  }
  
  // Sanity check
  if (abs(ax) > 4.0 || abs(ay) > 4.0 || abs(az) > 4.0 || 
      abs(gx) > 500 || abs(gy) > 500 || abs(gz) > 500) {
    return;
  }
  
  // Angular velocity processing - KEEP RAW VALUE FOR ODOM NODE
  float gz_corrected = gz - gyro_offset_z;
  float vel_angular_raw = gz_corrected * DEG_TO_RAD;  // RAW value for odometry node
  
  // Copy for Arduino's internal use (heading integration)
  vel_angular = vel_angular_raw;
  
  // Apply deadzone ONLY for Arduino's heading integration (prevents drift)
  if (fabs(vel_angular) < 0.035) {
    vel_angular = 0.0;  // Used only for heading calculation
  }
  
  // Heading integration uses FILTERED value (prevents drift)
  heading += vel_angular * dt;
  while (heading > PI) heading -= 2*PI;
  while (heading < -PI) heading += 2*PI;
  
  // Apply stronger low-pass filter
  filtered_ax = alpha * ax + (1.0 - alpha) * filtered_ax;
  filtered_ay = alpha * ay + (1.0 - alpha) * filtered_ay;
  
  // Linear acceleration processing
  float accel_x_clean = filtered_ax - accel_offset_x;
  float accel_y_clean = filtered_ay - accel_offset_y;
  float accel_x_ms2 = accel_x_clean * 9.81;
  float accel_y_ms2 = accel_y_clean * 9.81;
  
  // ZERO VELOCITY UPDATE - detect if robot is stationary
  float total_accel_magnitude = sqrt(accel_x_ms2*accel_x_ms2 + accel_y_ms2*accel_y_ms2);
  
  if (total_accel_magnitude < ACCEL_THRESHOLD && abs(vel_angular) < 0.035) {
    // Robot appears stationary
    if (!is_stationary) {
      stationary_start_time = current_time;
      is_stationary = true;
    }
    
    // If stationary for long enough, force velocities to zero
    if (current_time - stationary_start_time > STATIONARY_TIME_MS) {
      vel_x = 0.0;
      vel_y = 0.0;
      // Don't print debug messages when stationary
    }
  } else {
    is_stationary = false;
    
    // Process X acceleration ONLY if above threshold
    if (fabs(accel_x_ms2) > ACCEL_THRESHOLD) {
      vel_x += accel_x_ms2 * dt;
      if (fabs(accel_x_ms2) > 0.5) { // Only log significant accelerations
        Serial.print("🚀 X: "); Serial.print(accel_x_ms2, 2);
        Serial.print(" → vel_x="); Serial.println(vel_x, 3);
      }
    }
    
    // Process Y acceleration ONLY if above threshold  
    if (fabs(accel_y_ms2) > ACCEL_THRESHOLD) {
      vel_y += accel_y_ms2 * dt;
      if (fabs(accel_y_ms2) > 0.5) { // Only log significant accelerations
        Serial.print("🔄 Y: "); Serial.print(accel_y_ms2, 2);
        Serial.print(" → vel_y="); Serial.println(vel_y, 3);
      }
    }
  }
  
  // ALWAYS apply velocity decay (even when stationary)
  vel_x *= VELOCITY_DECAY;
  vel_y *= VELOCITY_DECAY;
  
  // Clamp velocities to reasonable limits
  vel_x = constrain(vel_x, -MAX_VELOCITY, MAX_VELOCITY);
  vel_y = constrain(vel_y, -MAX_VELOCITY, MAX_VELOCITY);
  
  // Stop very small velocities
  if (fabs(vel_x) < MIN_VELOCITY_THRESHOLD) vel_x = 0.0;
  if (fabs(vel_y) < MIN_VELOCITY_THRESHOLD) vel_y = 0.0;
  
  // Position integration
  pos_x += vel_x * dt;
  pos_y += vel_y * dt;
  
  // Output telemetry every 60ms - SEND RAW ANGULAR VELOCITY
  if (current_time - last_print_time >= 60) {
    Serial.print("ODOM:");
    Serial.print(pos_x, 4); Serial.print(",");
    Serial.print(pos_y, 4); Serial.print(",");
    Serial.print(heading, 3); Serial.print(",");
    Serial.print(vel_x, 3); Serial.print(",");
    Serial.print(vel_y, 3); Serial.print(",");
    Serial.println(vel_angular_raw, 3);  // SEND RAW VALUE to odometry node!
    
    last_print_time = current_time;
  }
  
  // Commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "RESET") {
      pos_x = 0; pos_y = 0; heading = 0; 
      vel_x = 0; vel_y = 0; vel_angular = 0;
      is_stationary = true;
      stationary_start_time = current_time;
      Serial.println("RESET_OK");
    }
    else if (cmd == "RECALIBRATE") {
      Serial.println("🔄 EMERGENCY RECALIBRATION - keep perfectly still!");
      delay(1000);
      
      // Quick recalibration
      float gyro_sum = 0, accel_x_sum = 0, accel_y_sum = 0;
      int samples = 0;
      
      for (int i = 0; i < 200; i++) {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
          IMU.readAcceleration(ax, ay, az);
          IMU.readGyroscope(gx, gy, gz);
          gyro_sum += gz;
          accel_x_sum += ax;
          accel_y_sum += ay;
          samples++;
        }
        delay(10);
      }
      
      if (samples > 100) {
        gyro_offset_z = gyro_sum / samples;
        accel_offset_x = accel_x_sum / samples;
        accel_offset_y = accel_y_sum / samples;
        
        filtered_ax = accel_offset_x;
        filtered_ay = accel_offset_y;
        
        Serial.print("New offsets - X: "); Serial.print(accel_offset_x, 6);
        Serial.print(" Y: "); Serial.print(accel_offset_y, 6);
        Serial.print(" Gyro: "); Serial.println(gyro_offset_z, 6);
        Serial.println("RECALIBRATE_OK");
      } else {
        Serial.println("RECALIBRATE_FAILED");
      }
    }
    else if (cmd == "TEST") {
      Serial.println("=== DEBUG INFO ===");
      Serial.print("Stationary: "); Serial.println(is_stationary ? "YES" : "NO");
      Serial.print("Total accel magnitude: "); Serial.println(total_accel_magnitude, 4);
      Serial.print("Raw angular velocity: "); Serial.println(vel_angular_raw, 6);
      Serial.print("Filtered angular (for heading): "); Serial.println(vel_angular, 6);
      Serial.print("Current heading: "); Serial.println(heading, 4);
      Serial.print("Current velocities: vx="); Serial.print(vel_x, 4);
      Serial.print(" vy="); Serial.println(vel_y, 4);
    }
    else if (cmd == "STATUS") {
      Serial.print("Position: ("); Serial.print(pos_x, 3); Serial.print(","); Serial.print(pos_y, 3);
      Serial.print(") Stationary: "); Serial.print(is_stationary ? "YES" : "NO");
      Serial.print(" Total accel: "); Serial.println(total_accel_magnitude, 3);
    }
    else if (cmd == "PING") {
      Serial.println("PONG");
    }
    else if (cmd == "HELP") {
      Serial.println("Commands: RESET, RECALIBRATE, TEST, STATUS, PING, HELP");
    }
  }
  
  delay(3);
}