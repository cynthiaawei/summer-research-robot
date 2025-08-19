/*1. Type: MEASURE
   - Collects 500 samples over 25 seconds
   - Keep robot PERFECTLY still during this time

2. Type: ANALYSIS  
   - Shows detailed noise statistics
   - Gives you recommended thresholds

3. Type: LIVE
   - Shows real-time readings so you can see noise live
*/



#include <Arduino_LSM6DS3.h>

// Simple noise measurement script
float gx, gy, gz, ax, ay, az;
float gyro_offset_z = 0.0;
float accel_offset_x = 0.0, accel_offset_y = 0.0;

// Storage for noise analysis
#define MAX_SAMPLES 500
float angular_samples[MAX_SAMPLES];
float accel_x_samples[MAX_SAMPLES];
float accel_y_samples[MAX_SAMPLES];
int sample_count = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== ADC NOISE MEASUREMENT TOOL ===");
  
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU failed!");
    while (1) delay(1000);
  }
  
  Serial.println("✅ IMU initialized");
  Serial.println("📏 Quick calibration - keep robot STILL!");
  delay(2000);
  
  // Quick calibration
  float gyro_sum = 0, accel_x_sum = 0, accel_y_sum = 0;
  int cal_samples = 0;
  
  for (int i = 0; i < 200; i++) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      gyro_sum += gz;
      accel_x_sum += ax;
      accel_y_sum += ay;
      cal_samples++;
    }
    delay(10);
  }
  
  gyro_offset_z = gyro_sum / cal_samples;
  accel_offset_x = accel_x_sum / cal_samples;
  accel_offset_y = accel_y_sum / cal_samples;
  
  Serial.println("✅ Calibration complete");
  Serial.print("Gyro offset: "); Serial.println(gyro_offset_z, 6);
  Serial.print("Accel X offset: "); Serial.println(accel_offset_x, 6);
  Serial.print("Accel Y offset: "); Serial.println(accel_offset_y, 6);
  
  Serial.println("\n🔍 NOISE MEASUREMENT READY");
  Serial.println("Commands:");
  Serial.println("  MEASURE - Collect noise data (keep robot PERFECTLY still!)");
  Serial.println("  LIVE - Show live readings");
  Serial.println("  ANALYSIS - Analyze collected data");
  Serial.println("  CLEAR - Clear collected data");
  Serial.println("  HELP - Show commands");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "MEASURE") {
      measureNoise();
    }
    else if (cmd == "LIVE") {
      showLiveReadings();
    }
    else if (cmd == "ANALYSIS") {
      analyzeNoise();
    }
    else if (cmd == "CLEAR") {
      sample_count = 0;
      Serial.println("✅ Data cleared");
    }
    else if (cmd == "HELP") {
      Serial.println("\nCommands:");
      Serial.println("  MEASURE - Collect 500 noise samples");
      Serial.println("  LIVE - Show real-time sensor readings");
      Serial.println("  ANALYSIS - Analyze noise statistics");
      Serial.println("  CLEAR - Clear collected data");
    }
    else {
      Serial.println("Unknown command. Type HELP for commands.");
    }
  }
  
  delay(10);
}

void measureNoise() {
  Serial.println("🔍 MEASURING NOISE - Keep robot PERFECTLY STILL!");
  Serial.println("Collecting 500 samples over 25 seconds...");
  
  sample_count = 0;
  
  for (int i = 0; i < MAX_SAMPLES; i++) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      // Calculate corrected values
      float gz_corrected = gz - gyro_offset_z;
      float angular_vel = gz_corrected * DEG_TO_RAD;
      float accel_x_corrected = ax - accel_offset_x;
      float accel_y_corrected = ay - accel_offset_y;
      
      // Store samples
      angular_samples[sample_count] = angular_vel;
      accel_x_samples[sample_count] = accel_x_corrected;
      accel_y_samples[sample_count] = accel_y_corrected;
      sample_count++;
      
      // Progress indicator
      if (i % 50 == 0) {
        Serial.print("Progress: "); 
        Serial.print((i * 100) / MAX_SAMPLES); 
        Serial.println("%");
      }
    }
    delay(50); // 20Hz sampling
  }
  
  Serial.println("✅ Data collection complete!");
  Serial.println("Type ANALYSIS to see results");
}

void showLiveReadings() {
  Serial.println("📊 LIVE READINGS - Press any key to stop");
  Serial.println("Format: Angular(rad/s) | AccelX | AccelY");
  
  unsigned long start_time = millis();
  while (!Serial.available() && (millis() - start_time < 30000)) { // 30 second timeout
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      float gz_corrected = gz - gyro_offset_z;
      float angular_vel = gz_corrected * DEG_TO_RAD;
      float accel_x_corrected = ax - accel_offset_x;
      float accel_y_corrected = ay - accel_offset_y;
      
      Serial.print("ω: "); Serial.print(angular_vel, 4);
      Serial.print(" | X: "); Serial.print(accel_x_corrected, 4);
      Serial.print(" | Y: "); Serial.print(accel_y_corrected, 4);
      Serial.println();
      
      delay(100);
    }
  }
  
  // Clear any input
  while (Serial.available()) Serial.read();
  Serial.println("Live readings stopped");
}

void analyzeNoise() {
  if (sample_count == 0) {
    Serial.println("❌ No data collected. Run MEASURE first.");
    return;
  }
  
  Serial.println("\n=== NOISE ANALYSIS RESULTS ===");
  Serial.print("Samples analyzed: "); Serial.println(sample_count);
  
  // Angular velocity analysis
  float angular_min = 999, angular_max = -999;
  float angular_sum = 0, angular_abs_sum = 0;
  
  for (int i = 0; i < sample_count; i++) {
    float val = angular_samples[i];
    if (val < angular_min) angular_min = val;
    if (val > angular_max) angular_max = val;
    angular_sum += val;
    angular_abs_sum += abs(val);
  }
  
  float angular_mean = angular_sum / sample_count;
  float angular_abs_mean = angular_abs_sum / sample_count;
  float angular_range = angular_max - angular_min;
  
  // Calculate standard deviation
  float angular_var_sum = 0;
  for (int i = 0; i < sample_count; i++) {
    float diff = angular_samples[i] - angular_mean;
    angular_var_sum += diff * diff;
  }
  float angular_std = sqrt(angular_var_sum / sample_count);
  
  // Find 95th percentile of absolute values
  float angular_sorted[MAX_SAMPLES];
  for (int i = 0; i < sample_count; i++) {
    angular_sorted[i] = abs(angular_samples[i]);
  }
  // Simple bubble sort for small arrays
  for (int i = 0; i < sample_count - 1; i++) {
    for (int j = 0; j < sample_count - i - 1; j++) {
      if (angular_sorted[j] > angular_sorted[j + 1]) {
        float temp = angular_sorted[j];
        angular_sorted[j] = angular_sorted[j + 1];
        angular_sorted[j + 1] = temp;
      }
    }
  }
  int p95_index = (int)(sample_count * 0.95);
  float angular_p95 = angular_sorted[p95_index];
  
  Serial.println("\n--- ANGULAR VELOCITY (rad/s) ---");
  Serial.print("Range: "); Serial.print(angular_min, 4); 
  Serial.print(" to "); Serial.println(angular_max, 4);
  Serial.print("Total range: "); Serial.println(angular_range, 4);
  Serial.print("Mean: "); Serial.println(angular_mean, 4);
  Serial.print("Mean absolute: "); Serial.println(angular_abs_mean, 4);
  Serial.print("Std deviation: "); Serial.println(angular_std, 4);
  Serial.print("95th percentile: "); Serial.println(angular_p95, 4);
  
  Serial.println("\n--- IN DEGREES/SECOND ---");
  Serial.print("Range: ±"); Serial.print(angular_range * 57.3 / 2, 2); Serial.println("°/s");
  Serial.print("Mean absolute: "); Serial.print(angular_abs_mean * 57.3, 2); Serial.println("°/s");
  Serial.print("95th percentile: "); Serial.print(angular_p95 * 57.3, 2); Serial.println("°/s");
  
  Serial.println("\n=== THRESHOLD RECOMMENDATIONS ===");
  
  // Conservative: 2x the 95th percentile
  float conservative = angular_p95 * 2.0;
  // Moderate: 1.5x the 95th percentile  
  float moderate = angular_p95 * 1.5;
  // Aggressive: 1.2x the 95th percentile
  float aggressive = angular_p95 * 1.2;
  
  Serial.print("Conservative (2x 95%): "); Serial.print(conservative, 4);
  Serial.print(" rad/s ("); Serial.print(conservative * 57.3, 1); Serial.println("°/s)");
  
  Serial.print("Moderate (1.5x 95%):   "); Serial.print(moderate, 4);
  Serial.print(" rad/s ("); Serial.print(moderate * 57.3, 1); Serial.println("°/s)");
  
  Serial.print("Aggressive (1.2x 95%): "); Serial.print(aggressive, 4);
  Serial.print(" rad/s ("); Serial.print(aggressive * 57.3, 1); Serial.println("°/s)");
  
  Serial.println("\n=== ARDUINO CODE TO USE ===");
  Serial.print("if (fabs(vel_angular) < "); Serial.print(moderate, 3); Serial.println(") {");
  Serial.println("  vel_angular = 0.0;");
  Serial.println("}");
  
  Serial.println("\n=== QUALITY ASSESSMENT ===");
  if (angular_p95 < 0.005) {
    Serial.println("✅ EXCELLENT: Very low noise sensor");
  } else if (angular_p95 < 0.02) {
    Serial.println("✅ GOOD: Normal noise levels");
  } else if (angular_p95 < 0.05) {
    Serial.println("⚠️ MODERATE: Higher than ideal noise");
    Serial.println("   Consider: better mounting, EMI shielding");
  } else {
    Serial.println("❌ HIGH: Very noisy sensor");
    Serial.println("   Consider: different IMU, better power supply");
  }
}
