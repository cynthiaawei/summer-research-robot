
#include <ArduinoBLE.h>

//=== MOTOR PIN DEFINITIONS ===//
#define Motor1_Speed A3 // PWM1 (Front Motor)
#define Motor1_Dir   3  // DIR1
#define Motor2_Speed A2 // PWM2 (Back Left Motor)
#define Motor2_Dir   2  // DIR2
#define Motor3_Speed A1 // PWM2 (Back Right Motor)
#define Motor3_Dir   4  // DIR2

//=== GLOBAL STATE ===//
int gCurSpeed1 = 0;
int gCurSpeed2 = 0;
int gCurSpeed3 = 0;
int gSliderSpeed = 25;

bool curDir1 = HIGH; // HIGH = forward
bool curDir2 = HIGH; // HIGH = forward
bool curDir3 = HIGH; // HIGH = forward

int motor3_compensate = 15;
bool permStop = true;

// Buffer to hold the last received BLE string (up to 128 bytes)
static char bleBuffer[128] = {0};

// BLE service & characteristic
BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Custom service UUID
BLECharacteristic commandChar(
  "19B10011-E8F2-537E-4F6C-D104768A1214",
  BLEWrite | BLENotify,
  128  // Up to 128 bytes per write
);

//=== HELPER: Ramp motor speeds smoothly ===//
void changeSpeedSmooth(int curSpeed1, int newSpeed1,
                       int curSpeed2, int newSpeed2,
                       int curSpeed3, int newSpeed3) {
  int i = curSpeed1, j = curSpeed2, k = curSpeed3;

  while (i != newSpeed1 || j != newSpeed2 || k != newSpeed3) {
    if (i < newSpeed1) i++;
    else if (i > newSpeed1) i--;
    if (j < newSpeed2) j++;
    else if (j > newSpeed2) j--;
    if (k < newSpeed3) k++;
    else if (k > newSpeed3) k--;

    analogWrite(Motor1_Speed, i);
    analogWrite(Motor2_Speed, j);
    analogWrite(Motor3_Speed, k);
    delay(10);
  }

  delay(10);
  gCurSpeed1 = newSpeed1;
  gCurSpeed2 = newSpeed2;
  gCurSpeed3 = newSpeed3;
}

//=== MOVEMENT FUNCTIONS ===//
void goForwards(int speed, int time_ms) {
  Serial.print("forwards ");
  Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms && !commandChar.written()) { /* spin */ }
}

void goBackwards(int speed, int time_ms) {
  Serial.print("backwards ");
  Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms && !commandChar.written()) { /* spin */ }
}

void stopMotors(int time_ms) {
  Serial.print("stop ");
  Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, 0,
                    gCurSpeed3, 0);

  if (time_ms >= 0) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)time_ms && !commandChar.written()) { /* spin */ }
  } else {
    permStop = true;
  }
}

void turnRight(int speed, int time_ms) {
  Serial.print("turnRight ");
  Serial.println(time_ms);

  curDir1 = LOW;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms && !commandChar.written()) { /* spin */ }
}

void turnLeft(int speed, int time_ms) {
  Serial.print("turnLeft ");
  Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms && !commandChar.written()) { /* spin */ }
}

void moveRight(int speed, int time_ms) {
  Serial.print("moveRight ");
  Serial.println(time_ms);

  curDir1 = LOW;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, 0,
                    gCurSpeed3, speed + motor3_compensate);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms && !commandChar.written()) { /* spin */ }
}

void moveLeft(int speed, int time_ms) {
  Serial.print("moveLeft ");
  Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, speed,
                    gCurSpeed3, 0);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms && !commandChar.written()) { /* spin */ }
}

//=== STRING SPLITTING HELPERS ===//
bool isEnd = false;

String nextWord(String &input) {
  input.trim();
  int index = input.indexOf(' ');
  if (index == -1) {
    isEnd = true;
    String word = input;
    input = "";
    return word;
  }
  String word = input.substring(0, index);
  input = input.substring(index + 1);
  return word;
}

void splitString(String input, String words[]) {
  int i = 0;
  isEnd = false;
  while (!isEnd && i < 20) {
    words[i++] = nextWord(input);
  }
}

void processCommand(String command, int time_ms) {
  if (command == "forward") {
    goForwards(gSliderSpeed, time_ms);
  } else if (command == "backward") {
    goBackwards(gSliderSpeed, time_ms);
  } else if (command == "turnRight") {
    turnRight(gSliderSpeed, time_ms);
  } else if (command == "turnLeft") {
    turnLeft(gSliderSpeed, time_ms);
  } else if (command == "moveRight") {
    moveRight(gSliderSpeed, time_ms);
  } else if (command == "moveLeft") {
    moveLeft(gSliderSpeed, time_ms);
  } else if (command == "stop") {
    stopMotors(time_ms);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait up to 5 seconds for Serial

  // Configure motor pins
  pinMode(Motor1_Speed, OUTPUT);
  pinMode(Motor1_Dir, OUTPUT);
  pinMode(Motor2_Speed, OUTPUT);
  pinMode(Motor2_Dir, OUTPUT);
  pinMode(Motor3_Speed, OUTPUT);
  pinMode(Motor3_Dir, OUTPUT);

  // Initialize motors
  analogWrite(Motor1_Speed, gCurSpeed1);
  digitalWrite(Motor1_Dir, curDir1);
  analogWrite(Motor2_Speed, gCurSpeed2);
  digitalWrite(Motor2_Dir, curDir2);
  analogWrite(Motor3_Speed, gCurSpeed3);
  digitalWrite(Motor3_Dir, curDir3);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("ERROR: Starting BLE failed!");
    // Blink LED to indicate failure (if available)
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }

  // Set device name and service
  BLE.setLocalName("RobotBLE");
  robotService.addCharacteristic(commandChar);
  BLE.addService(robotService);
  commandChar.writeValue("idle");
  BLE.setAdvertisedService(robotService);
  BLE.setDeviceName("RobotBLE");

  // Configure advertising
  BLE.setAdvertisingInterval(100); // 100ms interval
  BLE.setConnectable(true);

  // Start advertising
  if (!BLE.advertise()) {
    Serial.println("ERROR: Failed to start advertising!");
    // Blink LED faster to indicate advertising failure
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  Serial.println("BLE Robot Ready! Advertising as RobotBLE");
}

void loop() {
  // Poll BLE stack
  BLE.poll();

  // Check for central connection
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      if (commandChar.written()) {
        // Read BLE data
        int len = commandChar.valueLength();
        if (len > 127) len = 127; // Prevent overflow
        commandChar.readValue((uint8_t*)bleBuffer, len);
        bleBuffer[len] = '\0';

        Serial.print("Command received over BLE: ");
        Serial.println(bleBuffer);

        String incoming = String(bleBuffer);
        incoming.trim();

        static String words[20];
        for (int i = 0; i < 20; i++) {
          words[i] = "";
        }
        splitString(incoming, words);

        for (int i = 0; i + 1 < 20; i += 2) {
          if (words[i] == "" || words[i + 1] == "") break;
          String subCmd = words[i];
          int duration = words[i + 1].toInt();
          processCommand(subCmd, duration);
        }

        permStop = false;
      }

      if (permStop) {
        changeSpeedSmooth(gCurSpeed1, 0,
                          gCurSpeed2, 0,
                          gCurSpeed3, 0);
      }
      delay(10); // Avoid hammering BLE stack
    }

    Serial.println("Disconnected from central.");
    bleBuffer[0] = '\0';
    permStop = true;
  }
}
