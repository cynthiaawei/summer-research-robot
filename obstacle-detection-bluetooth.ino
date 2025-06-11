
#include <ArduinoBLE.h>

//=== MOTOR PIN DEFINITIONS ===//
#define Motor1_Speed A3 // PWM1 (Front Motor)
#define Motor1_Dir   3  // DIR1
#define Motor2_Speed A2 // PWM2 (Back Left Motor)
#define Motor2_Dir   2  // DIR2
#define Motor3_Speed A1 // PWM2 (Back Right Motor)
#define Motor3_Dir   4  // DIR2

//=== ULTRASONIC SENSOR===///
const int interruptPin1 = 8; // For outputPin1
const int interruptPin2 = 7; // For outputPin2
const int interruptPin3 = 6; // For outputPin3

volatile bool triggered1 = false;
volatile bool triggered2 = false;
volatile bool triggered3 = false;

void onSignal1() { triggered1 = true; }
void onSignal2() { triggered2 = true; }
void onSignal3() { triggered3 = true; }


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
bool interruptRequested = false; // Flag for interrupting current movement

// Buffer to hold the last received BLE string (up to 128 bytes)
static char bleBuffer[128] = {0};

// BLE service & characteristic
BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic commandChar(
  "19B10011-E8F2-537E-4F6C-D104768A1214",
  BLEWrite | BLENotify,
  128
);

//=== HELPER: Ramp motor speeds smoothly ===//
void changeSpeedSmooth(int curSpeed1, int newSpeed1,
                       int curSpeed2, int newSpeed2,
                       int curSpeed3, int newSpeed3) {
  int i = curSpeed1, j = curSpeed2, k = curSpeed3;
  interruptRequested = false;

  while ((i != newSpeed1 || j != newSpeed2 || k != newSpeed3) && !interruptRequested) {
    // Check for interruption during ramping
    if (commandChar.written()) {
      interruptRequested = true;
      break;
    }

    if (i < newSpeed1) i++;
    else if (i > newSpeed1) i--;
    if (j < newSpeed2) j++;
    else if (j > newSpeed2) j--;
    if (k < newSpeed3) k++;
    else if (k > newSpeed3) k--;

    analogWrite(Motor1_Speed, i);
    analogWrite(Motor2_Speed, j);
    analogWrite(Motor3_Speed, k);
    delay(5); // Reduced from 10ms for faster response
  }

  if (!interruptRequested) {
    gCurSpeed1 = newSpeed1;
    gCurSpeed2 = newSpeed2;
    gCurSpeed3 = newSpeed3;
  }
}

void stopNoTime(){
  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0);
}
bool interruptHandler(){
  if (triggered1) {
    triggered1 = false;
    //Serial.println("Interrupt from sensor 1!");
    // Put your reaction here
    stopNoTime();
    return true;
  }
  if (triggered2) {
    triggered2 = false;
   // Serial.println("Interrupt from sensor 2!");
    // Put your reaction here
    stopNoTime();
    return true;
  }
  if (triggered3) {
    triggered3 = false;
   // Serial.println("Interrupt from sensor 3!");
    // Put your reaction here
    stopNoTime();
    return true;
  }
  return false;
}

//=== IMMEDIATE MOVEMENT FUNCTIONS FOR KEYBOARD CONTROL ===//
void startForward() {
  Serial.println("Starting forward movement");
  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  // Set speeds immediately without ramping for responsiveness
  analogWrite(Motor1_Speed, 0);
  analogWrite(Motor2_Speed, gSliderSpeed);
  analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

  gCurSpeed1 = 0;
  gCurSpeed2 = gSliderSpeed;
  gCurSpeed3 = gSliderSpeed + motor3_compensate;
}

void startBackward() {
  Serial.println("Starting backward movement");
  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  analogWrite(Motor1_Speed, 0);
  analogWrite(Motor2_Speed, gSliderSpeed);
  analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

  gCurSpeed1 = 0;
  gCurSpeed2 = gSliderSpeed;
  gCurSpeed3 = gSliderSpeed + motor3_compensate;
}

void startTurnLeft() {
  Serial.println("Starting left turn");
  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  analogWrite(Motor1_Speed, gSliderSpeed);
  analogWrite(Motor2_Speed, gSliderSpeed);
  analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

  gCurSpeed1 = gSliderSpeed;
  gCurSpeed2 = gSliderSpeed;
  gCurSpeed3 = gSliderSpeed + motor3_compensate;
}

void startTurnRight() {
  Serial.println("Starting right turn");
  curDir1 = LOW;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  analogWrite(Motor1_Speed, gSliderSpeed);
  analogWrite(Motor2_Speed, gSliderSpeed);
  analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

  gCurSpeed1 = gSliderSpeed;
  gCurSpeed2 = gSliderSpeed;
  gCurSpeed3 = gSliderSpeed + motor3_compensate;
}

void immediateStop() {
  Serial.println("Immediate stop");
  analogWrite(Motor1_Speed, 0);
  analogWrite(Motor2_Speed, 0);
  analogWrite(Motor3_Speed, 0);

  gCurSpeed1 = 0;
  gCurSpeed2 = 0;
  gCurSpeed3 = 0;
}

//=== ORIGINAL TIMED MOVEMENT FUNCTIONS FOR SPEECH/TEXT CONTROL ===//
void goForwards(int speed, int time_ms) {
  triggered1 = triggered2 = triggered3 = false;

  // Serial.print("forwards ");
  // Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  if (interruptRequested) return; // Exit early if interrupted

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms) {
    if (commandChar.written()) {
      Serial.println("Movement interrupted by new command");
      break;
    }
    if(interruptHandler()) break; 

    delay(10); // Small delay to prevent excessive polling
  }
}

void goBackwards(int speed, int time_ms) {
  triggered1 = triggered2 = triggered3 = false;
  // Serial.print("backwards ");
  // Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  if (interruptRequested) return;

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms) {
    if (commandChar.written()) {
      Serial.println("Movement interrupted by new command");
      break;
    }
    if(interruptHandler()) break; 
    delay(10);
  }
}

void stopMotors(int time_ms) {
  // Serial.print("stop ");
  // Serial.println(time_ms);

  // Ramp down smoothly to zero
  changeSpeedSmooth(gCurSpeed1, 0,
                    gCurSpeed2, 0,
                    gCurSpeed3, 0);

  if (time_ms >= 0) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)time_ms) {
      if (commandChar.written()) {
        Serial.println("Stop interrupted by new command");
        break;
      }
      delay(10);
    }
  } else {
    permStop = true;
  }
}

void turnRight(int speed, int time_ms) {
  triggered1 = triggered2 = triggered3 = false;
  // Serial.print("turnRight ");
  // Serial.println(time_ms);

  curDir1 = LOW;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  if (interruptRequested) return;

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms) {
    if (commandChar.written()) {
      Serial.println("Turn right interrupted by new command");
      break;
    }
    if(interruptHandler()) break; 
    delay(10);
  }
}

void turnLeft(int speed, int time_ms) {
  triggered1 = triggered2 = triggered3 = false;
  // Serial.print("turnLeft ");
  // Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed,
                    gCurSpeed2, speed,
                    gCurSpeed3, speed + motor3_compensate);

  if (interruptRequested) return;

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms) {
    if (commandChar.written()) {
      Serial.println("Turn left interrupted by new command");
      break;
    }
    if(interruptHandler()) break; 
    delay(10);
  }
}

void moveRight(int speed, int time_ms) {
  triggered1 = triggered2 = triggered3 = false;
  // Serial.print("moveRight ");
  // Serial.println(time_ms);

  curDir1 = LOW;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, 0,
                    gCurSpeed3, speed + motor3_compensate);

  if (interruptRequested) return;

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms) {
    if (commandChar.written()) {
      Serial.println("Move right interrupted by new command");
      break;
    }
   if(interruptHandler()) break; 
    delay(10);
  }
}

void moveLeft(int speed, int time_ms) {
  triggered1 = triggered2 = triggered3 = false;
  // Serial.print("moveLeft ");
  // Serial.println(time_ms);

  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, speed * 1.5,
                    gCurSpeed2, speed,
                    gCurSpeed3, 0);

  if (interruptRequested) return;

  unsigned long start = millis();
  while (millis() - start < (unsigned long)time_ms) {
    if (commandChar.written()) {
      Serial.println("Move left interrupted by new command");
      break;
    }
  if(interruptHandler()) break; 
    delay(10);
  }
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

//=== COMMAND PROCESSING FUNCTIONS ===//
void processImmediateCommand(String command) {
  // Serial.print("Immediate command: ");
  // Serial.println(command);

  if (command == "forward") {
    startForward();
  } else if (command == "backward") {
    startBackward();
  } else if (command == "left") {
    startTurnLeft();
  } else if (command == "right") {
    startTurnRight();
  } else if (command == "stop") {
    immediateStop();
  } else {
    // Serial.print("Unknown immediate command: ");
    // Serial.println(command);
  }
}

void processCommand(String command, int time_ms) {
  // Serial.print("Processing: ");
  // Serial.print(command);
  // Serial.print(" for ");
  // Serial.print(time_ms);
  // Serial.println("ms");

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
  } else {
    Serial.print("Unknown timed command: ");
    Serial.println(command);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait up to 5 seconds for Serial

  Serial.println("=== Robot BLE Controller Starting ===");

  // Configure motor pins
  pinMode(Motor1_Speed, OUTPUT);
  pinMode(Motor1_Dir, OUTPUT);
  pinMode(Motor2_Speed, OUTPUT);
  pinMode(Motor2_Dir, OUTPUT);
  pinMode(Motor3_Speed, OUTPUT);
  pinMode(Motor3_Dir, OUTPUT);

  // Initialize motors to stopped state
  analogWrite(Motor1_Speed, 0);
  analogWrite(Motor2_Speed, 0);
  analogWrite(Motor3_Speed, 0);
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  // Interrupts
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  pinMode(interruptPin3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(interruptPin1), onSignal1, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), onSignal2, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin3), onSignal3, RISING);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("ERROR: Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("RobotBLE");
  robotService.addCharacteristic(commandChar);
  BLE.addService(robotService);
  commandChar.writeValue("idle");
  BLE.setAdvertisedService(robotService);
  BLE.setDeviceName("RobotBLE");
  BLE.setAdvertisingInterval(100); // 100 ms interval
  BLE.setConnectable(true);

  if (!BLE.advertise()) {
    Serial.println("ERROR: Failed to start advertising!");
    while (1);
  }
  Serial.println("BLE Robot Ready! Advertising as RobotBLE");
}



void loop() {
  // Poll BLE stack
  BLE.poll();

  // Check if a central has connected
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // While connected, handle incoming writes
    while (central.connected()) {
      if (commandChar.written()) {
        // Read the BLE data into bleBuffer
        int len = commandChar.valueLength();
        if (len > 127) len = 127; // Prevent overflow
        commandChar.readValue((uint8_t*)bleBuffer, len);
        bleBuffer[len] = '\0';

        Serial.print("Command received over BLE: ");
        Serial.println(bleBuffer);

        // Convert to String and split into words
        String incoming = String(bleBuffer);
        incoming.trim();
        static String words[20];
        for (int i = 0; i < 20; i++) {
          words[i] = "";
        }
        splitString(incoming, words);

        // If only a single token, treat as immediate command
        if (words[0] != "" && words[1] == "") {
          processImmediateCommand(words[0]);
        }
        else {
          // Otherwise, parse pairs: <command> <time_ms>
          for (int i = 0; i + 1 < 20; i += 2) {
            if (words[i] == "" || words[i + 1] == "") break;
            String subCmd = words[i];
            int duration = words[i + 1].toInt();
            interruptRequested = false;
            processCommand(subCmd, duration);
            if (interruptRequested) {
              Serial.println("Previous command interrupted by new BLE data");
              break;
            }
          }
        }

        bleBuffer[0] = '\0';         // Clear the buffer
        permStop = false;           // Allow movement
      }

      // If permStop is set externally, ensure motors are off
      if (permStop) {
        analogWrite(Motor1_Speed, 0);
        analogWrite(Motor2_Speed, 0);
        analogWrite(Motor3_Speed, 0);
      }
      delay(10); // Avoid hammering BLE stack too quickly
    }

    // Disconnected from central
    Serial.println("Disconnected from central.");
    bleBuffer[0] = '\0';
    permStop = true;
    interruptRequested = false;
  }
}
