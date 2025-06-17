
// #include <ArduinoBLE.h>

// //=== MOTOR PIN DEFINITIONS ===//
// #define Motor1_Speed A3 // PWM1 (Front Motor)
// #define Motor1_Dir   3  // DIR1
// #define Motor2_Speed A2 // PWM2 (Back Left Motor)
// #define Motor2_Dir   2  // DIR2
// #define Motor3_Speed A1 // PWM2 (Back Right Motor)
// #define Motor3_Dir   4  // DIR2

// //=== ULTRASONIC SENSOR===///
// const int interruptPin1 = 8; // For outputPin1
// const int interruptPin2 = 7; // For outputPin2
// const int interruptPin3 = 6; // For outputPin3

// volatile bool triggered1 = false;
// volatile bool triggered2 = false;
// volatile bool triggered3 = false;

// void onSignal1() { triggered1 = true; }
// void onSignal2() { triggered2 = true; }
// void onSignal3() { triggered3 = true; }


// //=== GLOBAL STATE ===//
// int gCurSpeed1 = 0;
// int gCurSpeed2 = 0;
// int gCurSpeed3 = 0;
// int gSliderSpeed = 25;

// bool curDir1 = HIGH; // HIGH = forward
// bool curDir2 = HIGH; // HIGH = forward
// bool curDir3 = HIGH; // HIGH = forward

// int motor3_compensate = 15;
// bool permStop = true;
// bool interruptRequested = false; // Flag for interrupting current movement

// // Buffer to hold the last received BLE string (up to 128 bytes)
// static char bleBuffer[128] = {0};

// // BLE service & characteristic
// BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");
// BLECharacteristic commandChar(
//   "19B10011-E8F2-537E-4F6C-D104768A1214",
//   BLEWrite | BLENotify,  128);

// //=== HELPER: Ramp motor speeds smoothly ===//
// void changeSpeedSmooth(int curSpeed1, int newSpeed1,
//                        int curSpeed2, int newSpeed2,
//                        int curSpeed3, int newSpeed3) {
//   int i = curSpeed1, j = curSpeed2, k = curSpeed3;
//   interruptRequested = false;

//   while ((i != newSpeed1 || j != newSpeed2 || k != newSpeed3) && !interruptRequested) {
//     // Check for interruption during ramping
//     if (commandChar.written()) {
//       interruptRequested = true;
//       break;
//     }

//     if (i < newSpeed1) i++;
//     else if (i > newSpeed1) i--;
//     if (j < newSpeed2) j++;
//     else if (j > newSpeed2) j--;
//     if (k < newSpeed3) k++;
//     else if (k > newSpeed3) k--;

//     analogWrite(Motor1_Speed, i);
//     analogWrite(Motor2_Speed, j);
//     analogWrite(Motor3_Speed, k);
//     delay(5); // Reduced from 10ms for faster response
//   }

//   if (!interruptRequested) {
//     gCurSpeed1 = newSpeed1;
//     gCurSpeed2 = newSpeed2;
//     gCurSpeed3 = newSpeed3;
//   }
// }

// void stopNoTime(){
//   curDir1 = HIGH;
//   curDir2 = HIGH;
//   curDir3 = HIGH;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0);
// }
// bool interruptHandler(){
//   if (triggered1) {
//     triggered1 = false;
//     //Serial.println("Interrupt from sensor 1!");
//     // Put your reaction here
//     stopNoTime();
//     return true;
//   }
//   if (triggered2) {
//     triggered2 = false;
//    // Serial.println("Interrupt from sensor 2!");
//     // Put your reaction here
//     stopNoTime();
//     return true;
//   }
//   if (triggered3) {
//     triggered3 = false;
//    // Serial.println("Interrupt from sensor 3!");
//     // Put your reaction here
//     stopNoTime();
//     return true;
//   }
//   return false;
// }

// //=== IMMEDIATE MOVEMENT FUNCTIONS FOR KEYBOARD CONTROL ===//
// void startForward() {
//   Serial.println("Starting forward movement");
//   curDir1 = HIGH;
//   curDir2 = HIGH;
//   curDir3 = LOW;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   // Set speeds immediately without ramping for responsiveness
//   analogWrite(Motor1_Speed, 0);
//   analogWrite(Motor2_Speed, gSliderSpeed);
//   analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

//   gCurSpeed1 = 0;
//   gCurSpeed2 = gSliderSpeed;
//   gCurSpeed3 = gSliderSpeed + motor3_compensate;
// }

// void startBackward() {
//   Serial.println("Starting backward movement");
//   curDir1 = HIGH;
//   curDir2 = LOW;
//   curDir3 = HIGH;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   analogWrite(Motor1_Speed, 0);
//   analogWrite(Motor2_Speed, gSliderSpeed);
//   analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

//   gCurSpeed1 = 0;
//   gCurSpeed2 = gSliderSpeed;
//   gCurSpeed3 = gSliderSpeed + motor3_compensate;
// }

// void startTurnLeft() {
//   Serial.println("Starting left turn");
//   curDir1 = HIGH;
//   curDir2 = LOW;
//   curDir3 = LOW;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   analogWrite(Motor1_Speed, gSliderSpeed);
//   analogWrite(Motor2_Speed, gSliderSpeed);
//   analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

//   gCurSpeed1 = gSliderSpeed;
//   gCurSpeed2 = gSliderSpeed;
//   gCurSpeed3 = gSliderSpeed + motor3_compensate;
// }

// void startTurnRight() {
//   Serial.println("Starting right turn");
//   curDir1 = LOW;
//   curDir2 = HIGH;
//   curDir3 = HIGH;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   analogWrite(Motor1_Speed, gSliderSpeed);
//   analogWrite(Motor2_Speed, gSliderSpeed);
//   analogWrite(Motor3_Speed, gSliderSpeed + motor3_compensate);

//   gCurSpeed1 = gSliderSpeed;
//   gCurSpeed2 = gSliderSpeed;
//   gCurSpeed3 = gSliderSpeed + motor3_compensate;
// }

// void immediateStop() {
//   Serial.println("Immediate stop");
//   analogWrite(Motor1_Speed, 0);
//   analogWrite(Motor2_Speed, 0);
//   analogWrite(Motor3_Speed, 0);

//   gCurSpeed1 = 0;
//   gCurSpeed2 = 0;
//   gCurSpeed3 = 0;
// }

// //=== ORIGINAL TIMED MOVEMENT FUNCTIONS FOR SPEECH/TEXT CONTROL ===//
// void goForwards(int speed, int time_ms) {
//   triggered1 = triggered2 = triggered3 = false;

//   // Serial.print("forwards ");
//   // Serial.println(time_ms);

//   curDir1 = HIGH;
//   curDir2 = HIGH;
//   curDir3 = LOW;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   changeSpeedSmooth(gCurSpeed1, 0,
//                     gCurSpeed2, speed,
//                     gCurSpeed3, speed + motor3_compensate);

//   if (interruptRequested) return; // Exit early if interrupted

//   unsigned long start = millis();
//   while (millis() - start < (unsigned long)time_ms) {
//     if (commandChar.written()) {
//       Serial.println("Movement interrupted by new command");
//       break;
//     }
//     if(interruptHandler()) break; 

//     delay(10); // Small delay to prevent excessive polling
//   }
// }

// void goBackwards(int speed, int time_ms) {
//   triggered1 = triggered2 = triggered3 = false;
//   // Serial.print("backwards ");
//   // Serial.println(time_ms);

//   curDir1 = HIGH;
//   curDir2 = LOW;
//   curDir3 = HIGH;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   changeSpeedSmooth(gCurSpeed1, 0,
//                     gCurSpeed2, speed,
//                     gCurSpeed3, speed + motor3_compensate);

//   if (interruptRequested) return;

//   unsigned long start = millis();
//   while (millis() - start < (unsigned long)time_ms) {
//     if (commandChar.written()) {
//       Serial.println("Movement interrupted by new command");
//       break;
//     }
//     if(interruptHandler()) break; 
//     delay(10);
//   }
// }

// void stopMotors(int time_ms) {
//   // Serial.print("stop ");
//   // Serial.println(time_ms);

//   // Ramp down smoothly to zero
//   changeSpeedSmooth(gCurSpeed1, 0,
//                     gCurSpeed2, 0,
//                     gCurSpeed3, 0);

//   if (time_ms >= 0) {
//     unsigned long start = millis();
//     while (millis() - start < (unsigned long)time_ms) {
//       if (commandChar.written()) {
//         Serial.println("Stop interrupted by new command");
//         break;
//       }
//       delay(10);
//     }
//   } else {
//     permStop = true;
//   }
// }

// void turnRight(int speed, int time_ms) {
//   triggered1 = triggered2 = triggered3 = false;
//   // Serial.print("turnRight ");
//   // Serial.println(time_ms);

//   curDir1 = LOW;
//   curDir2 = HIGH;
//   curDir3 = HIGH;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   changeSpeedSmooth(gCurSpeed1, speed,
//                     gCurSpeed2, speed,
//                     gCurSpeed3, speed + motor3_compensate);

//   if (interruptRequested) return;

//   unsigned long start = millis();
//   while (millis() - start < (unsigned long)time_ms) {
//     if (commandChar.written()) {
//       Serial.println("Turn right interrupted by new command");
//       break;
//     }
//     if(interruptHandler()) break; 
//     delay(10);
//   }
// }

// void turnLeft(int speed, int time_ms) {
//   triggered1 = triggered2 = triggered3 = false;
//   // Serial.print("turnLeft ");
//   // Serial.println(time_ms);

//   curDir1 = HIGH;
//   curDir2 = LOW;
//   curDir3 = LOW;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   changeSpeedSmooth(gCurSpeed1, speed,
//                     gCurSpeed2, speed,
//                     gCurSpeed3, speed + motor3_compensate);

//   if (interruptRequested) return;

//   unsigned long start = millis();
//   while (millis() - start < (unsigned long)time_ms) {
//     if (commandChar.written()) {
//       Serial.println("Turn left interrupted by new command");
//       break;
//     }
//     if(interruptHandler()) break; 
//     delay(10);
//   }
// }

// void moveRight(int speed, int time_ms) {
//   triggered1 = triggered2 = triggered3 = false;
//   // Serial.print("moveRight ");
//   // Serial.println(time_ms);

//   curDir1 = LOW;
//   curDir2 = HIGH;
//   curDir3 = LOW;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   changeSpeedSmooth(gCurSpeed1, speed * 1.5,
//                     gCurSpeed2, 0,
//                     gCurSpeed3, speed + motor3_compensate);

//   if (interruptRequested) return;

//   unsigned long start = millis();
//   while (millis() - start < (unsigned long)time_ms) {
//     if (commandChar.written()) {
//       Serial.println("Move right interrupted by new command");
//       break;
//     }
//    if(interruptHandler()) break; 
//     delay(10);
//   }
// }

// void moveLeft(int speed, int time_ms) {
//   triggered1 = triggered2 = triggered3 = false;
//   // Serial.print("moveLeft ");
//   // Serial.println(time_ms);

//   curDir1 = HIGH;
//   curDir2 = HIGH;
//   curDir3 = LOW;
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   changeSpeedSmooth(gCurSpeed1, speed * 1.5,
//                     gCurSpeed2, speed,
//                     gCurSpeed3, 0);

//   if (interruptRequested) return;

//   unsigned long start = millis();
//   while (millis() - start < (unsigned long)time_ms) {
//     if (commandChar.written()) {
//       Serial.println("Move left interrupted by new command");
//       break;
//     }
//   if(interruptHandler()) break; 
//     delay(10);
//   }
// }

// //=== STRING SPLITTING HELPERS ===//
// bool isEnd = false;

// String nextWord(String &input) {
//   input.trim();
//   int index = input.indexOf(' ');
//   if (index == -1) {
//     isEnd = true;
//     String word = input;
//     input = "";
//     return word;
//   }
//   String word = input.substring(0, index);
//   input = input.substring(index + 1);
//   return word;
// }

// void splitString(String input, String words[]) {
//   int i = 0;
//   isEnd = false;
//   while (!isEnd && i < 20) {
//     words[i++] = nextWord(input);
//   }
// }

// //=== COMMAND PROCESSING FUNCTIONS ===//
// void processImmediateCommand(String command) {
//   // Serial.print("Immediate command: ");
//   // Serial.println(command);

//   if (command == "forward") {
//     startForward();
//   } else if (command == "backward") {
//     startBackward();
//   } else if (command == "left") {
//     startTurnLeft();
//   } else if (command == "right") {
//     startTurnRight();
//   } else if (command == "stop") {
//     immediateStop();
//   } else {
//     // Serial.print("Unknown immediate command: ");
//     // Serial.println(command);
//   }
// }

// void processCommand(String command, int time_ms) {
//   // Serial.print("Processing: ");
//   // Serial.print(command);
//   // Serial.print(" for ");
//   // Serial.print(time_ms);
//   // Serial.println("ms");

//   if (command == "forward") {
//     goForwards(gSliderSpeed, time_ms);
//   } else if (command == "backward") {
//     goBackwards(gSliderSpeed, time_ms);
//   } else if (command == "turnRight") {
//     turnRight(gSliderSpeed, time_ms);
//   } else if (command == "turnLeft") {
//     turnLeft(gSliderSpeed, time_ms);
//   } else if (command == "moveRight") {
//     moveRight(gSliderSpeed, time_ms);
//   } else if (command == "moveLeft") {
//     moveLeft(gSliderSpeed, time_ms);
//   } else if (command == "stop") {
//     stopMotors(time_ms);
//   } else {
//     Serial.print("Unknown timed command: ");
//     Serial.println(command);
//   }
// }

// void setup() {
//   // Initialize serial for debugging
//   Serial.begin(9600);
//   while (!Serial && millis() < 5000); // Wait up to 5 seconds for Serial

//   Serial.println("=== Robot BLE Controller Starting ===");

//   // Configure motor pins
//   pinMode(Motor1_Speed, OUTPUT);
//   pinMode(Motor1_Dir, OUTPUT);
//   pinMode(Motor2_Speed, OUTPUT);
//   pinMode(Motor2_Dir, OUTPUT);
//   pinMode(Motor3_Speed, OUTPUT);
//   pinMode(Motor3_Dir, OUTPUT);

//   // Initialize motors to stopped state
//   analogWrite(Motor1_Speed, 0);
//   analogWrite(Motor2_Speed, 0);
//   analogWrite(Motor3_Speed, 0);
//   digitalWrite(Motor1_Dir, curDir1);
//   digitalWrite(Motor2_Dir, curDir2);
//   digitalWrite(Motor3_Dir, curDir3);

//   // Interrupts
//   pinMode(interruptPin1, INPUT_PULLUP);
//   pinMode(interruptPin2, INPUT_PULLUP);
//   pinMode(interruptPin3, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(interruptPin1), onSignal1, RISING);
//   attachInterrupt(digitalPinToInterrupt(interruptPin2), onSignal2, RISING);
//   attachInterrupt(digitalPinToInterrupt(interruptPin3), onSignal3, RISING);

//   // Initialize BLE
//   if (!BLE.begin()) {
//     Serial.println("ERROR: Starting BLE failed!");
//     while (1);
//   }

//   BLE.setLocalName("RobotBLE");
//   robotService.addCharacteristic(commandChar);
//   BLE.addService(robotService);
//   commandChar.writeValue("idle");
//   BLE.setAdvertisedService(robotService);
//   BLE.setDeviceName("RobotBLE");
//   BLE.setAdvertisingInterval(100); // 100 ms interval
//   BLE.setConnectable(true);

//   if (!BLE.advertise()) {
//     Serial.println("ERROR: Failed to start advertising!");
//     while (1);
//   }
//   Serial.println("BLE Robot Ready! Advertising as RobotBLE");
// }



// void loop() {
//   // Poll BLE stack
//   BLE.poll();

//   // Check if a central has connected
//   BLEDevice central = BLE.central();
//   if (central) {
//     Serial.print("Connected to central: ");
//     Serial.println(central.address());

//     // While connected, handle incoming writes
//     while (central.connected()) {
//       if (commandChar.written()) {
//         // Read the BLE data into bleBuffer
//         int len = commandChar.valueLength();
//         if (len > 127) len = 127; // Prevent overflow
//         commandChar.readValue((uint8_t*)bleBuffer, len);
//         bleBuffer[len] = '\0';

//         Serial.print("Command received over BLE: ");
//         Serial.println(bleBuffer);

//         // Convert to String and split into words
//         String incoming = String(bleBuffer);
//         incoming.trim();
//         static String words[20];
//         for (int i = 0; i < 20; i++) {
//           words[i] = "";
//         }
//         splitString(incoming, words);

//         // If only a single token, treat as immediate command
//         if (words[0] != "" && words[1] == "") {
//           processImmediateCommand(words[0]);
//         }
//         else {
//           // Otherwise, parse pairs: <command> <time_ms>
//           for (int i = 0; i + 1 < 20; i += 2) {
//             if (words[i] == "" || words[i + 1] == "") break;
//             String subCmd = words[i];
//             int duration = words[i + 1].toInt();
//             interruptRequested = false;
//             processCommand(subCmd, duration);
//             if (interruptRequested) {
//               Serial.println("Previous command interrupted by new BLE data");
//               break;
//             }
//           }
//         }

//         bleBuffer[0] = '\0';         // Clear the buffer
//         permStop = false;           // Allow movement
//       }

//       // If permStop is set externally, ensure motors are off
//       if (permStop) {
//         analogWrite(Motor1_Speed, 0);
//         analogWrite(Motor2_Speed, 0);
//         analogWrite(Motor3_Speed, 0);
//       }
//       delay(10); // Avoid hammering BLE stack too quickly
//     }

//     // Disconnected from central
//     Serial.println("Disconnected from central.");
//     bleBuffer[0] = '\0';
//     permStop = true;
//     interruptRequested = false;
//   }
// }
import asyncio
import re
import threading
import keyboard
import subprocess
import time
import platform
import speech_recognition as sr
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
import RPi.GPIO as GPIO

# === Motor Pin Definitions ===
Motor1_Speed = 38  # PWM 1
Motor1_Dir = 40   # Dir 1
Motor2_Speed = 32  # PWM 2
Motor2_Dir = 36    # Dir 2
Motor3_Speed = 16  # PWM 3
Motor3_Dir = 26    # Dir 3

# === Ultrasonic Sensor ===
Echo1 = 31
Echo2 = 29
Echo3 = 22
Trig1 = 11
Trig2 = 13
Trig3 = 15
triggered1 = False
triggered2 = False
triggered3 = False

# === GPIO Setup ===
GPIO.setmode(GPIO.BOARD)
GPIO.setup(Echo1, GPIO.IN)
GPIO.setup(Echo2, GPIO.IN)
GPIO.setup(Echo3, GPIO.IN)
GPIO.setup(Trig1, GPIO.OUT)
GPIO.setup(Trig2, GPIO.OUT)
GPIO.setup(Trig3, GPIO.OUT)
GPIO.setup(Motor1_Speed, GPIO.OUT, initial=0)
GPIO.setup(Motor1_Dir, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(Motor2_Speed, GPIO.OUT, initial=0)
GPIO.setup(Motor2_Dir, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(Motor3_Speed, GPIO.OUT, initial=0)
GPIO.setup(Motor3_Dir, GPIO.OUT, initial=GPIO.HIGH)

# Set PWM frequencies
freq = 1000
Motor1_pwm = GPIO.PWM(Motor1_Speed, freq)
Motor2_pwm = GPIO.PWM(Motor2_Speed, freq)
Motor3_pwm = GPIO.PWM(Motor3_Speed, freq)
Motor1_pwm.start(0)
Motor2_pwm.start(0)
Motor3_pwm.start(0)

# Interrupt handlers
def onSignal1(channel):
    global triggered1
    triggered1 = True

def onSignal2(channel):
    global triggered2
    triggered2 = True

def onSignal3(channel):
    global triggered3
    triggered3 = True

GPIO.add_event_detect(Echo1, GPIO.RISING, callback=onSignal1)
GPIO.add_event_detect(Echo2, GPIO.RISING, callback=onSignal2)
GPIO.add_event_detect(Echo3, GPIO.RISING, callback=onSignal3)

# === Global State ===
gCurSpeed1 = 0
gCurSpeed2 = 0
gCurSpeed3 = 0
gSliderSpeed = 64  # ~25% of 255
motor3_compensate = 15  # ~6% of 255
permStop = True
interruptRequested = False
spd_list = [Motor1_Speed, Motor2_Speed, Motor3_Speed]
dir_list = [Motor1_Dir, Motor2_Dir, Motor3_Dir]
commandCharacter = ""

# === Ultrasonic Trigger ===
def trigger_ultrasonic(trig_pin):
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(trig_pin, GPIO.LOW)

# === Helper: Ramp motor speeds smoothly ===
def changeSpeedSmooth(curSpeed1, newSpeed1, curSpeed2, newSpeed2, curSpeed3, newSpeed3):
    global interruptRequested, gCurSpeed1, gCurSpeed2, gCurSpeed3
    i = curSpeed1
    j = curSpeed2
    k = curSpeed3
    while (i != newSpeed1 or j != newSpeed2 or k != newSpeed3) and not interruptRequested:
        if i < newSpeed1:
            i += 1
        elif i > newSpeed1:
            i -= 1
        if j < newSpeed2:
            j += 1
        elif j > newSpeed2:
            j -= 1
        if k < newSpeed3:
            k += 1
        elif k > newSpeed3:
            k -= 1
        Motor1_pwm.ChangeDutyCycle(i * 100 / 255)
        Motor2_pwm.ChangeDutyCycle(j * 100 / 255)
        Motor3_pwm.ChangeDutyCycle(k * 100 / 255)
        time.sleep(0.005)
    if not interruptRequested:
        gCurSpeed1, gCurSpeed2, gCurSpeed3 = newSpeed1, newSpeed2, newSpeed3

def stopMotors():
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.HIGH))
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)

def interruptHandler():
    global triggered1, triggered2, triggered3
    if triggered1:
        triggered1 = False
        print("Interrupt from sensor 1!")
        stopMotors()
        return True
    if triggered2:
        triggered2 = False
        print("Interrupt from sensor 2!")
        stopMotors()
        return True
    if triggered3:
        triggered3 = False
        print("Interrupt from sensor 3!")
        stopMotors()
        return True
    return False

# === Immediate Movement Functions ===
def startForward():
    print("Starting forward movement")
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.LOW))
    Motor1_pwm.ChangeDutyCycle(0)
    Motor2_pwm.ChangeDutyCycle(gSliderSpeed * 100 / 255)
    Motor3_pwm.ChangeDutyCycle((gSliderSpeed + motor3_compensate) * 100 / 255)
    print(f"Motor2 PWM: {gSliderSpeed * 100 / 255}%, Motor3 PWM: {(gSliderSpeed + motor3_compensate) * 100 / 255}%")
    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    gCurSpeed1 = 0
    gCurSpeed2 = gSliderSpeed
    gCurSpeed3 = gSliderSpeed + motor3_compensate

def startBackward():
    print("Starting backward movement")
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.HIGH))
    Motor1_pwm.ChangeDutyCycle(0)
    Motor2_pwm.ChangeDutyCycle(gSliderSpeed * 100 / 255)
    Motor3_pwm.ChangeDutyCycle((gSliderSpeed + motor3_compensate) * 100 / 255)
    print(f"Motor2 PWM: {gSliderSpeed * 100 / 255}%, Motor3 PWM: {(gSliderSpeed + motor3_compensate) * 100 / 255}%")
    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    gCurSpeed1 = 0
    gCurSpeed2 = gSliderSpeed
    gCurSpeed3 = gSliderSpeed + motor3_compensate

def startTurnLeft():
    print("Starting left turn")
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.LOW))
    Motor1_pwm.ChangeDutyCycle(gSliderSpeed * 100 / 255)
    Motor2_pwm.ChangeDutyCycle(gSliderSpeed * 100 / 255)
    Motor3_pwm.ChangeDutyCycle((gSliderSpeed + motor3_compensate) * 100 / 255)
    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    gCurSpeed1 = gSliderSpeed
    gCurSpeed2 = gSliderSpeed
    gCurSpeed3 = gSliderSpeed + motor3_compensate

def startTurnRight():
    print("Starting right turn")
    GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH))
    Motor1_pwm.ChangeDutyCycle(gSliderSpeed * 100 / 255)
    Motor2_pwm.ChangeDutyCycle(gSliderSpeed * 100 / 255)
    Motor3_pwm.ChangeDutyCycle((gSliderSpeed + motor3_compensate) * 100 / 255)
    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    gCurSpeed1 = gSliderSpeed
    gCurSpeed2 = gSliderSpeed
    gCurSpeed3 = gSliderSpeed + motor3_compensate

def immediateStop():
    print("Immediate stop")
    Motor1_pwm.ChangeDutyCycle(0)
    Motor2_pwm.ChangeDutyCycle(0)
    Motor3_pwm.ChangeDutyCycle(0)
    global gCurSpeed1, gCurSpeed2, gCurSpeed3
    gCurSpeed1 = 0
    gCurSpeed2 = 0
    gCurSpeed3 = 0

# === Timed Movement Functions ===
def goForwards(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = triggered2 = triggered3 = False
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.LOW))
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    print(f"Motor2 PWM: {speed * 100 / 255}%, Motor3 PWM: {(speed + motor3_compensate) * 100 / 255}%")
    if interruptRequested:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        trigger_ultrasonic(Trig1)
        trigger_ultrasonic(Trig2)
        trigger_ultrasonic(Trig3)
        if commandCharacter:
            print("Movement interrupted by new command")
            break
        if interruptHandler():
            break
        time.sleep(0.01)

def goBackwards(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = triggered2 = triggered3 = False
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.HIGH))
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    print(f"Motor2 PWM: {speed * 100 / 255}%, Motor3 PWM: {(speed + motor3_compensate) * 100 / 255}%")
    if interruptRequested:
        return
    start = time.time()
    while time.time() - start < time_ms / 2:
        trigger_ultrasonic(Trig1)
        trigger_ultrasonic(Trig2)
        trigger_ultrasonic(Trig3)
        if commandCharacter:
            print("Movement interrupted by new command")
            break
        if interruptHandler():
            break
        time.sleep(0.01)

def stopTimed(time_ms):
    global interruptRequested
    changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0)
    if time_ms >= 0:
        start = time.time()
        while time.time() - start < time_ms / 1000:
            if commandCharacter:
                print("Stop interrupted by new command")
                break
            time.sleep(0.01)
    else:
        global permStop
        permStop = True

def turnRight(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = triggered2 = triggered3 = False
    GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH))
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        trigger_ultrasonic(Trig1)
        trigger_ultrasonic(Trig2)
        trigger_ultrasonic(Trig3)
        if commandCharacter:
            print("Turn right interrupted by new command")
            break
        if interruptHandler():
            break
        time.sleep(0.01)

def turnLeft(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = triggered2 = triggered3 = False
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.LOW, GPIO.LOW))
    changeSpeedSmooth(gCurSpeed1, speed, gCurSpeed2, speed, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        trigger_ultrasonic(Trig1)
        trigger_ultrasonic(Trig2)
        trigger_ultrasonic(Trig3)
        if commandCharacter:
            print("Turn left interrupted by new command")
            break
        if interruptHandler():
            break
        time.sleep(0.01)

def moveRight(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = triggered2 = triggered3 = False
    GPIO.output(dir_list, (GPIO.LOW, GPIO.HIGH, GPIO.LOW))
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, 0, gCurSpeed3, speed + motor3_compensate)
    if interruptRequested:
        return
    start = time.time()
    while time.time() - start < time_ms / 1:
        trigger_ultrasonic(Trig1)
        trigger_ultrasonic(Trig2)
        trigger_ultrasonic(Trig3)
        if commandCharacter:
            print("Move right interrupted by new command")
            break
        if interruptHandler():
            break
        time.sleep(0.01)

def moveLeft(speed, time_ms):
    global triggered1, triggered2, triggered3, interruptRequested
    triggered1 = triggered2 = triggered3 = False
    GPIO.output(dir_list, (GPIO.HIGH, GPIO.HIGH, GPIO.LOW))
    changeSpeedSmooth(gCurSpeed1, int(speed * 1.5), gCurSpeed2, speed, gCurSpeed3, 0)
    if interruptRequested:
        return
    start = time.time()
    while time.time() - start < time_ms / 1000:
        trigger_ultrasonic(Trig1)
        trigger_ultrasonic(Trig2)
        trigger_ultrasonic(Trig3)
        if commandCharacter:
            print("Move left interrupted by new command")
            break
        if interruptHandler():
            break
        time.sleep(0.01)

# === Command Processing Functions ===
def processImmediateCommand(command):
    global commandCharacter
    commandCharacter = ""
    if command == "forward":
        startForward()
    elif command == "backward":
        startBackward()
    elif command == "left":
        startTurnLeft()
    elif command == "right":
        startTurnRight()
    elif command == "stop":
        immediateStop()

def processCommand(command, time_ms):
    global commandCharacter
    commandCharacter = ""
    if command == "forward":
        goForwards(gSliderSpeed, time_ms)
    elif command == "backward":
        goBackwards(gSliderSpeed, time_ms)
    elif command == "turnRight":
        turnRight(gSliderSpeed, time_ms)
    elif command == "turnLeft":
        turnLeft(gSliderSpeed, time_ms)
    elif command == "moveRight":
        moveRight(gSliderSpeed, time_ms)
    elif command == "moveLeft":
        moveLeft(gSliderSpeed, time_ms)
    elif command == "stop":
        stopTimed(time_ms)
    else:
        print(f"Unknown timed command: {command}")

def speak(text):
    system = platform.system().lower()
    try:
        if system == "windows":
            ps_command = f'Add-Type -AssemblyName System.Speech; $speak = New-Object System.Speech.Synthesis.SpeechSynthesizer; $speak.Speak("{text}")'
            subprocess.run(["powershell", "-Command", ps_command], capture_output=True, check=True)
        elif system == "darwin":
            subprocess.run(["say", text], check=True)
        elif system == "linux":
            try:
                subprocess.run(["espeak", text], check=True)
            except (subprocess.CalledProcessError, FileNotFoundError):
                try:
                    subprocess.run(["echo", text, "|", "festival", "--tts"], shell=True, check=True)
                except (subprocess.CalledProcessError, FileNotFoundError):
                    subprocess.run(["spd-say", text], check=True)
        else:
            print(f"🔊 TTS: {text}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        try:
            if system == "windows":
                vbs_script = f'CreateObject("SAPI.SpVoice").Speak "{text}"'
                subprocess.run(["wscript", "/nologo", "-"], input=vbs_script, text=True, capture_output=True)
            elif system == "darwin":
                applescript = f'say "{text}"'
                subprocess.run(["osascript", "-e", applescript], check=True)
            else:
                raise subprocess.CalledProcessError(1, "TTS failed")
        except:
            print(f"🔊 TTS: {text}")

def listen():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening...")
        audio = r.listen(source)
        try:
            text = r.recognize_google(audio)
            print(f"🗣️ You said: {text}")
            return text.lower()
        except sr.UnknownValueError:
            print("❌ Didn't catch that.")
        except sr.RequestError:
            print("❌ API error.")
        return None

template = """Answer the question below.\nHere is the conversation history: {context}\nQuestion: {question}\nAnswer:"""
model = OllamaLLM(model="llama3")
prompt = ChatPromptTemplate.from_template(template)
chain = prompt | model

directions = {
    "forward": ["go forward", "move forward", "move ahead", "advance"],
    "backward": ["go backward", "move backward", "reverse"],
    "stop": ["stop", "halt", "stand still"],
    "turnLeft": ["turn left"],
    "turnRight": ["turn right"],
    "moveLeft": ["move left"],
    "moveRight": ["move right"],
}
time_patterns = {
    "seconds": r"(\d+)\s*seconds?",
    "minutes": r"(\d+)\s*minutes?",
    "hours": r"(\d+)\s*hours?"
}

def get_direction(user_input):
    for direction, phrases in directions.items():
        if any(phrase in user_input.lower() for phrase in phrases):
            return direction
    return None

def convert_to_milliseconds(text):
    for unit, pattern in time_patterns.items():
        match = re.search(pattern, text)
        if match:
            value = int(match.group(1))
            return value * 1000 if unit == "seconds" else value * 60000 if unit == "minutes" else value * 3600000
    return None

def keyboard_control_continuous():
    global keyboard_mode_active, exit_keyboard_mode
    print("🎮 Continuous keyboard mode activated!")
    print("Controls: ↑=Forward, ↓=Backward, ←=Left, →=Right, SPACE=Stop, E=Exit")
    last_key_state = {"up": False, "down": False, "left": False, "right": False, "space": False}
    key_commands = {"up": "forward", "down": "backward", "left": "left", "right": "right"}
    while keyboard_mode_active and not exit_keyboard_mode:
        try:
            if keyboard.is_pressed("e"):
                print("🚪 Exiting keyboard mode...")
                processImmediateCommand("stop")
                exit_keyboard_mode = True
                break
            current_key_states = {
                "up": keyboard.is_pressed("up"),
                "down": keyboard.is_pressed("down"),
                "left": keyboard.is_pressed("left"),
                "right": keyboard.is_pressed("right"),
                "space": keyboard.is_pressed("space")
            }
            for key, is_pressed in current_key_states.items():
                if is_pressed and not last_key_state[key]:
                    if key == "space":
                        processImmediateCommand("stop")
                        print("🛑 STOP pressed")
                    else:
                        command = key_commands[key]
                        processImmediateCommand(command)
                        print(f"▶️ {key.upper()} pressed → {command}")
            for key, is_pressed in current_key_states.items():
                if not is_pressed and last_key_state[key] and key != "space":
                    processImmediateCommand("stop")
                    print(f"⏹️ {key.upper()} released → stop")
            last_key_state = current_key_states.copy()
            time.sleep(0.02)
        except Exception as e:
            print(f"⚠️ Keyboard control error: {e}")
            time.sleep(0.1)

async def process_user_input(user_input, context):
    long_instruction = ""
    contain_instructions = False
    instructions = [instr.strip() for instr in user_input.split("then")]
    for instruction in instructions:
        instr_lower = instruction.lower()
        direction = get_direction(instr_lower)
        time_in_ms = convert_to_milliseconds(instr_lower)
        if 'turn left' in instr_lower:
            direction = 'turnLeft'
            time_in_ms = 5000
        elif 'turn right' in instr_lower:
            direction = 'turnRight'
            time_in_ms = 5000
        if direction and (time_in_ms is not None):
            long_instruction += f"{direction} {time_in_ms} "
            contain_instructions = True
        elif direction == "stop":
            long_instruction += "stop -1"
            contain_instructions = True
    if contain_instructions:
        print("Bot:", long_instruction.strip())
        words = long_instruction.strip().split()
        for i in range(0, len(words), 2):
            if i + 1 < len(words):
                command = words[i]
                duration = int(words[i + 1])
                global commandCharacter
                commandCharacter = command
                processCommand(command, duration)
        return True
    return False

async def handle_conversation():
    global keyboard_mode_active, exit_keyboard_mode
    context = ""
    print("Welcome to the AI Chatbot! Type 'exit' to quit.")
    while True:
        mode = input("Use (s)peech, (t)ype or (k)eyboard? ").strip().lower()
        if mode == 's':
            while True:
                user_input = listen()
                if not user_input:
                    continue
                if user_input.lower() == "exit":
                    return
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    result = chain.invoke({"context": context, "question": user_input})
                    print("Bot:", result)
                    speak(str(result))
                    context += f"\nUser: {user_input}\nAI: {result}"
                continue_mode = input("Continue speech mode? (y/n): ").strip().lower()
                if continue_mode == 'n':
                    break
        elif mode == 't':
            while True:
                user_input = input("You: ")
                if user_input.lower() == "exit":
                    return
                if await process_user_input(user_input, context):
                    context += f"\nUser: {user_input}\nAI: [Movement Command]"
                else:
                    result = chain.invoke({"context": context, "question": user_input})
                    print("Bot:", result)
                    speak(str(result))
                    context += f"\nUser: {user_input}\nAI: {result}"
                continue_mode = input("Continue text mode? (y/n): ").strip().lower()
                if continue_mode == 'n':
                    break
        elif mode == 'k':
            keyboard_mode_active = True
            exit_keyboard_mode = False
            keyboard_thread = threading.Thread(target=keyboard_control_continuous, daemon=True)
            keyboard_thread.start()
            while keyboard_mode_active and not exit_keyboard_mode:
                await asyncio.sleep(0.1)
            keyboard_mode_active = False
            print("🔄 Returning to mode selection...")
        else:
            print("❌ Invalid mode. Please choose 's', 't', or 'k'.")

async def main():
    global main_loop
    main_loop = asyncio.get_running_loop()
    try:
        await handle_conversation()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        print("Cleaning up GPIO...")
        Motor1_pwm.stop()
        Motor2_pwm.stop()
        Motor3_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
