/*
  BLE_Central_Device.ino

  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE Sense 
  as a central device and looks for a specified service and characteristic in a 
  peripheral device. If the specified service and characteristic is found in a 
  peripheral device, the last detected value of the on-board gesture sensor of 
  the Nano 33 BLE Sense, the APDS9960, is written in the specified characteristic. 

  The circuit:
  - Arduino Nano 33 BLE Sense. 

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>

const char* deviceServiceUuid = "B823D700-37A9-11EC-8F15-C0A645423200";
const char* deviceServiceCharacteristicUuid = "B823D700-37A9-11EC-8F15-C0A645423200";

int gesture = -1;
int oldGestureValue = -1;   

int gestureDetectection();
void controlPeripheral(BLEDevice peripheral);
void connectToPeripheral();

void setup() {
  Serial.begin(9600);
  //BLE.setLocalName("Arduino Nano BLE 33");
  while (!Serial);
  
  // if (!APDS.begin()) {
  //   Serial.println("* Error initializing APDS9960 sensor!");
  // } 

  //APDS.setGestureSensitivity(80); 
  
  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1);
  }
  
  BLE.setLocalName("Nano 33 BLE (Central)"); //set the findable bluetooth name of the device
  BLE.advertise();  // advertise: i.e. allow it to be found by other devices through bluetooth

  Serial.println("Arduino Nano 33 BLE Sense (Central Device)");
  Serial.println(" ");
  BLE.scan();
}

void loop() {
  connectToPeripheral(); //find the specified device and connect to it
}

void connectToPeripheral(){
  BLEDevice peripheral;
  
  Serial.println("- Discovering peripheral device...");

  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    //Serial.println("looping"); // debug, want to check why no periph dev found

    peripheral = BLE.available();
  } while (!peripheral);
  
  if (peripheral) {
    Serial.println("* Peripheral device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.print("* Device name: ");
    Serial.println(peripheral.localName());
    Serial.print("* Advertised service UUID: ");
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan();
    controlPeripheral(peripheral);
  }
}

void controlPeripheral(BLEDevice peripheral) {
  Serial.println("- Connecting to peripheral device...");

  if (peripheral.connect()) {
    Serial.println("* Connected to peripheral device!");
    Serial.println(" ");
  } else {
    Serial.println("* Connection to peripheral device failed!");
    Serial.println(" ");
    return;
  }

  Serial.println("- Discovering peripheral device attributes...");
  if (peripheral.discoverAttributes()) {
    Serial.println("* Peripheral device attributes discovered!");
    Serial.println(" ");
  } else {
    Serial.println("* Peripheral device attributes discovery failed!");
    Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  // BLECharacteristic gestureCharacteristic = peripheral.characteristic(deviceServiceCharacteristicUuid);
    
  // if (!gestureCharacteristic) {
  //   Serial.println("* Peripheral device does not have gesture_type characteristic!");
  //   peripheral.disconnect();
  //   return;
  // } else if (!gestureCharacteristic.canWrite()) {
  //   Serial.println("* Peripheral does not have a writable gesture_type characteristic!");
  //   peripheral.disconnect();
  //   return;
  // }
  
  // while (peripheral.connected()) {
  //   gesture = gestureDetectection();

  //   if (oldGestureValue != gesture) {  
  //     oldGestureValue = gesture;
  //     Serial.print("* Writing value to gesture_type characteristic: ");
  //     Serial.println(gesture);
  //     gestureCharacteristic.writeValue((byte)gesture);
  //     Serial.println("* Writing value to gesture_type characteristic done!");
  //     Serial.println(" ");
  //   }
  
  // }
  Serial.println("- Peripheral device disconnected!");
}
  
// int gestureDetectection() {
//   if (APDS.gestureAvailable()) {
//     gesture = APDS.readGesture();

//     switch (gesture) {
//       case GESTURE_UP:
//         Serial.println("- UP gesture detected");
//         break;
//       case GESTURE_DOWN:
//         Serial.println("- DOWN gesture detected");
//         break;
//       case GESTURE_LEFT:
//         Serial.println("- LEFT gesture detected");
//         break;
//       case GESTURE_RIGHT:
//         Serial.println("- RIGHT gesture detected");
//         break;
//       default:
//         Serial.println("- No gesture detected");
//         break;
//       }
//     }
//     return gesture;
// }/**/