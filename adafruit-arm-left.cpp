#pragma GCC optimize ("O0")

#include <AX12A.h>
#include <Servo.h>

#define DirectionPin  (1u) //TX PIN
#define BaudRate    (1000000u)

#define ID_1  (3u)
#define ID_2  (2u)
#define ID_3  (1u)
#define ID_4  (4u)
//#define ID_  (5u)
#define ID_5  (6u)
#define ID_6  (7u)

#include <Wire.h>
#define Left_Ada_Addr 5

// set up servo
Servo leftServo;
int lsPos = 0;

int receivedData = 0;
int time = 2500;

void setAngleLimits(){ //adjust as needed, 0 - 300 degrees
  ax12a.setAngleLimit(ID_1, 0, 1023);
  ax12a.setAngleLimit(ID_2, 0, 1023);
  ax12a.setAngleLimit(ID_3, 0, 1023);
  ax12a.setAngleLimit(ID_4, 0, 1023);
  ax12a.setAngleLimit(ID_5, 0, 1023);
  ax12a.setAngleLimit(ID_6, 0, 1023);
  //ax12a.setAngleLimit(ID_7, 0, 1023);
}
void setVoltageLimits(){ //9~11V
  ax12a.setVoltageLimit(ID_1, 9, 11);
  ax12a.setVoltageLimit(ID_2, 9, 11);
  ax12a.setVoltageLimit(ID_3, 9, 11);
  ax12a.setVoltageLimit(ID_4, 9, 11);
  ax12a.setVoltageLimit(ID_5, 9, 11);
  ax12a.setVoltageLimit(ID_6, 9, 11);
  //ax12a.setVoltageLimit(ID_7, 9, 11);
}

void setTorqueLimits(){
  ax12a.setMaxTorque(ID_1, 1023);
  ax12a.setMaxTorque(ID_2, 1023);
  ax12a.setMaxTorque(ID_3, 1023);
  ax12a.setMaxTorque(ID_4, 1023);
  ax12a.setMaxTorque(ID_5, 1023);
  ax12a.setMaxTorque(ID_6, 1023);
 // ax12a.setMaxTorque(ID_7, 1023);
}

void setTorqueOn(){
  ax12a.torqueStatus(ID_1, true);
  ax12a.torqueStatus(ID_2, true);
  ax12a.torqueStatus(ID_3, true);
  ax12a.torqueStatus(ID_4, true);
  ax12a.torqueStatus(ID_5, true);
  ax12a.torqueStatus(ID_6, true);
  //ax12a.torqueStatus(ID_7, true);

  ax12a.setEndless(ID_1, false);
  ax12a.setEndless(ID_2, false);
  ax12a.setEndless(ID_3, false);
  ax12a.setEndless(ID_4, false);
  ax12a.setEndless(ID_5, false);
  ax12a.setEndless(ID_6, false);
//  ax12a.setEndless(ID_7, false);
}

const int complianceMargin = 15;
void setComplianceMargins(){ //10 degrees within target position
  ax12a.setCMargin(ID_1, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_2, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_3, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_4, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_5, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_6, complianceMargin, complianceMargin);
  //ax12a.setCMargin(ID_7, complianceMargin, complianceMargin);
}

const int servoSpeed = 100;
// void moveToState1(){ //arms stowed away
//   ax12a.moveSpeed(ID_1, 512, servoSpeed);
//   ax12a.moveSpeed(ID_2, 112, servoSpeed);
//   ax12a.moveSpeed(ID_3, 914, servoSpeed);
//   ax12a.moveSpeed(ID_4, 813, servoSpeed);
//   ax12a.moveSpeed(ID_5, 219, servoSpeed);
//   ax12a.moveSpeed(ID_6, 205, servoSpeed);
//   //ax12a.moveSpeed(ID_7, 512, servoSpeed);
// }
// void moveToState2(){ //halfway lift arm up from stowed position
//   ax12a.moveSpeed(ID_1, 512, servoSpeed);
//   ax12a.moveSpeed(ID_2, 379, servoSpeed);
//   ax12a.moveSpeed(ID_3, 644, servoSpeed);
//   ax12a.moveSpeed(ID_4, 827, servoSpeed);
//   ax12a.moveSpeed(ID_5, 206, servoSpeed);
//   ax12a.moveSpeed(ID_6, 205, servoSpeed);
//   //ax12a.moveSpeed(ID_7, 512, servoSpeed);
// }
// void moveToState3(){ //arm fully lifted up from stowed position
//   ax12a.moveSpeed(ID_1, 512, servoSpeed);
//   ax12a.moveSpeed(ID_2, 346, servoSpeed);
//   ax12a.moveSpeed(ID_3, 679, servoSpeed);
//   ax12a.moveSpeed(ID_4, 701, servoSpeed);
//   ax12a.moveSpeed(ID_5, 331, servoSpeed);
//   ax12a.moveSpeed(ID_6, 205, servoSpeed);
//   //ax12a.moveSpeed(ID_7, 512, servoSpeed);
// }
// void moveToState4(){ //arm lower down to rest in front of computer
//   ax12a.moveSpeed(ID_1, 880, servoSpeed);
//   ax12a.moveSpeed(ID_2, 305, servoSpeed);
//   ax12a.moveSpeed(ID_3, 718, servoSpeed);
//   ax12a.moveSpeed(ID_4, 701, servoSpeed);
//   ax12a.moveSpeed(ID_5, 331, servoSpeed);
//   ax12a.moveSpeed(ID_6, 205, servoSpeed);
//   //ax12a.moveSpeed(ID_7, 512, servoSpeed);
// }
// void moveToState5(){ //arm back up to rotate hand for wave preparation
//   ax12a.moveSpeed(ID_1, 512, servoSpeed);
//   ax12a.moveSpeed(ID_2, 341, servoSpeed);
//   ax12a.moveSpeed(ID_3, 683, servoSpeed);
//   ax12a.moveSpeed(ID_4, 521, servoSpeed);
//   ax12a.moveSpeed(ID_5, 508, servoSpeed);
//   ax12a.moveSpeed(ID_6, 521, servoSpeed);
//   //ax12a.moveSpeed(ID_7, 512, servoSpeed);
// }
// void moveToState6(){ //move hand wave left
//   ax12a.moveSpeed(ID_1, 512, servoSpeed);
//   ax12a.moveSpeed(ID_2, 333, servoSpeed);
//   ax12a.moveSpeed(ID_3, 688, servoSpeed);
//   ax12a.moveSpeed(ID_4, 690, servoSpeed);
//   ax12a.moveSpeed(ID_5, 342, servoSpeed);
//   ax12a.moveSpeed(ID_6, 525, servoSpeed);
//   //ax12a.moveSpeed(ID_7, 512, servoSpeed);
// }
// void moveToState7(){ //move hand wave right
//   ax12a.moveSpeed(ID_1, 512, servoSpeed);
//   ax12a.moveSpeed(ID_2, 330, servoSpeed);
//   ax12a.moveSpeed(ID_3, 693, servoSpeed);
//   ax12a.moveSpeed(ID_4, 392, servoSpeed);
//   ax12a.moveSpeed(ID_5, 636, servoSpeed);
//   ax12a.moveSpeed(ID_6, 525, servoSpeed);
//   //ax12a.moveSpeed(ID_7, 512, servoSpeed);
// }
// void moveToState8(){ //move hand wave right
//   ax12a.moveSpeed(ID_1, 180, servoSpeed);
//   ax12a.moveSpeed(ID_2, 180, servoSpeed);
//   ax12a.moveSpeed(ID_3, 180, servoSpeed);
//   ax12a.moveSpeed(ID_4, 180, servoSpeed);
//   ax12a.moveSpeed(ID_5, 180, servoSpeed);
//   ax12a.moveSpeed(ID_6, 520, servoSpeed);
//   // ax12a.moveSpeed(ID_7, 180, servoSpeed);
// }

// void moveToState9(){ //move hand wave right
//   ax12a.moveSpeed(ID_1, 520, servoSpeed);
//   ax12a.moveSpeed(ID_2, 520, servoSpeed);
//   ax12a.moveSpeed(ID_3, 520, servoSpeed);
//   ax12a.moveSpeed(ID_4, 520, servoSpeed);
//   ax12a.moveSpeed(ID_5, 520, servoSpeed);
//   ax12a.moveSpeed(ID_6, 520, servoSpeed);
//   // ax12a.moveSpeed(ID_7, 520, servoSpeed);
// }

void changeSpeedSmooth(Servo servo, int pos){
  int i = 0;
  int cur = servo.read();
  // Serial.print("Current location ");
  // Serial.println(cur);
  // Serial.print("Target location: ");
  // Serial.println(pos);
  if(pos > cur){
    for(i = 0; i < (pos-cur); i++){
        servo.write(cur + i);
        delay(50);
        Serial.println(i);
    }
    Serial.println("ran1");
  } else if(pos < cur){
    for(i = 0; i < (cur-pos); i++){
      servo.write(cur - i);
      delay(50);
      Serial.println(i);
    }
    Serial.println("ran2");
  } else {
    Serial.println("Already there");
  }
}

//arm down straight
void normalPos(){
  changeSpeedSmooth(leftServo, 200);
  ax12a.moveSpeed(ID_1, 1023-800, servoSpeed);
  ax12a.moveSpeed(ID_2, 800, servoSpeed);
  ax12a.moveSpeed(ID_3, 1023-520, servoSpeed);
  ax12a.moveSpeed(ID_4, 520, servoSpeed);
  ax12a.moveSpeed(ID_5, 520, servoSpeed);
  ax12a.moveSpeed(ID_6, 800, servoSpeed);
  Serial.println("normal pos");
}

//arm extended to shake hand
void reachHand(){
  changeSpeedSmooth(leftServo, 130);
  ax12a.moveSpeed(ID_1, 1023-800, servoSpeed);
  ax12a.moveSpeed(ID_2, 800, servoSpeed);
  ax12a.moveSpeed(ID_3, 1023-520, servoSpeed);
  ax12a.moveSpeed(ID_4, 520, servoSpeed);
  ax12a.moveSpeed(ID_5, 520, servoSpeed);
  ax12a.moveSpeed(ID_6, 800, servoSpeed);
  Serial.println("reach hand");
}

//not working dont run
void openHand(){
  ax12a.moveSpeed(ID_6, 700, servoSpeed);
  Serial.println("open hand");
}

void wave(){
  changeSpeedSmooth(leftServo, 30);
  ax12a.moveSpeed(ID_1, 1023-520, servoSpeed);
  ax12a.moveSpeed(ID_2, 520, servoSpeed);
  ax12a.moveSpeed(ID_3, 1023-520, servoSpeed);
  ax12a.moveSpeed(ID_4, 520, servoSpeed);
  ax12a.moveSpeed(ID_5, 300, servoSpeed);
  ax12a.moveSpeed(ID_6, 800, servoSpeed);
  delay(500);

  for(int i = 0; i < 3; i++){
    ax12a.moveSpeed(ID_3, 1023-800, servoSpeed);
    ax12a.moveSpeed(ID_4, 800, servoSpeed);
    delay(500);
    ax12a.moveSpeed(ID_3, 1023-520, servoSpeed);
    ax12a.moveSpeed(ID_4, 520, servoSpeed);
    delay(500);
  }

  Serial.println("wave");
}

void setup()
{
  Serial.begin(9600);

  Wire.begin(Left_Ada_Addr);
  Wire.onReceive(receiveEvent);

  ax12a.begin(BaudRate, DirectionPin, &Serial1);
  setTorqueOn();
  //setTorqueLimits();
  setAngleLimits();
  setVoltageLimits();
  setComplianceMargins();

  leftServo.attach(9); // pin 9 is for the left servo
  //changeSpeedSmooth(leftServo, 0);
}

//data is receieved in 4 bit length packets
void receiveEvent(int byteCount) {
  // Serial.println("ReceiveEvent");

  receivedData = Wire.read();

  if(receivedData == 5){
    Serial.println("Startup Arm Received");   
  }else if(receivedData == 6){
    Serial.println("Shutdown Arm Received");
  }else if(receivedData == 7){
    Serial.println("Wave Arm Received");
  }else{
    Serial.println("This should not happen");
  }
}

void loop()
{
  // changeSpeedSmooth(leftServo, 120);
  // ax12a.moveSpeed(ID_1, 1023-520, servoSpeed);
  // ax12a.moveSpeed(ID_2, 520, servoSpeed);
  // ax12a.moveSpeed(ID_3, 1023-520, servoSpeed);
  // ax12a.moveSpeed(ID_4, 520, servoSpeed);
  // ax12a.moveSpeed(ID_5, 520, servoSpeed);
  // ax12a.moveSpeed(ID_6, 520, servoSpeed);
  // Serial.println("Main loop");
  // //Serial.println(leftServo.read());
  // delay(1000);

  // changeSpeedSmooth(leftServo, 200);
  // ax12a.moveSpeed(ID_1, 1023-450, servoSpeed);
  // ax12a.moveSpeed(ID_2, 450, servoSpeed);
  // ax12a.moveSpeed(ID_3, 1023-450, servoSpeed);
  // ax12a.moveSpeed(ID_4, 450, servoSpeed);
  // ax12a.moveSpeed(ID_5, 450, servoSpeed);
  // ax12a.moveSpeed(ID_6, 450, servoSpeed);

  normalPos();
  delay(5000);
  reachHand();
  delay(5000);
  openHand();
  delay(5000);
  wave();
  delay(5000);

  // Serial.println("500");
  // ax12a.moveSpeed(ID_6, 500, servoSpeed);
  // delay(1000);
  // Serial.println("400");
  // ax12a.moveSpeed(ID_6, 400, servoSpeed);
  // delay(1000);
  // Serial.println("800");
  // ax12a.moveSpeed(ID_6, 800, servoSpeed); //ok
  // Serial.println("700");
  // ax12a.moveSpeed(ID_6, 700, servoSpeed); //ok
  // delay(1000);
  
  // for(lsPos = 0; lsPos <= 180; lsPos += 1){
  //   leftServo.write(lsPos);
  //   delay(75);
  // }
  // for(lsPos = 180; lsPos >= 0; lsPos -= 1){
  //   leftServo.write(lsPos);
  //   delay(75);
  // }

}