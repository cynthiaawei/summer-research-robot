#pragma GCC optimize ("O0")

#include <AX12A.h>

#define DirectionPin  (1u) //TX PIN
#define BaudRate    (1000000u)
#define ID_1  (1u)
#define ID_2  (2u)
#define ID_3  (3u)
#define ID_4  (4u)
#define ID_5  (5u)
#define ID_6  (6u)
#define ID_7  (7u)

#include <Wire.h>
#define Left_Ada_Addr 5
int receivedData = 0;
int time = 2500;

void setAngleLimits(){ //adjust as needed, 0 - 300 degrees
  ax12a.setAngleLimit(ID_1, 0, 1023);
  ax12a.setAngleLimit(ID_2, 0, 1023);
  ax12a.setAngleLimit(ID_3, 0, 1023);
  ax12a.setAngleLimit(ID_4, 0, 1023);
  ax12a.setAngleLimit(ID_5, 0, 1023);
  ax12a.setAngleLimit(ID_6, 0, 1023);
  ax12a.setAngleLimit(ID_7, 0, 1023);
}
void setVoltageLimits(){ //9~11V
  ax12a.setVoltageLimit(ID_1, 9, 11);
  ax12a.setVoltageLimit(ID_2, 9, 11);
  ax12a.setVoltageLimit(ID_3, 9, 11);
  ax12a.setVoltageLimit(ID_4, 9, 11);
  ax12a.setVoltageLimit(ID_5, 9, 11);
  ax12a.setVoltageLimit(ID_6, 9, 11);
  ax12a.setVoltageLimit(ID_7, 9, 11);
}

void setTorqueLimits(){
  ax12a.setMaxTorque(ID_1, 1023);
  ax12a.setMaxTorque(ID_2, 500);
  ax12a.setMaxTorque(ID_3, 500);
  ax12a.setMaxTorque(ID_4, 500);
  ax12a.setMaxTorque(ID_5, 500);
  ax12a.setMaxTorque(ID_6, 500);
  ax12a.setMaxTorque(ID_7, 500);
}

void setTorqueOn(){
  ax12a.torqueStatus(ID_1, true);
  ax12a.torqueStatus(ID_2, true);
  ax12a.torqueStatus(ID_3, true);
  ax12a.torqueStatus(ID_4, true);
  ax12a.torqueStatus(ID_5, true);
  ax12a.torqueStatus(ID_6, true);
  ax12a.torqueStatus(ID_7, true);

  ax12a.setEndless(ID_1, false);
  ax12a.setEndless(ID_2, false);
  ax12a.setEndless(ID_3, false);
  ax12a.setEndless(ID_4, false);
  ax12a.setEndless(ID_5, false);
  ax12a.setEndless(ID_6, false);
  ax12a.setEndless(ID_7, false);
}

const int complianceMargin = 15;
void setComplianceMargins(){ //10 degrees within target position
  ax12a.setCMargin(ID_1, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_2, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_3, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_4, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_5, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_6, complianceMargin, complianceMargin);
  ax12a.setCMargin(ID_7, complianceMargin, complianceMargin);
}

const int servoSpeed = 100;
void moveToState1(){ //arms stowed away
  ax12a.moveSpeed(ID_1, 512, servoSpeed);
  ax12a.moveSpeed(ID_2, 112, servoSpeed);
  ax12a.moveSpeed(ID_3, 914, servoSpeed);
  ax12a.moveSpeed(ID_4, 813, servoSpeed);
  ax12a.moveSpeed(ID_5, 219, servoSpeed);
  ax12a.moveSpeed(ID_6, 205, servoSpeed);
  ax12a.moveSpeed(ID_7, 512, servoSpeed);
}
void moveToState2(){ //halfway lift arm up from stowed position
  ax12a.moveSpeed(ID_1, 512, servoSpeed);
  ax12a.moveSpeed(ID_2, 379, servoSpeed);
  ax12a.moveSpeed(ID_3, 644, servoSpeed);
  ax12a.moveSpeed(ID_4, 827, servoSpeed);
  ax12a.moveSpeed(ID_5, 206, servoSpeed);
  ax12a.moveSpeed(ID_6, 205, servoSpeed);
  ax12a.moveSpeed(ID_7, 512, servoSpeed);
}
void moveToState3(){ //arm fully lifted up from stowed position
  ax12a.moveSpeed(ID_1, 512, servoSpeed);
  ax12a.moveSpeed(ID_2, 346, servoSpeed);
  ax12a.moveSpeed(ID_3, 679, servoSpeed);
  ax12a.moveSpeed(ID_4, 701, servoSpeed);
  ax12a.moveSpeed(ID_5, 331, servoSpeed);
  ax12a.moveSpeed(ID_6, 205, servoSpeed);
  ax12a.moveSpeed(ID_7, 512, servoSpeed);
}
void moveToState4(){ //arm lower down to rest in front of computer
  ax12a.moveSpeed(ID_1, 880, servoSpeed);
  ax12a.moveSpeed(ID_2, 305, servoSpeed);
  ax12a.moveSpeed(ID_3, 718, servoSpeed);
  ax12a.moveSpeed(ID_4, 701, servoSpeed);
  ax12a.moveSpeed(ID_5, 331, servoSpeed);
  ax12a.moveSpeed(ID_6, 205, servoSpeed);
  ax12a.moveSpeed(ID_7, 512, servoSpeed);
}
void moveToState5(){ //arm back up to rotate hand for wave preparation
  ax12a.moveSpeed(ID_1, 512, servoSpeed);
  ax12a.moveSpeed(ID_2, 341, servoSpeed);
  ax12a.moveSpeed(ID_3, 683, servoSpeed);
  ax12a.moveSpeed(ID_4, 521, servoSpeed);
  ax12a.moveSpeed(ID_5, 508, servoSpeed);
  ax12a.moveSpeed(ID_6, 521, servoSpeed);
  ax12a.moveSpeed(ID_7, 512, servoSpeed);
}
void moveToState6(){ //move hand wave left
  ax12a.moveSpeed(ID_1, 512, servoSpeed);
  ax12a.moveSpeed(ID_2, 333, servoSpeed);
  ax12a.moveSpeed(ID_3, 688, servoSpeed);
  ax12a.moveSpeed(ID_4, 690, servoSpeed);
  ax12a.moveSpeed(ID_5, 342, servoSpeed);
  ax12a.moveSpeed(ID_6, 525, servoSpeed);
  ax12a.moveSpeed(ID_7, 512, servoSpeed);
}
void moveToState7(){ //move hand wave right
  ax12a.moveSpeed(ID_1, 512, servoSpeed);
  ax12a.moveSpeed(ID_2, 330, servoSpeed);
  ax12a.moveSpeed(ID_3, 693, servoSpeed);
  ax12a.moveSpeed(ID_4, 392, servoSpeed);
  ax12a.moveSpeed(ID_5, 636, servoSpeed);
  ax12a.moveSpeed(ID_6, 525, servoSpeed);
  ax12a.moveSpeed(ID_7, 512, servoSpeed);
}
void moveToState8(){ //move hand wave right
  ax12a.moveSpeed(ID_1, 180, servoSpeed);
  ax12a.moveSpeed(ID_2, 180, servoSpeed);
  ax12a.moveSpeed(ID_3, 180, servoSpeed);
  ax12a.moveSpeed(ID_4, 180, servoSpeed);
  ax12a.moveSpeed(ID_5, 180, servoSpeed);
  ax12a.moveSpeed(ID_6, 520, servoSpeed);
  ax12a.moveSpeed(ID_7, 180, servoSpeed);
}

void moveToState9(){ //move hand wave right
  ax12a.moveSpeed(ID_1, 520, servoSpeed);
  ax12a.moveSpeed(ID_2, 520, servoSpeed);
  ax12a.moveSpeed(ID_3, 520, servoSpeed);
  ax12a.moveSpeed(ID_4, 520, servoSpeed);
  ax12a.moveSpeed(ID_5, 520, servoSpeed);
  ax12a.moveSpeed(ID_6, 520, servoSpeed);
  ax12a.moveSpeed(ID_7, 520, servoSpeed);
}

void test1(){
  ax12a.moveSpeed(ID_1, 0, servoSpeed);
  delay(500);
  ax12a.moveSpeed(ID_2, 500, servoSpeed);
  ax12a.moveSpeed(ID_3, 1023-500, servoSpeed);
  delay(500);
  ax12a.moveSpeed(ID_4, 1023-300, servoSpeed);
  ax12a.moveSpeed(ID_5, 300, servoSpeed);
  delay(500);
  ax12a.moveSpeed(ID_4, 1023-600, servoSpeed);
  ax12a.moveSpeed(ID_5, 600, servoSpeed);
  delay(500);
  ax12a.moveSpeed(ID_4, 1023-300, servoSpeed);
  ax12a.moveSpeed(ID_5, 300, servoSpeed);
  delay(500);
  delay(6000);
  Serial.println(ax12a.readLoad(ID_1));

  ax12a.moveSpeed(ID_1, 600, 100);
  delay(500);
  ax12a.moveSpeed(ID_2, 600, servoSpeed);
  ax12a.moveSpeed(ID_3, 1023-600, servoSpeed);
  delay(5000);
  Serial.println(ax12a.readLoad(ID_1));

  ax12a.moveSpeed(ID_1, 1023, 200);
  delay(500);
  ax12a.moveSpeed(ID_2, 500, servoSpeed);
  ax12a.moveSpeed(ID_3, 1023-500, servoSpeed);
  delay(500);
  delay(5000);
  Serial.println(ax12a.readLoad(ID_1));
  Serial.println("");
}



void extendShake(){
  if(ax12a.readSpeed(ID_1) <= 0){
    ax12a.torqueStatus(ID_1, true);
    ax12a.setMaxTorque(ID_1, 1023);
  }

  ax12a.moveSpeed(ID_1, 100, servoSpeed);
  delay(500);
  ax12a.moveSpeed(ID_2, 1023-250, servoSpeed);
  ax12a.moveSpeed(ID_3, 250, servoSpeed);
  delay(500);
  ax12a.moveSpeed(ID_6, 520, servoSpeed);
  delay(500);
  ax12a.moveSpeed(ID_4, 1023-500, servoSpeed/2);
  ax12a.moveSpeed(ID_5, 500, servoSpeed/2);
  //ax12a.setMaxTorque(ID_1, 0);


  delay(500);
  Serial.println(ax12a.readSpeed(ID_1));
  if(ax12a.readSpeed(ID_1) <= 0){
    ax12a.torqueStatus(ID_1, true);
    ax12a.setMaxTorque(ID_1, 1023);
  }
  ax12a.moveSpeed(ID_1, 750, servoSpeed);
  // delay(500);
  // ax12a.moveSpeed(ID_2, 1023-300, servoSpeed/4);
  // ax12a.moveSpeed(ID_3, 300, servoSpeed/4);
  // delay(500);
  // ax12a.moveSpeed(ID_4, 1023-450, servoSpeed/4);
  // ax12a.moveSpeed(ID_5, 450, servoSpeed/4);
  // delay(500);
  // ax12a.moveSpeed(ID_6, 250, servoSpeed);
  
  delay(time*1.5);
  Serial.println("ran");
}


void setup()
{
  Serial.begin(9600);

  Wire.begin(Left_Ada_Addr);
  Wire.onReceive(receiveEvent);

  ax12a.begin(BaudRate, DirectionPin, &Serial1);
  setTorqueOn();
  setTorqueLimits();
  setAngleLimits();
  setVoltageLimits();
  setComplianceMargins();
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
extendShake();

  
}