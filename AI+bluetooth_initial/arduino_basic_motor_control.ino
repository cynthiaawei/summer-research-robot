
//FRONT MOTOR
#define Motor1_Speed A3 //PWM1
#define Motor1_Dir 3 //DIR1

//BACK LEFT MOTOR
#define Motor2_Speed A2 //PWM2 
#define Motor2_Dir 2 //DIR2

//BACK RIGHT MOTOR
#define Motor3_Speed A1 //PWM2 (second h-bridge)
#define Motor3_Dir 4 //DIR2 (second-bridge)

//global current speed of motors + directions
int gCurSpeed1 = 0;
int gCurSpeed2 = 0;
int gCurSpeed3 = 0;
int gSliderSpeed = 25;
bool curDir1 = HIGH; //HIGH = forward direction
bool curDir2 = HIGH; //HIGH = forward direction
bool curDir3 = HIGH; //HIGH = forward direction

//bool conditions for movement direction
//bool movingBack = false;

void changeSpeedSmooth(int curSpeed1, int newSpeed1, int curSpeed2, int newSpeed2, int curSpeed3, int newSpeed3){ 
  //bool flip1, bool flip2, bool flip3){

  int i = curSpeed1, j = curSpeed2, k = curSpeed3;
  while(i != newSpeed1 || j != newSpeed2 || k != newSpeed3){
    if(i < newSpeed1) i++;
    else if(i > newSpeed1) i--;
    if(j < newSpeed2) j++;
    else if(j > newSpeed2) j--;
    if(k < newSpeed3) k++;
    else if(k > newSpeed3) k--; 

    analogWrite(Motor1_Speed, i);
    analogWrite(Motor2_Speed, j);
    analogWrite(Motor3_Speed, k);
    
    delay(10); 
    //delay is used to slow down the update of the speed to the motors 
    //this is to ensure we don't immediately jump from 0 to max speed and fry our motor controller
  }

  /*
  //update direction pins on flip condition
  if(flip1) digitalWrite(Motor1_Dir, curDir1);
  if(flip2) digitalWrite(Motor2_Dir, curDir2);
  if(flip3) digitalWrite(Motor3_Dir, curDir3);
  */
  delay(10);
  gCurSpeed1 = newSpeed1;
  gCurSpeed2 = newSpeed2;
  gCurSpeed3 = newSpeed3;
}

void setup() {
  // put your setup code here, to run once:
  //setup motor control pins
  pinMode(Motor1_Speed, OUTPUT);
  pinMode(Motor1_Dir, OUTPUT);
  pinMode(Motor2_Speed, OUTPUT);
  pinMode(Motor2_Dir, OUTPUT);
  pinMode(Motor3_Speed, OUTPUT);
  pinMode(Motor3_Dir, OUTPUT);

  //initilize speed and direction to 0
  analogWrite(Motor1_Speed,gCurSpeed1);
  digitalWrite(Motor1_Dir,curDir1);
  analogWrite(Motor2_Speed,gCurSpeed2);
  digitalWrite(Motor2_Dir,curDir2);
  analogWrite(Motor3_Speed,gCurSpeed3);
  digitalWrite(Motor3_Dir,curDir3);

  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, gSliderSpeed, gCurSpeed3, gSliderSpeed);

}
int motor3_compensate = 15;

void goForwards(int speed, int time_ms){
  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, gSliderSpeed, gCurSpeed3, gSliderSpeed+motor3_compensate);

  int currentTime = millis();
  while(millis() - currentTime < time_ms){}
}

void goBackwards(int speed, int time_ms){
  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, gSliderSpeed, gCurSpeed3, gSliderSpeed+motor3_compensate);

  int currentTime = millis();
  while(millis() - currentTime < time_ms){}
}

void stop(int time_ms){
  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, 0, gCurSpeed2, 0, gCurSpeed3, 0);

  int currentTime = millis();
  while(millis() - currentTime < time_ms){}
}

void turnRight(int speed, int time_ms){
  curDir1 = LOW;
  curDir2 = HIGH;
  curDir3 = HIGH;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, gSliderSpeed, gCurSpeed2, gSliderSpeed, gCurSpeed3, gSliderSpeed+motor3_compensate);

  int currentTime = millis();
  while(millis() - currentTime < time_ms){}
}

void turnLeft(int speed, int time_ms){
  curDir1 = HIGH;
  curDir2 = LOW;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, gSliderSpeed, gCurSpeed2, gSliderSpeed, gCurSpeed3, gSliderSpeed+motor3_compensate);

  int currentTime = millis();
  while(millis() - currentTime < time_ms){}
}

void moveRight(int speed, int time_ms){
  curDir1 = LOW;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, gSliderSpeed*1.5, gCurSpeed2, 0, gCurSpeed3, gSliderSpeed+motor3_compensate);

  int currentTime = millis();
  while(millis() - currentTime < time_ms){}
}

void moveLeft(int speed, int time_ms){
  curDir1 = HIGH;
  curDir2 = HIGH;
  curDir3 = LOW;
  digitalWrite(Motor1_Dir, curDir1);
  digitalWrite(Motor2_Dir, curDir2);
  digitalWrite(Motor3_Dir, curDir3);

  changeSpeedSmooth(gCurSpeed1, gSliderSpeed*1.5, gCurSpeed2, gSliderSpeed, gCurSpeed3, 0);

  int currentTime = millis();
  while(millis() - currentTime < time_ms){}
}


void loop() {
  goForwards(gSliderSpeed, 2000);
  stop(2000);

  goBackwards(gSliderSpeed, 2000);
  stop(2000);

  turnRight(gSliderSpeed, 2000);
  stop(2000);

  turnLeft(gSliderSpeed, 2000);
  stop(2000);

  moveRight(gSliderSpeed, 2000);
  stop(2000);

  moveLeft(gSliderSpeed, 2000);
  stop(2000);
}



/*
  MOTOR 1: HIGH = CW
  MOTOR 2: HIGH = CCW
  MOTOR 3: HIGH = CCW
*/
