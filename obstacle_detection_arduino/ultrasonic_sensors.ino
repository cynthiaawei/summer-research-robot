
const int trigPin1 = 5;
const int trigPin2 = 3;
const int trigPin3 = 2;

const int echoPin1 = 6;
const int echoPin2 = 9;
const int echoPin3 = 10;

const int outputPin1 = 11;
const int outputPin2 = 12;
const int outputPin3 = 13;

int threshold_distance = 30;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set trigPin as output and echoPin as input
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);

  pinMode(echoPin1, INPUT);
  pinMode(outputPin1, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(outputPin3, OUTPUT);
}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond pulse to trigger
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  // Read the echo time (in microseconds)
  long duration1 = pulseIn(echoPin1, HIGH);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond pulse to trigger
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  long duration2 = pulseIn(echoPin2, HIGH);

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond pulse to trigger
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  long duration3 = pulseIn(echoPin3, HIGH);

  // Calculate distance in cm (speed of sound = 34300 cm/s)
  int distance1 = duration1 * 0.034 / 2;
  int distance2 = duration2 * 0.034 / 2;
  int distance3 = duration3 * 0.034 / 2;

  //Print distance to the Serial Monitor
  // Serial.print("Distance 1: ");
  // Serial.print(distance1);
  // Serial.println(" cm");
  // Serial.print("Distance 2: ");
  // Serial.print(distance2);
  // Serial.println(" cm");
  // Serial.print("Distance 3: ");
  // Serial.print(distance3);
  // Serial.println(" cm");
  // Serial.println("\n");

  if(distance1 > threshold_distance){
    digitalWrite(outputPin1, LOW);
    Serial.println("1: l");
  } else {
    digitalWrite(outputPin1, HIGH);
    Serial.println("1: h");
  }

  if(distance2 > threshold_distance){
    digitalWrite(outputPin2, LOW);
    Serial.println("2: l");
  } else {
    digitalWrite(outputPin2, HIGH);
    Serial.println("2: h");
  }

  if(distance3 > threshold_distance){
    digitalWrite(outputPin3, LOW);
    Serial.println("3: l");
  } else {
    digitalWrite(outputPin3, HIGH);
    Serial.println("3: h");
  }

  // Serial.println(digitalRead(outputPin1));
  // Serial.println(digitalRead(outputPin2));
  // Serial.println(digitalRead(outputPin3));
  Serial.println("\n");
}
