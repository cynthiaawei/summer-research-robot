
const int interruptPin1 = 8; // For outputPin1
const int interruptPin2 = 7; // For outputPin2
const int interruptPin3 = 6; // For outputPin3

volatile bool triggered1 = false;
volatile bool triggered2 = false;
volatile bool triggered3 = false;

void onSignal1() { triggered1 = true; }
void onSignal2() { triggered2 = true; }
void onSignal3() { triggered3 = true; }

void setup() {
  Serial.begin(9600);
  pinMode(interruptPin1, INPUT);
  pinMode(interruptPin2, INPUT);
  pinMode(interruptPin3, INPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPin1), onSignal1, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), onSignal2, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin3), onSignal3, RISING);
}

void loop() {
  if (triggered1) {
    triggered1 = false;
    Serial.println("Interrupt from sensor 1!");
    // Put your reaction here
  }
  if (triggered2) {
    triggered2 = false;
    Serial.println("Interrupt from sensor 2!");
    // Put your reaction here
  }
  if (triggered3) {
    triggered3 = false;
    Serial.println("Interrupt from sensor 3!");
    // Put your reaction here
  }

  Serial.println("not interrupted");
  delay(2000);
}
