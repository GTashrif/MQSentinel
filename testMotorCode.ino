// DRV8874/8876 Motor Driver + Hall Encoder — Arduino Nano
// Final pin mapping:
const int IN1_PIN   = 3;   // EN/IN1 - PWM speed control (PHASE/ENABLE mode)
const int IN2_PIN   = 6;   // PH/IN2 - direction control
const int FAULT_PIN = 5;   // FAULT - active LOW, open-drain (needs pull-up)
const int CS_PIN    = A7;  // Current sense analog input
const int SLEEP_PIN = 7;   // SLEEP - must be HIGH to enable driver

const int ENCODER_A = 2;   // Hall Sensor A Vout (interrupt-capable pin)
const int ENCODER_B = 4;   // Hall Sensor B Vout

// PMODE: hardwired LOW to common GND (selects PHASE/ENABLE mode)
// VREF:  hardwired to regulated 5V rail (from AJ25 regulator)

volatile long encoderCount = 0;

void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(FAULT_PIN, INPUT_PULLUP);
  pinMode(CS_PIN, INPUT);
  pinMode(SLEEP_PIN, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  digitalWrite(SLEEP_PIN, HIGH);  // Enable driver — must be set before PMODE latches
  delay(1);

  stopMotor();
}

void loop() {
  setMotor(125, true);   
  delay(2000);
  checkFault();

  setMotor(125, false);  // reverse
  delay(2000);
  checkFault();

  stopMotor();
  delay(1000);

  Serial.print("Encoder count: ");
  Serial.print(encoderCount);
  Serial.print("  |  Current sense (raw): ");
  Serial.println(analogRead(CS_PIN));

  delay(500);
}

void setMotor(int speed, bool forward) {
  digitalWrite(IN2_PIN, forward ? HIGH : LOW);
  analogWrite(IN1_PIN, speed);
}

void stopMotor() {
  analogWrite(IN1_PIN, 0);
  digitalWrite(IN2_PIN, LOW);
}

void checkFault() {
  if (digitalRead(FAULT_PIN) == LOW) {
    Serial.println("FAULT DETECTED!");
    stopMotor();
  }
}

void encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}