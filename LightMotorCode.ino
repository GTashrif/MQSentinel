const int IN1_PIN   = 3;
const int IN2_PIN   = 6;
const int SLEEP_PIN = 7;

void setup() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);

  digitalWrite(SLEEP_PIN, HIGH);
  delay(1);

  stopMotor();
}

void loop() {
  setMotor(150, true);
  delay(2000);

  setMotor(150, false);
  delay(2000);

  stopMotor();
  delay(1000);
}

void setMotor(int speed, bool forward) {
  digitalWrite(IN2_PIN, forward ? HIGH : LOW);
  analogWrite(IN1_PIN, speed);
}

void stopMotor() {
  analogWrite(IN1_PIN, 0);
  digitalWrite(IN2_PIN, LOW);
}