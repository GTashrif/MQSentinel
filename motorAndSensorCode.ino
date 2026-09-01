class MotorController {
  public:
    MotorController(int in1Pin, int in2Pin, int sleepPin)
      : _in1Pin(in1Pin), _in2Pin(in2Pin), _sleepPin(sleepPin) {}

    void begin() {
      pinMode(_in1Pin, OUTPUT);
      pinMode(_in2Pin, OUTPUT);
      pinMode(_sleepPin, OUTPUT);

      digitalWrite(_sleepPin, HIGH);
      delay(1);

      stop();
    }

    void run(int speed, bool forward) {
      digitalWrite(_in2Pin, forward ? HIGH : LOW);
      analogWrite(_in1Pin, speed);
    }

    void stop() {
      analogWrite(_in1Pin, 0);
      digitalWrite(_in2Pin, LOW);
    }

  private:
    int _in1Pin;
    int _in2Pin;
    int _sleepPin;
};

class IRSensor {
  public:
    IRSensor(int pin, int numReadings = 10)
      : _pin(pin), _numReadings(numReadings), _readIndex(0), _total(0) {
      _readings = new int[_numReadings];
      for (int i = 0; i < _numReadings; i++) { // 0's out the code to ensure first readings are not incorrect
        _readings[i] = 0;
      }
    }

    ~IRSensor() {
      delete[] _readings;
    }

    void begin() {
      pinMode(_pin, INPUT);
    }

    void update() {
      _total -= _readings[_readIndex];
      _readings[_readIndex] = analogRead(_pin);
      _total += _readings[_readIndex];
      _lastRawIndex = _readIndex;
      _readIndex = (_readIndex + 1) % _numReadings;
    }

    int getRaw() {
      return _readings[_lastRawIndex];
    }

    int getSmoothed() {
      return _total / _numReadings;
    }

  private:
    int _pin;
    int _numReadings;
    int* _readings;
    int _readIndex;
    int _lastRawIndex = 0;
    long _total;
};

MotorController motor(3, 6, 7);  // IN1, IN2, SLEEP pins
IRSensor irSensor1(A0, 10);
IRSensor irSensor2(A1, 10);
int tol = 10;


void setup() {
  motor.begin();
  irSensor1.begin();
  irSensor2.begin();

}

void loop() {
  irSensor1.update();
  irSensor2.update();

  int s1 = irSensor1.getSmoothed();
  int s2 = irSensor2.getSmoothed();

 
  if (s1 > tol || s2 > tol) { // signal detected
    int diff = getDiff(s1, s2);

    if (absVal(diff) <= tol) { // centered dont spin
      motor.stop();
    } else if (diff > 0) { // detected left spin clockwise
      motor.run(150, true);
    } else { // detected rigtht spin counter clockwise
      motor.run(150, false);
    }
  } else { // no signal at all
    motor.stop();
  }

  delay(100);
}

int getDiff(int x, int y) {
  return x - y;
}

int absVal(int x) {
  if (x < 0) {
    return (x + (x * -2));
  } else {
    return x;
  }
}