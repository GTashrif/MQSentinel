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

IRSensor irSensor(A0, 10);

void setup() {
  Serial.begin(9600);
  irSensor.begin();
}

void loop() {
  irSensor.update();
  Serial.print("IR Intensity (raw): ");
  Serial.print(irSensor.getRaw());
  Serial.print("   Smoothed: ");
  Serial.println(irSensor.getSmoothed());
  delay(50);
}