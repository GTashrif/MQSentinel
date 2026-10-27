#include <Arduino.h>

// MOTOR CONTROLLER
class MotorController {
  public:
    MotorController(int in1Pin, int in2Pin, int sleepPin)
      : _in1Pin(in1Pin),
        _in2Pin(in2Pin),
        _sleepPin(sleepPin) {}

    void begin() {
      pinMode(_in1Pin, OUTPUT);
      pinMode(_in2Pin, OUTPUT);
      pinMode(_sleepPin, OUTPUT);

      digitalWrite(_sleepPin, HIGH);
      delay(1);

      stop();
    }

    void run(int speed, bool forward) {
      speed = constrain(speed, 0, 255);

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


// MULTIPLEXER (74HC4051)
class Multiplexer {
  public:
    Multiplexer(int s0Pin, int s1Pin, int s2Pin, int zPin)
      : _s0(s0Pin),
        _s1(s1Pin),
        _s2(s2Pin),
        _z(zPin) {}

    void begin() {
      pinMode(_s0, OUTPUT);
      pinMode(_s1, OUTPUT);
      pinMode(_s2, OUTPUT);
      pinMode(_z, INPUT);
    }

    void selectChannel(byte channel) {
      digitalWrite(_s0, (channel & 0x01) ? HIGH : LOW);
      digitalWrite(_s1, (channel & 0x02) ? HIGH : LOW);
      digitalWrite(_s2, (channel & 0x04) ? HIGH : LOW);
    }

    int readChannel(byte channel) {
      selectChannel(channel);
      delayMicroseconds(10);   // settle time after switching channels

      analogRead(_z);          // throwaway read so the ADC settles on the new channel
      return analogRead(_z);
    }

  private:
    int _s0, _s1, _s2, _z;
};


// IR SENSOR
class IRSensor {
  public:

    static const byte MAX_READINGS = 10;

    IRSensor(Multiplexer &mux, byte channel, byte numReadings = 10)
      : _mux(mux),
        _channel(channel),
        _numReadings(numReadings),
        _readIndex(0),
        _lastRawIndex(0),
        _total(0) {

      if (_numReadings > MAX_READINGS) {
        _numReadings = MAX_READINGS;
      }

      for (byte i = 0; i < MAX_READINGS; i++) {
        _readings[i] = 0;
      }
    }

    void begin() {

      // Sensor is read through the multiplexer, so no pinMode is needed here.
      // The multiplexer's begin() sets up the pins.

      // Fill moving-average buffer with an initial reading
      // so we do not have to wait for 10 samples at startup.
      int firstReading = _mux.readChannel(_channel);

      _total = 0;

      for (byte i = 0; i < _numReadings; i++) {
        _readings[i] = firstReading;
        _total += firstReading;
      }
    }

    void update() {

      // Remove oldest reading
      _total -= _readings[_readIndex];

      // Take new reading
      _readings[_readIndex] = _mux.readChannel(_channel);

      // Add new reading
      _total += _readings[_readIndex];
      _lastRawIndex = _readIndex;
      _readIndex++;

      if (_readIndex >= _numReadings) {
        _readIndex = 0;
      }
    }

    int getRaw() {
      return _readings[_lastRawIndex];
    }

    int getSmoothed() {
      return _total / _numReadings;
    }

  private:

    Multiplexer &_mux;
    byte _channel;
    byte _numReadings;

    int _readings[MAX_READINGS];

    byte _readIndex;
    byte _lastRawIndex;

    long _total;
};


// MOTOR
MotorController motor(3, 6, 7);

// MULTIPLEXER
// S0, S1, S2, Z (Z goes to analog pin A0)
Multiplexer mux(10, 11, 12, A0);

// 8 IR SENSORS
// Clockwise around satellite when viewed from above.
// Each sensor is on its own multiplexer channel (Y0-Y7).
IRSensor ir0(mux, 0, 10);   // Front-right
IRSensor ir1(mux, 1, 10);   // Right-front
IRSensor ir2(mux, 2, 10);   // Right-rear
IRSensor ir3(mux, 3, 10);   // Rear-right

IRSensor ir4(mux, 4, 10);   // Rear-left
IRSensor ir5(mux, 5, 10);   // Left-rear
IRSensor ir6(mux, 6, 10);   // Left-front
IRSensor ir7(mux, 7, 10);   // Front-left


const byte NUM_SENSORS = 8;

IRSensor* sensors[NUM_SENSORS] = {
  &ir0,
  &ir1,
  &ir2,
  &ir3,
  &ir4,
  &ir5,
  &ir6,
  &ir7
};



// SETTINGS
// Minimum IR signal required before we say a target exists
int tol = 10;


// Difference allowed between the two front sensors before the target is considered centred.
int centerTol = 100;


// Full speed when target is far away from the front
const int SEARCH_SPEED = 255;


// Slower speed when doing final alignment. This will help to prevent overshooting the target.
const int ALIGN_SPEED = 140;


// Laser pin
const int laser = 8;


// Laser must stay ON for 2 seconds
const unsigned long fireDuration = 2000;


// Preventing immediate repeated firing
const unsigned long cooldownDuration = 1000;


unsigned long lastFireTime = 0;

bool onCooldown = false;


// MOTOR DIRECTION SETTINGS

// These follow the behaviour of your original 2-sensor code.
//
// LEFT sensor stronger  -> motor.run(..., true)
// RIGHT sensor stronger -> motor.run(..., false)
//
// !!!!!!! If your satellite rotates in the opposite direction, we need to swap the true and false !!!!!!!!

const bool TURN_TOWARD_LEFT  = true;
const bool TURN_TOWARD_RIGHT = false;


// SETUP
void setup() {

  motor.begin();

  // Multiplexer must be started before the sensors, since sensors read through it
  mux.begin();

  for (byte i = 0; i < NUM_SENSORS; i++) {
    sensors[i]->begin();
  }

  pinMode(laser, OUTPUT);

  digitalWrite(laser, LOW);
}


// MAIN LOOP
void loop() {

  sensorLogic();

}

// SENSOR / TRACKING LOGIC
void sensorLogic() {

  int sensorValue[NUM_SENSORS];

  int strongestValue = 0;
  int strongestSensor = -1;

  // Read all 8 sensors
  for (byte i = 0; i < NUM_SENSORS; i++) {

    sensors[i]->update();

    sensorValue[i] = sensors[i]->getSmoothed();


    // Find which sensor sees the strongest IR signal
    if (sensorValue[i] > strongestValue) {

      strongestValue = sensorValue[i];
      strongestSensor = i;

    }
  }


  unsigned long now = millis();

  // Cooldown management
  if (onCooldown) {

    if (now - lastFireTime >= cooldownDuration) {

      onCooldown = false;

    }
  }


  // NO IR TARGET
  if (strongestValue <= tol) {

    motor.stop();

    digitalWrite(laser, LOW);

    delay(20);

    return;
  }


  // TARGET IS IN FRONT
  // Front is between:
  // Sensor 7 = front-left
  // Sensor 0 = front-right

  if (strongestSensor == 7 || strongestSensor == 0) {

    int leftSignal  = sensorValue[7];

    int rightSignal = sensorValue[0];


    int diff = leftSignal - rightSignal;

    // TARGET CENTRED
    if (abs(diff) <= centerTol) {

      motor.stop();


      // Only fire if cooldown has finished
      if (!onCooldown) {
        digitalWrite(laser, HIGH);
        // Laser remains continuously ON for 2 seconds
        delay(fireDuration);
        digitalWrite(laser, LOW);
        lastFireTime = millis();
        onCooldown = true;

      }

      else {

        digitalWrite(laser, LOW);

      }
    }

    // TARGET SLIGHTLY LEFT
    else if (diff > 0) {
      digitalWrite(laser, LOW);
      motor.run(
        ALIGN_SPEED,
        TURN_TOWARD_LEFT
      );

    }


    // TARGET SLIGHTLY RIGHT
    else {
      digitalWrite(laser, LOW);

      motor.run(
        ALIGN_SPEED,
        TURN_TOWARD_RIGHT
      );

    }
  }


  // TARGET ON RIGHT SIDE
  else if (
    strongestSensor >= 1 &&
    strongestSensor <= 3
  ) {

    digitalWrite(laser, LOW);

    motor.run(
      SEARCH_SPEED,
      TURN_TOWARD_RIGHT
    );

  }


  // TARGET ON LEFT SIDE
  else if (
    strongestSensor >= 4 &&
    strongestSensor <= 6
  ) {

    digitalWrite(laser, LOW);

    motor.run(
      SEARCH_SPEED,
      TURN_TOWARD_LEFT
    );

  }


  // Faster loop than the original 100 ms.
  delay(20);
}
