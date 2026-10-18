#include <Arduino.h>


// ==========================================================
// MOTOR CONTROLLER
// ==========================================================

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

      digitalWrite(
        _in2Pin,
        forward ? HIGH : LOW
      );

      analogWrite(
        _in1Pin,
        speed
      );
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


// ==========================================================
// 74HC4051 MULTIPLEXER
// ==========================================================

class Multiplexer {
  public:

    Multiplexer(
      int s0Pin,
      int s1Pin,
      int s2Pin,
      int zPin
    )
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

      digitalWrite(
        _s0,
        (channel & 0x01) ? HIGH : LOW
      );

      digitalWrite(
        _s1,
        (channel & 0x02) ? HIGH : LOW
      );

      digitalWrite(
        _s2,
        (channel & 0x04) ? HIGH : LOW
      );
    }

    int readChannel(byte channel) {

      selectChannel(channel);

      // Allow multiplexer to settle
      delayMicroseconds(20);

      // Throw away first ADC reading
      analogRead(_z);

      // Actual reading
      return analogRead(_z);
    }

  private:

    int _s0;
    int _s1;
    int _s2;
    int _z;
};


// ==========================================================
// IR SENSOR
// ==========================================================

class IRSensor {
  public:

    static const byte MAX_READINGS = 10;

    IRSensor(
      Multiplexer &mux,
      byte channel,
      byte numReadings = 10
    )
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

      int firstReading =
        _mux.readChannel(_channel);

      _total = 0;

      // Fill smoothing buffer
      for (byte i = 0; i < _numReadings; i++) {

        _readings[i] = firstReading;

        _total += firstReading;
      }
    }

    void update() {

      // Remove oldest reading
      _total -= _readings[_readIndex];

      // Get new reading
      _readings[_readIndex] =
        _mux.readChannel(_channel);

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


// ==========================================================
// MOTOR
// ==========================================================
//
// IN1 = D3
// IN2 = D6
// SLEEP = D7
//

MotorController motor(3, 6, 7);


// ==========================================================
// MULTIPLEXER
// ==========================================================
//
// S0 = D10
// S1 = D11
// S2 = D12
// Z  = Arduino A0
//

Multiplexer mux(10, 11, 12, A0);


// ==========================================================
// IR SENSOR ARRANGEMENT
// ==========================================================
//
// TOP VIEW
//
//                         LASER
//                           ↑
//
//                    A7           A0
//
//
//               A6                   A1
//
//
//               A5                   A2
//
//
//                    A4           A3
//
//
// Multiplexer:
//
// Y0 = sensor A0
// Y1 = sensor A1
// Y2 = sensor A2
// Y3 = sensor A3
// Y4 = sensor A4
// Y5 = sensor A5
// Y6 = sensor A6
// Y7 = sensor A7
//
// ==========================================================

IRSensor ir0(mux, 0, 10);
IRSensor ir1(mux, 1, 10);
IRSensor ir2(mux, 2, 10);
IRSensor ir3(mux, 3, 10);

IRSensor ir4(mux, 4, 10);
IRSensor ir5(mux, 5, 10);
IRSensor ir6(mux, 6, 10);
IRSensor ir7(mux, 7, 10);


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


// ==========================================================
// IR DETECTION SETTINGS
// ==========================================================

// Anything BELOW 100 is ignored.
//
// 0-99 = NO VALID IR SIGNAL
// 100+ = valid reading

const int DETECTION_THRESHOLD = 100;


// Difference allowed between A7 and A0.
//
// Example:
//
// A7 = 520
// A0 = 450
//
// difference = 70
//
// 70 <= 120
//
// Target considered centred.

const int CENTER_TOLERANCE = 120;


// ==========================================================
// MOTOR SPEED SETTINGS
// ==========================================================

// Search speed for A1-A6

const int SEARCH_SPEED = 90;


// Much slower near A7/A0

const int ALIGN_SPEED = 45;


// ==========================================================
// MOTOR PULSE SETTINGS
// ==========================================================

// Motor ON duration while searching

const unsigned long SEARCH_PULSE = 120;


// Motor ON duration near A7/A0

const unsigned long ALIGN_PULSE = 60;


// Wait after EVERY pulse before reading sensors again

const unsigned long SETTLE_TIME = 250;


// ==========================================================
// LASER SETTINGS
// ==========================================================

const int laser = 8;


// Laser stays on for 2 seconds

const unsigned long FIRE_DURATION = 2000;


// Prevent immediate re-fire

const unsigned long COOLDOWN_DURATION = 1000;


unsigned long lastFireTime = 0;

bool onCooldown = false;


// ==========================================================
// MOTOR DIRECTION
// ==========================================================
//
// If your satellite moves the WRONG direction,
// swap these:
//
// LEFT  = false
// RIGHT = true
//
// ==========================================================

const bool TURN_TOWARD_LEFT = true;

const bool TURN_TOWARD_RIGHT = false;


// ==========================================================
// MOTOR PULSE FUNCTION
// ==========================================================
//
// Instead of continuously spinning:
//
// Motor ON
// ↓
// short pulse
// ↓
// Motor STOP
// ↓
// wait 150 ms
// ↓
// read sensors again
//
// ==========================================================

void motorPulse(
  int speed,
  bool direction,
  unsigned long pulseTime
) {

  // Start movement
  motor.run(
    speed,
    direction
  );

  // Run only for short period
  delay(pulseTime);

  // STOP motor
  motor.stop();

  // Allow satellite to settle
  delay(SETTLE_TIME);
}


// ==========================================================
// SETUP
// ==========================================================

void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("          MQ SENTINEL STARTED");
  Serial.println("==============================================");


  // Start motor
  motor.begin();


  // Start multiplexer
  mux.begin();


  // Start all IR sensors
  for (byte i = 0; i < NUM_SENSORS; i++) {

    sensors[i]->begin();

  }


  // Laser
  pinMode(laser, OUTPUT);

  digitalWrite(laser, LOW);


  Serial.println("System ready.");
  Serial.println("Motor will stay stopped until IR >= 100.");
  Serial.println("Waiting for IR ray...");
  Serial.println("==============================================");
}


// ==========================================================
// MAIN LOOP
// ==========================================================

void loop() {

  sensorLogic();

}


// ==========================================================
// SENSOR / TRACKING LOGIC
// ==========================================================

void sensorLogic() {

  int sensorValue[NUM_SENSORS];


  int strongestValue = 0;

  int strongestSensor = -1;


  // ========================================================
  // READ ALL 8 SENSORS
  // ========================================================

  for (byte i = 0; i < NUM_SENSORS; i++) {

    sensors[i]->update();

    sensorValue[i] =
      sensors[i]->getSmoothed();


    // ------------------------------------------------------
    // IGNORE EVERYTHING BELOW 100
    // ------------------------------------------------------

    if (
      sensorValue[i] <
      DETECTION_THRESHOLD
    ) {

      continue;
    }


    // ------------------------------------------------------
    // FIND STRONGEST VALID SENSOR
    // ------------------------------------------------------

    if (
      sensorValue[i] >
      strongestValue
    ) {

      strongestValue =
        sensorValue[i];

      strongestSensor =
        i;
    }
  }


  // ========================================================
  // PRINT ALL SENSOR READINGS
  // ========================================================

  Serial.print("A0: ");
  Serial.print(sensorValue[0]);

  Serial.print("   A1: ");
  Serial.print(sensorValue[1]);

  Serial.print("   A2: ");
  Serial.print(sensorValue[2]);

  Serial.print("   A3: ");
  Serial.print(sensorValue[3]);

  Serial.print("   A4: ");
  Serial.print(sensorValue[4]);

  Serial.print("   A5: ");
  Serial.print(sensorValue[5]);

  Serial.print("   A6: ");
  Serial.print(sensorValue[6]);

  Serial.print("   A7: ");
  Serial.println(sensorValue[7]);


  // ========================================================
  // NO VALID IR RAY
  // ========================================================

  if (strongestSensor == -1) {

    // MOTOR MUST REMAIN STOPPED

    motor.stop();

    digitalWrite(
      laser,
      LOW
    );


    Serial.println(
      "Strongest Sensor: NONE"
    );

    Serial.println(
      "IR STATUS: NO VALID RAY"
    );

    Serial.println(
      "Motor: STOP"
    );

    Serial.println(
      "Waiting for IR >= 100..."
    );

    Serial.println(
      "----------------------------------------------"
    );


    delay(50);

    return;
  }


  // ========================================================
  // VALID IR RAY DETECTED
  // ========================================================

  Serial.print(
    "Strongest Sensor: A"
  );

  Serial.println(
    strongestSensor
  );


  Serial.print(
    "Strongest Value: "
  );

  Serial.println(
    strongestValue
  );


  Serial.println(
    "IR STATUS: VALID RAY DETECTED"
  );


  // ========================================================
  // COOLDOWN
  // ========================================================

  unsigned long now =
    millis();


  if (onCooldown) {

    if (
      now - lastFireTime >=
      COOLDOWN_DURATION
    ) {

      onCooldown = false;

      Serial.println(
        "Cooldown finished."
      );
    }
  }


  // ========================================================
  // FRONT SENSOR VALUES
  // ========================================================

  int leftSignal =
    sensorValue[7];


  int rightSignal =
    sensorValue[0];


  int diff =
    leftSignal -
    rightSignal;


  // A7 must be >= 100 to count

  bool frontLeftDetected =
    leftSignal >=
    DETECTION_THRESHOLD;


  // A0 must be >= 100 to count

  bool frontRightDetected =
    rightSignal >=
    DETECTION_THRESHOLD;


  // Both sensors must detect valid IR

  bool frontPairDetected =
    frontLeftDetected &&
    frontRightDetected;


  // ========================================================
  // CHECK WHETHER TARGET IS CENTRED
  //
  // This happens BEFORE any motor movement.
  // ========================================================

  if (
    frontPairDetected &&
    abs(diff) <= CENTER_TOLERANCE
  ) {

    // STOP MOTOR COMPLETELY

    motor.stop();


    digitalWrite(
      laser,
      LOW
    );


    Serial.print(
      "Front Left A7: "
    );

    Serial.println(
      leftSignal
    );


    Serial.print(
      "Front Right A0: "
    );

    Serial.println(
      rightSignal
    );


    Serial.print(
      "Difference: "
    );

    Serial.println(
      diff
    );


    Serial.println(
      "Action: TARGET CENTRED"
    );


    Serial.println(
      "Motor: STOP"
    );


    Serial.println(
      "Target is between A7 and A0."
    );


    // Allow satellite to settle
    delay(300);


    // ======================================================
    // FIRE LASER
    // ======================================================

    if (!onCooldown) {

      // Ensure motor remains stopped
      motor.stop();


      Serial.println(
        "Laser: ON"
      );

      Serial.println(
        "TARGET HIT - LASER ON FOR 2 SECONDS"
      );


      digitalWrite(
        laser,
        HIGH
      );


      delay(
        FIRE_DURATION
      );


      digitalWrite(
        laser,
        LOW
      );


      motor.stop();


      Serial.println(
        "Laser: OFF"
      );


      Serial.println(
        "Cooldown started..."
      );


      lastFireTime =
        millis();


      onCooldown =
        true;
    }

    else {

      motor.stop();

      digitalWrite(
        laser,
        LOW
      );


      Serial.println(
        "Laser: OFF - cooldown active"
      );
    }


    Serial.println(
      "----------------------------------------------"
    );


    delay(50);

    return;
  }


  // ========================================================
  // TARGET NEAR FRONT
  //
  // A7 or A0 strongest
  //
  // Use very small movement pulse.
  // ========================================================

  if (
    strongestSensor == 7 ||
    strongestSensor == 0
  ) {

    Serial.print(
      "Front Left A7: "
    );

    Serial.println(
      leftSignal
    );


    Serial.print(
      "Front Right A0: "
    );

    Serial.println(
      rightSignal
    );


    Serial.print(
      "Difference: "
    );

    Serial.println(
      diff
    );


    // ======================================================
    // A7 STRONGER
    // ======================================================

    if (diff > 0) {

      digitalWrite(
        laser,
        LOW
      );


      Serial.println(
        "Action: SMALL ALIGNMENT LEFT"
      );


      Serial.print(
        "Motor Speed: "
      );

      Serial.println(
        ALIGN_SPEED
      );


      Serial.print(
        "Motor ON: "
      );

      Serial.print(
        ALIGN_PULSE
      );

      Serial.println(
        " ms"
      );


      // Short movement only
      motorPulse(
        ALIGN_SPEED,
        TURN_TOWARD_LEFT,
        ALIGN_PULSE
      );


      Serial.println(
        "Motor: STOP"
      );


      Serial.println(
        "Waited 150 ms."
      );


      Serial.println(
        "Reading sensors again..."
      );
    }


    // ======================================================
    // A0 STRONGER
    // ======================================================

    else {

      digitalWrite(
        laser,
        LOW
      );


      Serial.println(
        "Action: SMALL ALIGNMENT RIGHT"
      );


      Serial.print(
        "Motor Speed: "
      );

      Serial.println(
        ALIGN_SPEED
      );


      Serial.print(
        "Motor ON: "
      );

      Serial.print(
        ALIGN_PULSE
      );

      Serial.println(
        " ms"
      );


      // Short movement only
      motorPulse(
        ALIGN_SPEED,
        TURN_TOWARD_RIGHT,
        ALIGN_PULSE
      );


      Serial.println(
        "Motor: STOP"
      );


      Serial.println(
        "Waited 150 ms."
      );


      Serial.println(
        "Reading sensors again..."
      );
    }
  }


  // ========================================================
  // TARGET ON RIGHT SIDE
  //
  // A1 / A2 / A3
  // ========================================================

  else if (
    strongestSensor >= 1 &&
    strongestSensor <= 3
  ) {

    digitalWrite(
      laser,
      LOW
    );


    Serial.println(
      "Action: SHORT ROTATION RIGHT"
    );


    Serial.print(
      "Motor Speed: "
    );

    Serial.println(
      SEARCH_SPEED
    );


    Serial.print(
      "Motor ON: "
    );

    Serial.print(
      SEARCH_PULSE
    );

    Serial.println(
      " ms"
    );


    // One short pulse only

    motorPulse(
      SEARCH_SPEED,
      TURN_TOWARD_RIGHT,
      SEARCH_PULSE
    );


    Serial.println(
      "Motor: STOP"
    );


    Serial.println(
      "Waited 150 ms."
    );


    Serial.println(
      "Reading sensors again..."
    );
  }


  // ========================================================
  // TARGET ON LEFT SIDE
  //
  // A4 / A5 / A6
  // ========================================================

  else if (
    strongestSensor >= 4 &&
    strongestSensor <= 6
  ) {

    digitalWrite(
      laser,
      LOW
    );


    Serial.println(
      "Action: SHORT ROTATION LEFT"
    );


    Serial.print(
      "Motor Speed: "
    );

    Serial.println(
      SEARCH_SPEED
    );


    Serial.print(
      "Motor ON: "
    );

    Serial.print(
      SEARCH_PULSE
    );

    Serial.println(
      " ms"
    );


    // One short pulse only

    motorPulse(
      SEARCH_SPEED,
      TURN_TOWARD_LEFT,
      SEARCH_PULSE
    );


    Serial.println(
      "Motor: STOP"
    );


    Serial.println(
      "Waited 150 ms."
    );


    Serial.println(
      "Reading sensors again..."
    );
  }


  // ========================================================
  // END OF LOOP
  // ========================================================

  Serial.println(
    "----------------------------------------------"
  );


  delay(50);
}
