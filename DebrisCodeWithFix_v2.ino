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
// MULTIPLEXER - 74HC4051
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


      // Give multiplexer time to settle

      delayMicroseconds(20);


      // Discard first ADC measurement

      analogRead(_z);


      // Return actual measurement

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


      for (
        byte i = 0;
        i < MAX_READINGS;
        i++
      ) {

        _readings[i] = 0;
      }
    }



    void begin() {

      int firstReading =
        _mux.readChannel(_channel);


      _total = 0;


      for (
        byte i = 0;
        i < _numReadings;
        i++
      ) {

        _readings[i] = firstReading;

        _total += firstReading;
      }
    }



    void update() {

      // Remove oldest reading

      _total -=
        _readings[_readIndex];


      // Take new reading

      _readings[_readIndex] =
        _mux.readChannel(_channel);


      // Add newest reading

      _total +=
        _readings[_readIndex];


      _lastRawIndex =
        _readIndex;


      _readIndex++;


      if (
        _readIndex >= _numReadings
      ) {

        _readIndex = 0;
      }
    }



    int getRaw() {

      return
        _readings[_lastRawIndex];
    }



    int getSmoothed() {

      return
        _total / _numReadings;
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
// DRV8874:
//
// IN1 / EN  -> D3
// IN2 / PH  -> D6
// SLEEP     -> D7
//

MotorController motor(
  3,
  6,
  7
);



// ==========================================================
// MULTIPLEXER
// ==========================================================
//
// 74HC4051:
//
// S0 -> D10
// S1 -> D11
// S2 -> D12
// Z  -> A0
//

Multiplexer mux(
  10,
  11,
  12,
  A0
);



// ==========================================================
// SENSOR POSITIONS
// ==========================================================
//
// TOP VIEW:
//
//                       LASER
//                         ↑
//
//                  A7           A0
//
//
//             A6                     A1
//
//
//             A5                     A2
//
//
//                  A4           A3
//
//
// Multiplexer channels:
//
// Y0 -> A0 sensor
// Y1 -> A1 sensor
// Y2 -> A2 sensor
// Y3 -> A3 sensor
// Y4 -> A4 sensor
// Y5 -> A5 sensor
// Y6 -> A6 sensor
// Y7 -> A7 sensor
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
// IR SETTINGS
// ==========================================================


// Minimum value considered an IR signal.
//
// IMPORTANT:
// Adjust this after looking at your actual Serial readings.

int tol = 10;



// Maximum difference between A7 and A0
// before the system considers the target centred.
//
// Example:
//
// A7 = 520
// A0 = 450
//
// Difference = 70
//
// 70 < 120
//
// Therefore TARGET CENTRED.

int centerTol = 120;



// ==========================================================
// MOTOR SPEED SETTINGS
// ==========================================================


// Search speed.
//
// Much slower than your original 255.

const int SEARCH_SPEED = 90;



// Final alignment speed.
//
// Very slow near A7 / A0.

const int ALIGN_SPEED = 45;



// ==========================================================
// MOTOR PULSE SETTINGS
// ==========================================================
//
// IMPORTANT:
//
// The motor DOES NOT stay running.
//
// It runs briefly,
// stops,
// waits,
// and reads sensors again.
// ==========================================================


// Motor ON time while searching

const unsigned long SEARCH_PULSE = 120;


// Motor ON time during final alignment

const unsigned long ALIGN_PULSE = 60;


// Time for satellite to settle after motor stops

const unsigned long SETTLE_TIME = 250;



// ==========================================================
// LASER
// ==========================================================

const int laser = 8;



// Laser ON for 2 seconds

const unsigned long fireDuration =
  2000;



// Prevent immediate repeated firing

const unsigned long cooldownDuration =
  1000;



unsigned long lastFireTime = 0;


bool onCooldown = false;



// ==========================================================
// MOTOR DIRECTION
// ==========================================================
//
// If the satellite moves the WRONG way,
// swap these:
//
// true  -> false
// false -> true
//
// ==========================================================

const bool TURN_TOWARD_LEFT =
  true;


const bool TURN_TOWARD_RIGHT =
  false;



// ==========================================================
// MOTOR PULSE FUNCTION
// ==========================================================
//
// Motor runs only briefly.
//
// Example:
//
// motor ON
//     ↓
// 120 ms
//     ↓
// motor STOP
//     ↓
// wait 250 ms
//     ↓
// read sensors again
//
// ==========================================================

void motorPulse(
  int speed,
  bool direction,
  unsigned long pulseTime
) {


  motor.run(
    speed,
    direction
  );


  delay(
    pulseTime
  );


  // VERY IMPORTANT:
  // Stop motor after every movement.

  motor.stop();


  // Let satellite settle before
  // measuring sensors again.

  delay(
    SETTLE_TIME
  );
}



// ==========================================================
// SETUP
// ==========================================================

void setup() {


  Serial.begin(115200);


  Serial.println();

  Serial.println(
    "=============================================="
  );

  Serial.println(
    "           MQ SENTINEL STARTED"
  );

  Serial.println(
    "=============================================="
  );



  // Start motor system

  motor.begin();



  // Start multiplexer

  mux.begin();



  // Start all IR sensors

  for (
    byte i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    sensors[i]->begin();

  }



  // Laser

  pinMode(
    laser,
    OUTPUT
  );


  digitalWrite(
    laser,
    LOW
  );



  Serial.println(
    "System ready."
  );

  Serial.println(
    "Waiting for IR beacon..."
  );

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

  for (
    byte i = 0;
    i < NUM_SENSORS;
    i++
  ) {


    sensors[i]->update();


    sensorValue[i] =
      sensors[i]->getSmoothed();



    // Find strongest sensor

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
  // PRINT SENSOR VALUES
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



  // Print strongest sensor

  Serial.print(
    "Strongest Sensor: "
  );


  if (
    strongestSensor >= 0
  ) {


    Serial.print("A");

    Serial.println(
      strongestSensor
    );


    Serial.print(
      "Strongest Value: "
    );

    Serial.println(
      strongestValue
    );

  }


  else {


    Serial.println(
      "NONE"
    );

  }



  // ========================================================
  // COOLDOWN CHECK
  // ========================================================


  unsigned long now =
    millis();



  if (
    onCooldown
  ) {


    if (
      now - lastFireTime
      >= cooldownDuration
    ) {


      onCooldown =
        false;


      Serial.println(
        "Cooldown finished."
      );
    }
  }



  // ========================================================
  // NO TARGET
  // ========================================================


  if (
    strongestValue <= tol
  ) {


    // Make absolutely sure motor is stopped.

    motor.stop();


    digitalWrite(
      laser,
      LOW
    );


    Serial.println(
      "Action: NO TARGET"
    );


    Serial.println(
      "Motor: STOP"
    );


    Serial.println(
      "Laser: OFF"
    );


    Serial.println(
      "----------------------------------------------"
    );


    delay(100);


    return;
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



  bool frontLeftDetected =
    leftSignal > tol;


  bool frontRightDetected =
    rightSignal > tol;



  bool frontPairDetected =
    frontLeftDetected &&
    frontRightDetected;



  // ========================================================
  // FIRST CHECK:
  //
  // IS TARGET ALREADY BETWEEN A7 AND A0?
  //
  // This check happens BEFORE any motor movement.
  // ========================================================


  if (
    frontPairDetected &&
    abs(diff) <= centerTol
  ) {


    // ======================================================
    // TARGET CENTRED
    // ======================================================


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



    // Give reaction wheel / satellite
    // additional time to settle.

    delay(300);



    // ======================================================
    // FIRE LASER
    // ======================================================


    if (
      !onCooldown
    ) {


      // Motor stays stopped.

      motor.stop();



      Serial.println(
        "Laser: ON"
      );


      Serial.println();


      Serial.println(
        "TARGET HIT"
      );


      Serial.println(
        "Laser ON for 2 seconds"
      );


      Serial.println();



      digitalWrite(
        laser,
        HIGH
      );



      delay(
        fireDuration
      );



      digitalWrite(
        laser,
        LOW
      );



      // Motor remains stopped.

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


      digitalWrite(
        laser,
        LOW
      );


      motor.stop();


      Serial.println(
        "Laser: OFF - cooldown active"
      );
    }



    Serial.println(
      "----------------------------------------------"
    );


    delay(100);


    return;
  }



  // ========================================================
  // TARGET NEAR FRONT
  //
  // A7 or A0 strongest.
  //
  // Use VERY SHORT motor pulse.
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
    //
    // Make a tiny movement toward LEFT.
    // ======================================================


    if (
      diff > 0
    ) {


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
        "Motor Pulse: "
      );

      Serial.print(
        ALIGN_PULSE
      );

      Serial.println(
        " ms"
      );



      // MOTOR MOVES BRIEFLY THEN STOPS

      motorPulse(
        ALIGN_SPEED,
        TURN_TOWARD_LEFT,
        ALIGN_PULSE
      );



      Serial.println(
        "Motor: STOP"
      );


      Serial.println(
        "Reading sensors again..."
      );
    }



    // ======================================================
    // A0 STRONGER
    //
    // Make a tiny movement toward RIGHT.
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
        "Motor Pulse: "
      );

      Serial.print(
        ALIGN_PULSE
      );

      Serial.println(
        " ms"
      );



      // MOTOR MOVES BRIEFLY THEN STOPS

      motorPulse(
        ALIGN_SPEED,
        TURN_TOWARD_RIGHT,
        ALIGN_PULSE
      );



      Serial.println(
        "Motor: STOP"
      );


      Serial.println(
        "Reading sensors again..."
      );
    }
  }



  // ========================================================
  // TARGET ON RIGHT SIDE
  //
  // A1
  // A2
  // A3
  //
  // Motor does NOT continuously rotate.
  //
  // It makes ONE short movement then stops.
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
      "Motor Pulse: "
    );

    Serial.print(
      SEARCH_PULSE
    );

    Serial.println(
      " ms"
    );



    // Motor ON briefly then STOP

    motorPulse(
      SEARCH_SPEED,
      TURN_TOWARD_RIGHT,
      SEARCH_PULSE
    );



    Serial.println(
      "Motor: STOP"
    );


    Serial.println(
      "Reading sensors again..."
    );
  }



  // ========================================================
  // TARGET ON LEFT SIDE
  //
  // A4
  // A5
  // A6
  //
  // Motor does NOT continuously rotate.
  //
  // It makes ONE short movement then stops.
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
      "Motor Pulse: "
    );

    Serial.print(
      SEARCH_PULSE
    );

    Serial.println(
      " ms"
    );



    // Motor ON briefly then STOP

    motorPulse(
      SEARCH_SPEED,
      TURN_TOWARD_LEFT,
      SEARCH_PULSE
    );



    Serial.println(
      "Motor: STOP"
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
