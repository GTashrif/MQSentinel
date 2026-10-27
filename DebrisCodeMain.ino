// MOTOR CONTROLLER - DRV8874

class MotorController
{

public:
    MotorController(int in1Pin, int in2Pin, int sleepPin)
        : _in1Pin(in1Pin),
          _in2Pin(in2Pin),
          _sleepPin(sleepPin) {}

    void begin()
    {
        pinMode(_in1Pin, OUTPUT);
        pinMode(_in2Pin, OUTPUT);
        pinMode(_sleepPin, OUTPUT);

        digitalWrite(_sleepPin, HIGH);

        delay(1);

        stop();
    }

    void run(int speed, bool forward)
    {
        speed = constrain(speed, 0, 255);

        digitalWrite(_in2Pin, forward ? HIGH : LOW);

        analogWrite(_in1Pin, speed);
    }

    void stop()
    {
        analogWrite(_in1Pin, 0);

        digitalWrite(_in2Pin, LOW);
    }

private:
    int _in1Pin;
    int _in2Pin;
    int _sleepPin;
};

// MULTIPLEXER (8 CHANNEL ANALOG) - 74HC4051

class Multiplexer
{

public:
    Multiplexer(
        int s0Pin,
        int s1Pin,
        int s2Pin,
        int zPin)
        : _s0(s0Pin),
          _s1(s1Pin),
          _s2(s2Pin),
          _z(zPin) {}

    void begin()
    {
        pinMode(_s0, OUTPUT);
        pinMode(_s1, OUTPUT);
        pinMode(_s2, OUTPUT);

        pinMode(_z, INPUT);
    }

    void selectChannel(byte channel)
    {
        digitalWrite(_s0, (channel & 0x01) ? HIGH : LOW);

        digitalWrite(_s1, (channel & 0x02) ? HIGH : LOW);

        digitalWrite(_s2, (channel & 0x04) ? HIGH : LOW);
    }

    int readChannel(byte channel)
    {

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

// IR SENSOR ANALOG

class IRSensor
{

public:
    static const byte MAX_READINGS = 10;

    IRSensor(
        Multiplexer &mux,
        byte channel,
        byte numReadings = 10)
        : _mux(mux),
          _channel(channel),
          _numReadings(numReadings),
          _readIndex(0),
          _lastRawIndex(0),
          _total(0)
    {

        if (_numReadings > MAX_READINGS)
        {
            _numReadings = MAX_READINGS;
        }

        for (byte i = 0; i < MAX_READINGS; i++)
        {
            _readings[i] = 0;
        }
    }

    void begin()
    {
        int firstReading = _mux.readChannel(_channel);

        _total = 0;

        for (byte i = 0; i < _numReadings; i++){
            _readings[i] = firstReading;
            _total += firstReading;
        }
    }

    void update()
    {
        // Remove oldest reading

        _total -= _readings[_readIndex];

        // Take new reading

        _readings[_readIndex] = _mux.readChannel(_channel);

        // Add newest reading

        _total += _readings[_readIndex];

        _lastRawIndex = _readIndex;

        _readIndex++;

        if (_readIndex >= _numReadings){
            _readIndex = 0;
        }
    }

    int getRaw()
    {

        return _readings[_lastRawIndex];
    }

    int getSmoothed()
    {

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

/*
MOTOR:
IN1 / EN  -> D3
IN2 / PH  -> D6
SLEEP     -> D7
*/

MotorController motor(
    3,
    6,
    7);

/*
MULTIPLEXER:
S0 -> D10
S1 -> D11
S2 -> D12
Z  -> A0
*/

Multiplexer mux(
    10,
    11,
    12,
    A0);

IRSensor ir0(mux, 0, 10);

IRSensor ir1(mux, 1, 10);

IRSensor ir2(mux, 2, 10);

IRSensor ir3(mux, 3, 10);

IRSensor ir4(mux, 4, 10);

IRSensor ir5(mux, 5, 10);

IRSensor ir6(mux, 6, 10);

IRSensor ir7(mux, 7, 10);

const byte NUM_SENSORS = 8;

IRSensor *sensors[NUM_SENSORS] = {

    &ir0,
    &ir1,
    &ir2,
    &ir3,
    &ir4,
    &ir5,
    &ir6,
    &ir7

};

// sensor settings
const int tol = 10;           // ADC: idle noise ceiling, signal is lost at or below this (measure idle)
const int tolMargin = 5;      // extra counts above tol needed to START tracking
const int centerTolPct = 15;  // front pair imbalance (%) that counts as centred (tune)

// position of each sensor in sensors[]
// for me to fill in and measure
const byte FRONT_RIGHT = 0;
const byte FRONT_LEFT = 7;

// bearing of each sensor from the front in degrees, positive = right
// to fill in / for me to measure
const int sensorAngle[NUM_SENSORS] = {23, 68, 113, 158, -158, -113, -68, -23};

// motor settings
const int maxSpeed = 255;
const int fineMaxSpeed = 130;
const int minSpeed = 60;    // lowest PWM that turns the loaded satellite from rest, plus a margin
const bool turnLeft = true;
const bool turnRight = false;

// laser = D8
const int laser = 8;
const unsigned long fireDuration = 2000;

// target must stay centred this long before firing
const unsigned long settleTime = 300;

unsigned long fireStart = 0;
unsigned long centeredSince = 0;
bool centered = false;
bool armed = true;

enum State { SEARCH, TRACK, FIRE };
State state = SEARCH;

int sensorValue[NUM_SENSORS];
int strongestValue = 0;
int strongestSensor = -1;

// read all sensors and find the strongest
void readSensors()
{
    strongestValue = 0;
    strongestSensor = -1;

    for (byte i = 0; i < NUM_SENSORS; i++)
    {
        sensors[i]->update();
        sensorValue[i] = sensors[i]->getSmoothed();

        if (sensorValue[i] > strongestValue)
        {
            strongestValue = sensorValue[i];
            strongestSensor = i;
        }
    }
}


void searchState(){
    motor.stop();
    digitalWrite(laser, LOW);
    centered = false;

    //wait after firing the laser
    if (!armed){
        bool inFront = (strongestSensor == FRONT_LEFT || strongestSensor == FRONT_RIGHT);

        if (strongestValue <= tol || !inFront){
            armed = true;
        }
        else{
            return;
        }
    }

    // higher threshold to start tracking than to stop
    if (strongestValue > tol + tolMargin){
        state = TRACK;
    }
    
}


void trackState(){

    digitalWrite(laser, LOW);

    // no signal case
    if (strongestValue <= tol){
        motor.stop();
        centered = false;
        state = SEARCH;
        return;
    }

    // front case
    if (strongestSensor == FRONT_LEFT || strongestSensor == FRONT_RIGHT){
        long left = sensorValue[FRONT_LEFT];
        long right = sensorValue[FRONT_RIGHT];

        // positive = target on the left
        int error = (int)(100L * (left - right) / (left + right));
        int absError = abs(error);

        // centered
        if (absError <= centerTolPct){
            motor.stop();

            if (!centered){
                centered = true;
                centeredSince = millis();
            }
            else if (millis() - centeredSince >= settleTime){
                centered = false;
                fireStart = millis();
                state = FIRE;
            }
            return;
        }

        centered = false;

        int speed = map(constrain(absError, centerTolPct, 100), centerTolPct, 100, minSpeed, fineMaxSpeed);
        motor.run(speed, error > 0 ? turnLeft : turnRight);
        return;
    }

    // side or behind case
    centered = false;

    int angle = sensorAngle[strongestSensor];
    int speed = map(abs(angle), 0, 180, minSpeed, maxSpeed);
    motor.run(speed, angle > 0 ? turnRight : turnLeft);
}

void fireState(){
    motor.stop();
    digitalWrite(laser, HIGH);

    if (millis() - fireStart >= fireDuration){
        digitalWrite(laser, LOW);
        armed = false;
        state = SEARCH;
    }
}


void setup(){
    motor.begin();
    mux.begin();

    for (byte i = 0; i < NUM_SENSORS; i++){
        sensors[i]->begin();
    }

    pinMode(laser, OUTPUT);
    digitalWrite(laser, LOW);
}

void loop(){
    readSensors();

    switch (state){
    case SEARCH:
        searchState();
        break;
    case TRACK:
        trackState();
        break;
    case FIRE:
        fireState();
        break;
    }
}
