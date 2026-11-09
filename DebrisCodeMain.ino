#include <Wire.h>

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

// GYROSCOPE - MPU-6050, READS ROTATION ABOUT ITS X AXIS

class Gyro
{

public:
    Gyro(byte address, int sign)
        : _address(address),
          _sign(sign),
          _bias(0),
          _heading(0),
          _rate(0),
          _lastMicros(0) {}

    // wake the sensor and set its range and filter
    bool begin()
    {
        if (!writeRegister(0x6B, 0x01)) return false; // wake, use gyro clock
        delay(50);                                    // let the gyro start up
        if (!writeRegister(0x19, 0x00)) return false; // no extra sample divider
        if (!writeRegister(0x1A, 0x03)) return false; // about 42 Hz low-pass filter
        if (!writeRegister(0x1B, 0x08)) return false; // +/- 500 deg/s range

        resetTiming();
        return true;
    }

    // measure the reading at rest. The satellite must be still.
    bool calibrate()
    {
        for (byte attempt = 0; attempt < CALIBRATION_ATTEMPTS; attempt++)
        {
            long sum = 0;
            int lowest = 32767;
            int highest = -32768;

            for (int i = 0; i < CALIBRATION_SAMPLES; i++)
            {
                int raw;
                if (!readRaw(raw)) return false;

                sum += raw;
                if (raw < lowest) lowest = raw;
                if (raw > highest) highest = raw;
                delay(2);
            }

            _bias = (float)sum / CALIBRATION_SAMPLES;

            // a small spread means it was still, so stop trying
            if (highest - lowest <= STILL_RANGE) break;
        }

        _heading = 0;
        resetTiming();
        return true;
    }

    // add the rotation since the last good reading to the heading
    bool update()
    {
        int raw;
        if (!readRaw(raw)) return false;

        unsigned long now = micros();
        float seconds = (now - _lastMicros) * 0.000001;
        _lastMicros = now;

        _rate = _sign * (raw - _bias) / COUNTS_PER_DEG_PER_SEC;
        _heading += _rate * seconds;
        return true;
    }

    void resetTiming()
    {
        _lastMicros = micros();
    }

    // degrees turned since calibration, positive = right
    float heading()
    {
        return _heading;
    }

    // degrees per second right now, positive = right
    float rate()
    {
        return _rate;
    }

private:
    static const byte CALIBRATION_ATTEMPTS = 5;
    static const int CALIBRATION_SAMPLES = 500;
    static const int STILL_RANGE = 100;
    static constexpr float COUNTS_PER_DEG_PER_SEC = 65.5;

    bool writeRegister(byte reg, byte value)
    {
        Wire.beginTransmission(_address);
        Wire.write(reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    bool readRaw(int &raw)
    {
        Wire.beginTransmission(_address);
        Wire.write(0x43); // GYRO_XOUT_H, the axis along the satellite's turning axis
        if (Wire.endTransmission(false) != 0) return false;

        if (Wire.requestFrom(_address, (byte)2) != 2) return false;

        int high = Wire.read();
        int low = Wire.read();
        raw = (int16_t)(((uint16_t)high << 8) | low);
        return true;
    }

    byte _address;
    int _sign;
    float _bias;
    float _heading;
    float _rate;
    unsigned long _lastMicros;
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

// sensor s0 to s7 are wired to Y0 to Y7
IRSensor ir0(mux, 0, 10);
IRSensor ir1(mux, 1, 10);
IRSensor ir2(mux, 2, 10);
IRSensor ir3(mux, 3, 10);
IRSensor ir4(mux, 4, 10);
IRSensor ir5(mux, 5, 10);
IRSensor ir6(mux, 6, 10);
IRSensor ir7(mux, 7, 10);

const byte NUM_SENSORS = 8;

IRSensor *sensors[NUM_SENSORS] = {&ir0, &ir1, &ir2, &ir3, &ir4, &ir5, &ir6, &ir7};

/*
GYROSCOPE:
SDA -> A4
SCL -> A5
Mounted with the X arrow pointing up, along the turning axis.
GYRO_SIGN = 1 makes clockwise from above positive (checked with g in the test code).
Power on with the satellite resting still on the bench, then hang it.
*/

const byte GYRO_ADDRESS = 0x68;
const int GYRO_SIGN = 1;

Gyro gyro(GYRO_ADDRESS, GYRO_SIGN);

const bool DEBUG_LOG = false;          // prints what the code is doing, 5 times a second
const bool CALIBRATE_GYRO = true;      // keep true. False skips calibration and the heading drifts
const bool USE_INTERNAL_REF = true;    // 1.1 V reference: about 4.5x larger readings. Nothing may be wired to AREF

// ---------------- SETTINGS ----------------

// sensors
const int tol = 10;                    // a reading above this counts as seeing the beacon
const int tolMargin = 5;               // extra needed to enter tracking case

// s0 is right of the laser, s7 is left of it. Bearing of each sensor from the laser, positive = right
const byte FRONT_RIGHT = 0;
const byte FRONT_LEFT = 7;
const float sensorAngle[NUM_SENSORS] = {22.5, 67.5, 112.5, 157.5, -157.5, -112.5, -67.5, -22.5};

const bool turnLeft = true;
const bool turnRight = false;
const int deadbandPwm = 30;            // PWM where the flywheel just starts to move
const int maxSpeed = 255;              // most PWM allowed
const int holdMaxPwm = 255;            // most PWM allowed while holding on the beacon

// PID: the outer part turns "degrees off target" into a wanted turning rate,
// the inner part changes the flywheel speed until the body turns at that rate
const float angleKp = 1.5;             // wanted deg/s per degree off target
const float rateKi = 15.0;             // PWM per second added per deg/s of rate error (integral)
const float rateKp = 2.0;              // PWM per deg/s of rate error (proportional)
const float rateFilter = 0.3;          // light smoothing of the measured rate, smaller = smoother
const float rampBand = 15.0;           // output below this ramps smoothly from 0, so there's no kick when it changes direction
const float turnMaxRate = 40.0;        // fastest turn, deg/s
const float holdMaxRate = 15.0;        // fastest correction while holding, deg/s
const float sweepRate = 20.0;          // slow turn through the gap between s0 and s7, deg/s

// flywheel at full speed but the body has stopped short (friction or thread twist used up the push):
// slow the flywheel down gently, then push again
const unsigned long stallTime = 400;   // ms at full power without moving before unwinding
const float unwindRate = 60.0;         // PWM per second the flywheel slows by while unwinding
const float idleRampRate = 150.0;      // PWM per second the flywheel slows by when stopping normally

// gap between s0 and s7
const float maxFrontGap = 45.0;        // furthest to sweep past the last sighting, degrees
const float frontGapGuess = 20.0;      // used only if the other front sensor never sees it

// firing
const float holdTol = 5.0;             // degrees off target allowed before firing
const float lateHoldTol = 8.0;         // allowed once it has been holding for lateHoldTime without managing holdTol
const unsigned long lateHoldTime = 2500;
const float holdRateTol = 10.0;        // deg/s allowed before firing (averaged, so motor vibration doesn't count)
const unsigned long settleTime = 250;  // ms it must stay settled before firing
const unsigned long holdTimeout = 5000;// ms to try settling before looking again
const unsigned long fireDuration = 2000;
const float fireTol = 10.0;            // degrees: if the aim drifts further than this while firing, laser off and settle again

// timeouts
const unsigned long signalLostTimeout = 5000;  // ms with no signal before giving up
const unsigned long sideSeenTime = 200;        // ms a side sensor must clearly lead to leave HOLD or SWEEP
const unsigned long gyroTimeout = 100;         // ms of failed gyro reads before stopping
const unsigned long runawayTime = 2500;        // ms turning the wrong way before stopping
const float runawayRate = 100.0;               // deg/s. The code never asks for more than 40, so faster than this is a fault
const unsigned long runawayFastTime = 1000;    // ms above runawayRate before stopping
const unsigned long runawayPause = 3000;       // ms to stay stopped (LED on) after a runaway

// laser = D8
const int laser = 8;

// ---------------- STATE ----------------

enum State { SEARCH, TURN, SWEEP, HOLD, FIRE };
State state = SEARCH;

int sensorValue[NUM_SENSORS];
int strongestValue = 0;
int strongestSensor = -1;
int ambient = 0;                   // room light, taken off every reading

float target = 0;                  // heading to point the laser at
float lastSeenHeading = 0;         // SWEEP: last heading where the first front sensor saw it
int sweepSensor = -1;              // SWEEP: the front sensor that saw it first
int sweepDir = 1;                  // SWEEP: +1 = turning right, -1 = turning left

float ctlCommand = 0;              // signed PWM above the deadband, positive = turn right
unsigned long ctlMicros = 0;
float smoothRate = 0;              // gyro rate with the noise lightly smoothed
int lastPwm = 0;
bool lastDirection = true;
unsigned long wrongWaySince = 0;
bool unwinding = false;            // slowing the flywheel down after it ran out of push
unsigned long stallSince = 0;
float guardRate = 0;               // smoothed rate for the runaway check
float settleRate = 0;              // heavily smoothed rate for deciding it has stopped
bool okFront = false, okAngle = false, okRate = false;   // which firing checks pass (for the debug log)
unsigned long fastSince = 0;
unsigned long pauseUntil = 0;      // after a runaway: stay stopped until this time

unsigned long lastSignalTime = 0;
unsigned long stateStart = 0;
unsigned long sideSeenSince = 0;
unsigned long centeredSince = 0;
unsigned long fireStart = 0;
unsigned long lastGyroRead = 0;
bool centered = false;
bool armed = true;
bool gyroOk = true;

// ---------------- SENSORS ----------------

void readSensors(){
    strongestValue = 0;
    strongestSensor = -1;

    int raw[NUM_SENSORS];
    int sorted[NUM_SENSORS];
    for (byte i = 0; i < NUM_SENSORS; i++){
        sensors[i]->update();
        raw[i] = sensors[i]->getSmoothed();
        sorted[i] = raw[i];
    }

    // room light reaches every sensor, the beacon reaches only one or two. The middle
    // reading of the eight is therefore the room light, so take it off every sensor
    for (byte i = 1; i < NUM_SENSORS; i++){
        int v = sorted[i];
        byte j = i;
        while (j > 0 && sorted[j - 1] > v){
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = v;
    }
    ambient = (sorted[3] + sorted[4]) / 2;

    for (byte i = 0; i < NUM_SENSORS; i++){
        sensorValue[i] = raw[i] - ambient;
        if (sensorValue[i] < 0) sensorValue[i] = 0;

        if (sensorValue[i] > strongestValue){
            strongestValue = sensorValue[i];
            strongestSensor = i;
        }
    }
}

bool hasSignal(){
    return strongestValue > tol;
}

bool isFront(int s){
    return s == FRONT_LEFT || s == FRONT_RIGHT;
}

// bearing of the beacon from the laser, positive = right. Uses the strongest
// sensor and leans towards its brighter neighbour if that one also sees it
float beaconAngle(){
    int s = strongestSensor;
    if (s < 0) return 0;
    int up = (s + 1) % NUM_SENSORS;
    int down = (s + NUM_SENSORS - 1) % NUM_SENSORS;
    int n = sensorValue[up] >= sensorValue[down] ? up : down;
    float a = sensorAngle[s];

    if (sensorValue[n] > tol){
        float step = (n == up) ? 45.0 : -45.0;
        a += step * sensorValue[n] / (float)(sensorValue[s] + sensorValue[n]);
    }

    if (a > 180.0) a -= 360.0;
    if (a < -180.0) a += 360.0;
    return a;
}

// centre heading from the balance of s0 and s7 (both must see the beacon)
float balancedTarget(){
    long l = sensorValue[FRONT_LEFT];
    long r = sensorValue[FRONT_RIGHT];
    float error = (float)(l - r) / (float)(l + r);    // +1 = all on the left, -1 = all on the right
    return gyro.heading() - error * sensorAngle[FRONT_RIGHT];
}

bool bothFrontSee(){
    return sensorValue[FRONT_LEFT] > tol && sensorValue[FRONT_RIGHT] > tol;
}

// a side or back sensor clearly stronger than both front sensors, for a while
bool sideClearlyLeads(){
    int front = max(sensorValue[FRONT_LEFT], sensorValue[FRONT_RIGHT]);
    bool leads = !isFront(strongestSensor) && strongestValue > tol + tolMargin && strongestValue > 2 * front;

    if (!leads){
        sideSeenSince = millis();
        return false;
    }
    return millis() - sideSeenSince >= sideSeenTime;
}

// ---------------- PID MOTOR CONTROL ----------------

void enter(State s);

void ctlReset(){
    ctlCommand = 0;
    ctlMicros = micros();
    smoothRate = gyro.rate();
    wrongWaySince = millis();
    unwinding = false;
    stallSince = millis();
}

void stopMotor(){
    motor.stop();
    lastPwm = 0;
    ctlReset();
}

// something is wrong so stop, LED on, wait, then try again
void runawayStop(){
    stopMotor();
    digitalWrite(laser, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    pauseUntil = millis() + runawayPause;
    enter(SEARCH);
}

// slow the flywheel down gently instead of braking it. Braking hands all of the
// flywheel's spin to the body and throws the satellite round
void rampDown(){
    smoothRate += rateFilter * (gyro.rate() - smoothRate);
    unsigned long now = micros();
    float seconds = (now - ctlMicros) * 0.000001;
    ctlMicros = now;
    if (seconds > 0.05) seconds = 0.05;

    float pwm = lastPwm - idleRampRate * seconds;
    if (pwm < deadbandPwm){
        motor.stop();
        lastPwm = 0;
    }
    else{
        motor.run((int)pwm, lastDirection);
        lastPwm = (int)pwm;
    }
    ctlCommand = 0;
    unwinding = false;
    stallSince = millis();
    wrongWaySince = millis();
}

// make the body turn at wantedRate (deg/s, positive = right)
void driveRate(float wantedRate, int maxPwm){
    unsigned long now = micros();
    float seconds = (now - ctlMicros) * 0.000001;
    ctlMicros = now;
    if (seconds > 0.05) seconds = 0.05;     // ignore long gaps, such as just after a reset

    smoothRate += rateFilter * (gyro.rate() - smoothRate);

    float rateError = wantedRate - smoothRate;
    float limit = maxPwm - deadbandPwm;
    float out;

    if (unwinding){
        // a reaction wheel can only push while its speed is changing. At full speed it has
        // nothing left, so bring it back down slowly (gently enough not to throw the body back)
        // if the body starts drifting backwards, slow the unwinding right down so friction can hold it
        bool driftingBack = smoothRate * ctlCommand < 0 && fabs(smoothRate) > 2.0;
        float step = unwindRate * seconds * (driftingBack ? 0.2 : 1.0);
        if (ctlCommand > step) ctlCommand -= step;
        else if (ctlCommand < -step) ctlCommand += step;
        else ctlCommand = 0;

        // done once it's nearly stopped, or the body is already moving the right way again
        if (fabs(ctlCommand) < 5.0 || (wantedRate * smoothRate > 0 && fabs(smoothRate) >= 0.5 * fabs(wantedRate))){
            unwinding = false;
            stallSince = millis();
        }
        out = ctlCommand;
    }
    else{
        // integral: keep changing the flywheel speed until the body turns at the wanted rate
        ctlCommand += rateKi * rateError * seconds;
        ctlCommand = constrain(ctlCommand, -limit, limit);

        // proportional: react straight away to a rate error
        out = constrain(ctlCommand + rateKp * rateError, -limit, limit);

        // stalled: flywheel at full speed pushing the right way, but the body isn't turning
        bool stalled = fabs(ctlCommand) >= limit - 1 && wantedRate * ctlCommand > 0
                       && fabs(wantedRate) > 3.0 && fabs(smoothRate) < 0.3 * fabs(wantedRate);
        if (!stalled) stallSince = millis();
        else if (millis() - stallSince >= stallTime && state != FIRE) unwinding = true;   // never move the aim while firing
    }

    // small outputs ramp up smoothly from 0. Jumping straight to the deadband PWM
    // would kick the flywheel from one direction to the other every time it crosses zero
    float size = fabs(out);
    int pwm = size < rampBand ? (int)(size * (deadbandPwm + rampBand) / rampBand) : deadbandPwm + (int)size;
    bool direction = out >= 0 ? turnRight : turnLeft;

    lastPwm = pwm;
    lastDirection = direction;
    motor.run(pwm, direction);

    // safety: turning the wrong way for a long time means something is wrong
    bool wrongWay = fabs(wantedRate) > 5.0 && wantedRate * smoothRate < 0 && fabs(smoothRate) > 20.0;
    if (!wrongWay) wrongWaySince = millis();
    else if (millis() - wrongWaySince >= runawayTime) runawayStop();
}

// turn to a heading, slowing down as it gets close
void driveTo(float heading, float maxRate, int maxPwm){
    float degreesOff = heading - gyro.heading();
    float wantedRate = constrain(angleKp * degreesOff, -maxRate, maxRate);
    driveRate(wantedRate, maxPwm);
}

// ---------------- GYRO ----------------

void updateGyro(){
    bool ok = gyroOk ? gyro.update() : (gyro.begin() && gyro.update());

    if (ok){
        lastGyroRead = millis();
        if (!gyroOk){
            gyroOk = true;
            digitalWrite(LED_BUILTIN, LOW);
        }
        return;
    }

    if (gyroOk && millis() - lastGyroRead >= gyroTimeout){
        gyroOk = false;
        digitalWrite(LED_BUILTIN, HIGH);

        if (state != SEARCH){
            stopMotor();
            digitalWrite(laser, LOW);
            state = SEARCH;
        }
    }
}

// ---------------- STATES ----------------

void enter(State s){
    state = s;
    stateStart = millis();
    sideSeenSince = millis();
    centered = false;
}

void startSweep(int frontSensor){
    sweepSensor = frontSensor;
    sweepDir = (frontSensor == FRONT_RIGHT) ? 1 : -1;   // beacon on the right, so turn right
    lastSeenHeading = gyro.heading();
    enter(SWEEP);
}

// decide what to do with a fresh sighting
void startTracking(){
    lastSignalTime = millis();

    if (bothFrontSee()){
        target = balancedTarget();
        enter(HOLD);
    }
    else if (isFront(strongestSensor) && fabs(beaconAngle()) <= 30.0){
        startSweep(strongestSensor);
    }
    else{
        target = gyro.heading() + beaconAngle();
        enter(TURN);
    }
}

void searchState(){
    if ((long)(pauseUntil - millis()) > 0){
        stopMotor();
        digitalWrite(laser, LOW);
        return;
    }
    if (gyroOk) digitalWrite(LED_BUILTIN, LOW);

    rampDown();
    digitalWrite(laser, LOW);

    // after firing, wait until the beacon leaves the front before tracking again
    if (!armed){
        if (!hasSignal() || !isFront(strongestSensor)) armed = true;
        else return;
    }

    if (gyroOk && strongestValue > tol + tolMargin && lastPwm == 0){
        ctlReset();
        startTracking();
    }
}

// coarse turn towards the beacon
void turnState(){
    if (hasSignal()){
        lastSignalTime = millis();

        if (bothFrontSee() || (isFront(strongestSensor) && fabs(beaconAngle()) <= 30.0)){
            startTracking();
            return;
        }

        // refresh the estimate every loop, so it corrects itself as it turns.
        // When the beacon is behind, keep turning the way it already is
        float a = beaconAngle();
        if (fabs(a) > 135.0 && ctlCommand != 0 && (a > 0) != (ctlCommand > 0)){
            a += (a > 0) ? -360.0 : 360.0;
        }
        target = gyro.heading() + a;
    }
    else if (millis() - lastSignalTime >= signalLostTimeout){
        enter(SEARCH);
        return;
    }

    driveTo(target, turnMaxRate, maxSpeed);
}

// slow turn through the gap between s0 and s7, to find where the centre is
void sweepState(){
    int other = (sweepSensor == FRONT_RIGHT) ? FRONT_LEFT : FRONT_RIGHT;
    float heading = gyro.heading();

    if (hasSignal()) lastSignalTime = millis();

    // both see it: the balance gives the centre directly
    if (bothFrontSee()){
        target = balancedTarget();
        enter(HOLD);
        return;
    }

    // still on the first front sensor: remember where
    if (sensorValue[sweepSensor] > tol) lastSeenHeading = heading;

    // reached the other front sensor: the centre is midway between the two sightings
    if (sensorValue[other] > tol){
        target = (lastSeenHeading + heading) / 2.0;
        enter(HOLD);
        return;
    }

    // a side or back sensor clearly has it, so this wasn't the front
    if (sideClearlyLeads()){
        target = heading + beaconAngle();
        enter(TURN);
        return;
    }

    // swept the whole gap without the other sensor seeing it
    if ((heading - lastSeenHeading) * sweepDir > maxFrontGap){
        target = lastSeenHeading + sweepDir * frontGapGuess / 2.0;
        enter(HOLD);
        return;
    }

    driveRate(sweepDir * sweepRate, maxSpeed);
}

// hold on the centre with the motor still running, then fire once settled
bool holdOnTarget(){
    // when both front sensors see it, gently correct the centre from their balance
    if (bothFrontSee()){
        target += 0.05 * (balancedTarget() - target);
    }

    driveTo(target, holdMaxRate, holdMaxPwm);

    // never fire while a side or back sensor clearly has the beacon. A side sensor that is only a
    // little higher (room light, reflections, the beacon sitting in the gap between s0 and s7) is fine
    int front = max(sensorValue[FRONT_LEFT], sensorValue[FRONT_RIGHT]);
    int side = 0;
    for (byte i = 0; i < NUM_SENSORS; i++){
        if (!isFront(i) && sensorValue[i] > side) side = sensorValue[i];
    }
    okFront = !(side > tol + tolMargin && side > 2 * front);

    // within the tolerance, and a looser one if it has been trying for a while
    float allowed = (state == HOLD && millis() - stateStart >= lateHoldTime) ? lateHoldTol : holdTol;
    okAngle = fabs(target - gyro.heading()) <= allowed;

    okRate = fabs(settleRate) <= holdRateTol;

    return okFront && okAngle && okRate;
}

void holdState(){
    digitalWrite(laser, LOW);

    if (hasSignal()) lastSignalTime = millis();
    else if (millis() - lastSignalTime >= signalLostTimeout){
        enter(SEARCH);
        return;
    }

    if (sideClearlyLeads()){
        target = gyro.heading() + beaconAngle();
        enter(TURN);
        return;
    }

    bool settled = holdOnTarget();
    if (state != HOLD) return;

    if (!settled){
        centered = false;
    }
    else if (!centered){
        centered = true;
        centeredSince = millis();
    }
    else if (millis() - centeredSince >= settleTime){
        fireStart = millis();
        enter(FIRE);
        return;
    }

    if (millis() - stateStart >= holdTimeout){
        if (hasSignal()){
            startTracking();
        }
        else{
            enter(SEARCH);
        }
    }
}

void fireState(){
    holdOnTarget();
    if (state != FIRE) return;

    if (fabs(target - gyro.heading()) > fireTol){
        digitalWrite(laser, LOW);
        enter(HOLD);
        return;
    }
    digitalWrite(laser, HIGH);

    if (millis() - fireStart >= fireDuration){
        digitalWrite(laser, LOW);
        armed = false;
        enter(SEARCH);
    }
}

// ---------------- DEBUG ----------------

// prints: state, strongest sensor, its reading, PWM, heading, target, rate, U if unwinding,
// firing checks (F = no side sensor has it, A = aimed within tolerance, R = slow enough; all three needed), s0, s7, room light
void debugLog(){
    static unsigned long last = 0;
    if (millis() - last < 200) return;
    last = millis();

    const char *names[] = {"SEARCH", "TURN", "SWEEP", "HOLD", "FIRE"};
    Serial.print(names[state]);         Serial.print(' ');
    Serial.print(strongestSensor);      Serial.print(' ');
    Serial.print(strongestValue);       Serial.print(' ');
    Serial.print(lastPwm);              Serial.print(' ');
    Serial.print(gyro.heading(), 1);    Serial.print(' ');
    Serial.print(target, 1);            Serial.print(' ');
    Serial.print(gyro.rate(), 1);       Serial.print(' ');
    Serial.print(unwinding ? 'U' : '-'); Serial.print(' ');
    Serial.print(okFront ? 'F' : '-');  Serial.print(okAngle ? 'A' : '-'); Serial.print(okRate ? 'R' : '-'); Serial.print(' ');
    Serial.print(sensorValue[FRONT_RIGHT]); Serial.print(' ');
    Serial.print(sensorValue[FRONT_LEFT]);  Serial.print(' ');
    Serial.println(ambient);
}

// ---------------- SETUP AND LOOP ----------------

void setup(){
    if (DEBUG_LOG) Serial.begin(115200);

    motor.begin();
    mux.begin();

    pinMode(laser, OUTPUT);
    digitalWrite(laser, LOW);
    pinMode(LED_BUILTIN, OUTPUT);

    Wire.begin();
    Wire.setWireTimeout(3000, true);    // a noisy bus can't hang the code with the motor running

    // the motor stays stopped until the gyro responds. The LED blinks until then
    while (!gyro.begin() || (CALIBRATE_GYRO && !gyro.calibrate())){
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(250);
    }
    digitalWrite(LED_BUILTIN, LOW);
    lastGyroRead = millis();

    // the first readings after changing the reference are wrong, so throw them away
    if (USE_INTERNAL_REF){
        analogReference(INTERNAL);
        for (byte i = 0; i < 10; i++){
            analogRead(A0);
            delay(5);
        }
    }

    for (byte i = 0; i < NUM_SENSORS; i++){
        sensors[i]->begin();
    }

    lastPwm = 0;
    pauseUntil = millis();
    guardRate = 0;
    fastSince = millis();
    ctlReset();
}

void loop(){
    readSensors();
    updateGyro();

    // runaway check that nothing else can override: spinning far faster than the code ever asks for
    guardRate += 0.1 * (gyro.rate() - guardRate);
    settleRate += 0.03 * (gyro.rate() - settleRate);
    if (state == SEARCH || fabs(guardRate) < runawayRate) fastSince = millis();
    else if (millis() - fastSince >= runawayFastTime) runawayStop();

    switch (state){
    case SEARCH: searchState(); break;
    case TURN:   turnState();   break;
    case SWEEP:  sweepState();  break;
    case HOLD:   holdState();   break;
    case FIRE:   fireState();   break;
    }

    if (DEBUG_LOG) debugLog();
}
