# MQ Sentinel

**MQ Sentinel** is an ENGG2000 engineering project developed at Macquarie University. The project explores a proof-of-concept autonomous satellite system that can detect, track, and engage simulated space debris using infrared sensing, internal rotational control, and an eye-safe laser.

The system is designed to detect a **38 kHz infrared beacon**, determine the direction of the active target, rotate the satellite using an internally driven reaction-wheel/flywheel mechanism, align the laser with the target, and hold the laser on-target for the required dwell period.

## Project Objective

The primary objective is to demonstrate the following autonomous control loop:

```text
IR beacon
   ↓
IR sensing
   ↓
Target detection / direction estimation
   ↓
Arduino control logic
   ↓
Motor driver
   ↓
Reaction wheel / satellite rotation
   ↓
Target alignment
   ↓
Laser activation
```

For the project demonstration, the complete system is intended to:

- Detect an active **38 kHz IR beacon**
- Determine the target direction from sensor readings
- Rotate using internal propulsion only
- Align the laser with the corresponding target
- Maintain the laser on-target for at least **2 seconds**
- Operate autonomously without manual repositioning
- Repeat the targeting cycle during the demonstration period

## System Overview

MQ Sentinel combines three main engineering subsystems:

### 1. Infrared Sensing

IR receivers provide target information to the Arduino. The software reads sensor values and uses differences between sensor measurements to determine whether the target is left, right, or approximately centred.

Sensor test code also implements a moving-average filter to reduce noise in the raw measurements.

### 2. Motor / Attitude Control

The Arduino sends direction and PWM commands to the motor driver. The motor drives the reaction wheel/flywheel, producing an opposing rotational response in the satellite body.

The control logic:

- rotates clockwise when the target is detected on one side,
- rotates counter-clockwise when detected on the other side,
- stops when the target is within the centring tolerance.

### 3. Laser Targeting

Once the sensor readings indicate that the target is centred, the motor is stopped and the laser is activated.

The integrated prototype code holds the laser on for **2 seconds** before switching it off.

## Hardware

The project is based around the following components:

- Arduino Nano
- 38 kHz infrared receivers
- DC gearmotor
- Motor driver
- Reaction wheel / flywheel
- Eye-safe laser module
- Battery supply
- Mechanical sensor mounting structure
- PVC satellite body
- Optional / developing multi-sensor interface using an 8-channel multiplexer

## Repository Structure

| File | Purpose |
| --- | --- |
| `sensorTestCode.ino` | Tests IR sensor readings and applies moving-average smoothing |
| `testMotorCode.ino` | Basic motor-driver test |
| `LightMotorCode.ino` | Tests forward/reverse motor operation |
| `motorAndSensorCode.ino` | Integrates two IR sensors with motor direction control |
| `motorAndSensorCodeWithLaser.ino` | Integrates sensing, motor control, target centring, and laser activation |
| `motorAndSensorCodeWithLaser(2).ino` | Updated integrated sensing/motor/laser prototype |
| `README.md` | Project documentation |

## Current Arduino Logic

The integrated prototype uses two IR sensor inputs and compares their smoothed values:

```text
Read Sensor 1 + Sensor 2
          ↓
    Signal detected?
      /        \
    No          Yes
    ↓            ↓
 Stop motor   Compare sensors
                 ↓
        Is target centred?
          /            \
        Yes             No
        ↓                ↓
   Stop motor       Rotate toward
   Fire laser       stronger signal
   for 2 s
```

The current integrated code uses:

| Function | Configuration |
| --- | --- |
| Motor IN1 | Digital/PWM pin 3 |
| Motor IN2 | Pin 6 |
| Motor sleep | Pin 7 |
| IR sensor 1 | A0 |
| IR sensor 2 | A1 |
| Laser | Pin 8 |
| Sensor moving average | 10 readings |
| Centre tolerance | 30 |
| Motor PWM in integrated code | 250 |
| Laser dwell | 2000 ms |

> **Note:** Pin assignments and threshold values reflect the current prototype code and may change as the system is calibrated and expanded.

## IR Sensor Filtering

The `IRSensor` class maintains a rolling set of measurements and calculates a moving average:

```cpp
int getSmoothed() {
    return _total / _numReadings;
}
```

Using a moving average reduces short-term fluctuations in the sensor signal and provides more stable input to the target-direction algorithm.

## Target Direction Logic

For the two-sensor prototype:

```cpp
diff = sensor1 - sensor2
```

The controller then interprets the difference as:

- `|diff| <= centre tolerance` → target centred
- `diff > 0` → rotate in one direction
- `diff < 0` → rotate in the opposite direction

This creates a simple closed-loop targeting behaviour in which sensor measurements directly influence the satellite's rotational movement.

## Getting Started

### Requirements

- Arduino IDE
- Arduino Nano-compatible board
- Appropriate USB cable
- MQ Sentinel prototype hardware

### Clone the Repository

```bash
git clone https://github.com/GTashrif/MQSentinel.git
cd MQSentinel
```

### Upload a Sketch

1. Open the required `.ino` file in the Arduino IDE.
2. Select the correct **Arduino Nano** board.
3. Select the correct processor and serial port.
4. Verify the wiring matches the pin assignments in the selected sketch.
5. Compile and upload the sketch.
6. For sensor testing, open the Serial Monitor where applicable.

## Suggested Testing Sequence

To isolate faults before full integration, test the project progressively:

1. **Sensor test** — confirm that the IR receiver responds to the target and produces stable readings.
2. **Motor test** — confirm forward, reverse, stop, and PWM behaviour.
3. **Sensor + motor test** — verify that sensor imbalance produces the correct rotation direction.
4. **Centring test** — tune the sensor tolerance so that the motor stops near target alignment.
5. **Laser integration test** — verify that the laser only activates after alignment.
6. **Full-system test** — validate the complete detect → rotate → align → fire sequence.

## Design Development

The sensing architecture has been explored using multiple possible configurations, including:

- four directly connected IR sensors,
- eight directly connected IR sensors,
- eight IR sensors accessed through an 8-channel multiplexer.

The multi-sensor approach is intended to improve directional resolution while balancing Arduino I/O limitations, software complexity, and integration requirements.

## Project Constraints

The MQ Sentinel prototype is developed under several engineering constraints, including:

- autonomous operation,
- internal propulsion/rotation,
- Arduino Nano as the embedded controller,
- limited package dimensions,
- reusable supplied components,
- modular construction,
- safe laser operation,
- limited component budget.

These constraints require the sensing, software, electrical, and mechanical subsystems to be developed as an integrated system rather than independently optimised components.

## Future Development

Possible next steps include:

- expanding from the current two-sensor prototype to a larger sensor array,
- integrating the 8-channel multiplexer,
- calibrating detection thresholds experimentally,
- improving target-direction estimation,
- reducing overshoot around the target,
- implementing more advanced feedback control,
- improving signal-loss handling,
- adding sensor and motor diagnostic output,
- measuring target-acquisition time and tracking accuracy,
- refining the system for repeated autonomous targeting.

## Academic Context

This repository documents prototype development for the **MQ Sentinel Space Debris Cleanup Challenge** as part of **ENGG2000 at Macquarie University**.

It contains iterative experimental code rather than production flight software.

## Repository

**GitHub:** https://github.com/GTashrif/MQSentinel

## License

No open-source licence is currently specified for this repository. Unless a licence is added, the code remains subject to the copyright rights of its respective authors.
