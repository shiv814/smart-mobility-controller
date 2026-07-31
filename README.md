# Smart Mobility Controller

Smart Mobility Controller is a safety-oriented C++17 control core for a powered mobility prototype. Version 2 grows the original five-command PWM ramp into a layered controller with joystick mixing, configurable drive modes, obstacle-aware speed limiting, battery derating, seat and emergency interlocks, command watchdogs, latched hardware faults, direction-reversal braking, telemetry, event history, a scenario simulator, and an Arduino integration sketch.

> This repository is an engineering portfolio prototype, not certified medical-device software. Real mobility hardware requires formal hazard analysis, redundant safety systems, hardware interlocks, electrical review, verification, validation, and regulatory compliance.

## Control features

### Differential drive inputs

The controller accepts both discrete commands and normalized joystick coordinates:

- stop, forward, reverse, pivot left, pivot right
- two-axis joystick mixing with configurable deadzone
- output normalization so mixed commands stay inside the PWM envelope

### Drive modes

- **Eco**: 55% nominal speed for controlled indoor movement
- **Normal**: 80% nominal speed
- **Sport**: 100% configured speed

Drive-mode scaling is combined with safety scaling rather than bypassing it.

### Motion shaping

- separate acceleration and deceleration steps
- controlled PWM ramping on every update
- reversal guard that returns each motor to zero before changing direction
- explicit braking state when a safety stop reaches zero output

## Safety envelope

### Command watchdog

A stale command automatically produces a stopped state and `command-timeout` fault report. A fresh command clears this transient condition.

### Obstacle response

Four directional distance measurements support context-aware protection:

- full speed beyond the slowdown distance
- proportional derating inside the slowdown zone
- complete stop inside the stop distance
- forward/reverse/turn/joystick commands choose the relevant sensors

### Battery management

- estimated battery percentage from critical and full-voltage calibration
- low-battery speed derating
- latched critical-battery fault requiring safe voltage and an explicit clear

### Interlocks and faults

- emergency stop is latched
- invalid or negative sensor data is latched as a sensor fault
- an unoccupied seat causes a transient stop
- latched faults cannot be cleared while unsafe conditions remain
- fault, state, and command transitions are retained in a bounded event log

## Telemetry

Each update records:

- sequence and timestamp
- command and drive mode
- ready, degraded, stopped, or fault safety state
- current and latched fault status
- left/right PWM and brake state
- requested and safety speed scales
- battery percentage
- nearest relevant obstacle
- watchdog status

## Build and test

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

The host tests verify acceleration, braking-before-reversal, obstacle slowdown and stopping, timeout recovery, emergency-stop latching, safe fault clearing, battery derating, critical battery handling, joystick mixing, deadzone behaviour, seat interlock, invalid-sensor latching, event-log retention, and configuration validation.

## Scenario simulator

Run the built-in demonstration:

```bash
./build/mobility_simulator --demo
```

Run a CSV scenario:

```bash
./build/mobility_simulator examples/safety_scenario.csv
```

Scenario columns are:

```text
timestamp_ms,command,front_cm,rear_cm,left_cm,right_cm,battery_voltage,e_stop,seat_occupied
```

The simulator emits CSV telemetry suitable for plotting, regression comparison, or spreadsheet analysis.

## Library example

```cpp
#include "controller.hpp"

mobility::Controller controller;
controller.set_drive_mode(mobility::DriveMode::Normal);
controller.joystick(0.25, 0.8, 0);

mobility::SensorFrame sensors;
sensors.front_distance_cm = 55.0;
sensors.battery_voltage = 23.4;

auto motors = controller.update(20, sensors);
auto telemetry = controller.telemetry();
```

## Embedded integration

`arduino/TeddyBearWheelchair.ino` shows how the architecture maps to joystick ADC inputs, an emergency-stop input, seat switch, motor PWM/direction pins, watchdog logic, and JSON serial telemetry. The portable host controller remains the tested reference model; production embedded integration should use a suitable build system and hardware abstraction layer so the same safety logic can be compiled directly for the target.

## Project structure

```text
src/
├── controller.hpp  # public domain types, configuration, telemetry, API
├── controller.cpp  # state machine, safety envelope, motion shaping
└── simulator.cpp   # deterministic CSV scenario runner
arduino/
└── TeddyBearWheelchair.ino
examples/
└── safety_scenario.csv
tests/
└── test_controller.cpp
```

## Suggested next engineering steps

- hardware-in-the-loop tests with mocked range and current sensors
- wheel encoder feedback and closed-loop velocity control
- redundant stop channels and independent watchdog hardware
- fault-injection tests and requirements traceability
- structured hazard analysis such as FMEA or STPA
- recorded telemetry replay and coverage-guided scenario generation
