# Smart Mobility Controller

This repository contains the control software for a small Arduino-based mobility prototype. The motion logic is separated into portable C++ so safety behaviour can be tested on a computer before deployment to the microcontroller.

## Features

- Forward, reverse, pivot-left, pivot-right, and stop commands
- Gradual PWM ramping to reduce abrupt motor starts
- Dead-man timeout that automatically returns both motors to zero
- Host-side unit tests for command transitions and timeout behaviour
- Arduino integration sketch for a dual H-bridge motor driver

## Host test

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Arduino serial controls

Send `F`, `B`, `L`, `R`, or `S` at 9600 baud. Pin definitions are at the top of the sketch and should be adjusted to match the final wiring.

> Hardware must be tested at low power with the wheels lifted before carrying any load.
