# Validation Strategy

The host-side controller and v3 modules are deterministic so they can be exercised without physical hardware.

## Automated coverage

- direction and PWM ramp behavior
- timeout and fault latching
- obstacle, seat and battery safety behavior
- command-priority/expiry arbitration
- emergency-stop arbitration override
- battery reserve/range estimator boundaries
- straight-line differential odometry
- diagnostic safety KPI accumulation
- deterministic CSV scenario simulation

CI builds Debug and Release configurations on Linux, Windows and macOS and executes CTest targets in every configuration.

## Hardware validation still required

A real powered platform would require motor-current testing, braking distance tests, sensor fault injection, EMI/EMC review, battery/BMS validation, watchdog independence, emergency-stop verification, mechanical load testing, human-factors review and formal traceability from hazards to requirements/tests.
