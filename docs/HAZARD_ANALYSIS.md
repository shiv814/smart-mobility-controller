# Prototype Hazard Analysis

This document demonstrates safety thinking; it is **not** a formal ISO 14971, IEC 62304, ISO 7176, or regulatory risk file.

| Hazard | Example cause | Software mitigation represented here | Real-system controls still required |
|---|---|---|---|
| Uncommanded motion | stale command / input fault | watchdog timeout, command TTLs, fallback Stop | independent hardware inhibit, redundant validation |
| Collision | obstacle ahead | slowdown/stop thresholds, telemetry | redundant sensing, braking validation, field-of-view analysis |
| Runaway after emergency | software state fault | latched emergency fault / arbitration stop | hardwired emergency circuit |
| Unsafe reverse transition | immediate sign reversal | brake-before-reversal | motor-driver validation, mechanical braking |
| Low-voltage loss of control | depleted battery | derating + critical battery latch + reserve estimate | BMS, undervoltage protection, battery qualification |
| Occupant absent | seat state changes | seat interlock | validated occupancy sensing / fail-safe wiring |
| Sensor corruption | invalid readings | sensor-fault latch | plausibility redundancy, diagnostics, fault injection testing |

## Safety principle

Software should reduce risk but must not be the only layer preventing hazardous motion. The Arduino sketch and host controller are educational prototypes, not certified mobility-device software.
