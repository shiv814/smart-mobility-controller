# Contributing

Safety-related behavior should be small, deterministic and testable.

- do not weaken Stop/fault behavior to make a demo smoother
- add tests for every new interlock, fault transition or command source
- keep telemetry/diagnostics separate from motor-state mutation
- build and test both Debug and Release
- document new assumptions in the hazard/validation notes
- preserve the explicit prototype/non-certified disclaimer
