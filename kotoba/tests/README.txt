Fixture tests
=============

Goldens are in goldens.txt (derived from this repo's Python Protocol 2.0
handler: stuffing, header, CRC-16). No UART and no motor.

Run:

  kotoba/tests/run_fixtures.sh

Each tests/cases/*.kotoba file is concatenated with the library and compiled
to its own wasm module. kotoba-wasm v0.7.2 traps if two CRC folds are live in
one module, so the runner keeps one packet CRC per artifact.
