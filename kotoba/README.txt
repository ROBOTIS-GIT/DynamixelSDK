Kotoba language binding for Dynamixel SDK
=========================================

This tree is an in-language Dynamixel Protocol 2.0 packet codec. It sits next
to python/ as another supported language, not as a wrap of the C dynamic
library.

Why it is not a C dylib wrap
----------------------------
Kotoba guest programs have no ambient FFI (no-interop). They cannot load
libdxl_x64_c.so / .dylib / .dll. The Python module already implements the
protocol in-language; this binding follows that model.

What is in v1
-------------
- Protocol 2.0 instruction encode: ping, read, write (1/2/4 byte and a 3-byte
  stuffing fixture), reg_write, action, sync_read (two IDs), sync_write
  (one ID, 4 data bytes), reboot
- Status packet decode: header, INST_STATUS (0x55), CRC, error bits
  (ERRNUM_* and ERRBIT_ALERT)
- Constants from robotis_def (BROADCAST_ID, COMM_*, INST_*)
- CRC-16 (poly 0x8005), matching the Python/C Protocol 2.0 table method
- Byte stuffing for the 0xFF 0xFF 0xFD pattern (vendored write3 fixture)
- example/ping_write.kotoba builds a ping and a write packet (no device)
- tests/cases/*.kotoba compare those packets to goldens in tests/goldens.txt
  (one wasm per case: kotoba-wasm v0.7.2 traps if two CRC folds are live
  in the same module)

What is a host concern
----------------------
port_handler (serial / USB / U2D2) is not implemented in guest code. The guest
emits instruction bytes and accepts status bytes. A host process that has a
real UART can write the encoded bytes and feed received bytes back into the
decode helpers. Fixture tests use a mock byte pipe only.

Follow-up (not in this tree)
----------------------------
- Protocol 1.0
- easy_sdk / control-table helpers
- group_bulk_* and fast sync/bulk variants
- A host serial provider bound through a Kotoba capability

Kotoba surface used
-------------------
Admitted against kotoba-lang/kotoba CLI v0.7.2 and the kotoba-lang language
contract (language releases are still 0; this binding does not claim a
language v0.6.0). The codec is i64-only so `kotoba compile --target wasm`
emits a module with no host imports.

Portable wasm currently lacks a guest bytes type, integer-key maps, bit-shift
opcodes, rem/mod, and cross-file prelude bindings. Packet bytes are therefore
indexed i64 functions, shifts are `* 2` / `quot`, and the test runner
concatenates library files in front of an entry before compile.

Build and test
--------------
Install the Kotoba CLI (v0.7.2):

  https://github.com/kotoba-lang/kotoba/releases

Then:

  kotoba/tests/run_fixtures.sh

That script concatenates src/dynamixel_sdk/*.kotoba with the example or test
entry, compiles to wasm, rejects unexpected imports, and requires each
exported fixture to return 0.

Manual compile of the example:

  cat kotoba/src/dynamixel_sdk/robotis_def.kotoba \
      kotoba/src/dynamixel_sdk/protocol2.kotoba \
      kotoba/example/ping_write.kotoba \
      > /tmp/ping_write.kotoba
  kotoba compile /tmp/ping_write.kotoba --target wasm --output /tmp/ping_write.wasm --json

Protocol documentation
----------------------
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
