#!/usr/bin/env bash
# Compile Kotoba Protocol 2.0 sources to wasm and run fixture + example mains.
# Each entry is compiled separately: kotoba-wasm v0.7.2 traps if two CRC folds
# are live in one module.
# Requires: kotoba CLI (v0.7.2 contract), node 18+.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
KOTOBA_DIR="$ROOT/kotoba"
LIB=(
  "$KOTOBA_DIR/src/dynamixel_sdk/robotis_def.kotoba"
  "$KOTOBA_DIR/src/dynamixel_sdk/protocol2.kotoba"
)
OUT="${KOTOBA_DIR}/.build"
mkdir -p "$OUT"

KOTOBA_BIN="${KOTOBA_BIN:-kotoba}"
if ! command -v "$KOTOBA_BIN" >/dev/null 2>&1; then
  echo "kotoba CLI not found; set KOTOBA_BIN or install v0.7.2" >&2
  exit 1
fi

compile_entry() {
  local entry="$1"
  local name="$2"
  local src="$OUT/${name}.kotoba"
  local wasm="$OUT/${name}.wasm"
  cat "${LIB[@]}" "$entry" > "$src"
  echo "compile $name" >&2
  local json
  json="$("$KOTOBA_BIN" compile "$src" --target wasm --output "$wasm" --json)"
  python3 -c '
import json, sys
payload = json.loads(sys.argv[1])
ok = payload.get("kotoba.cli/ok?") is True
code = payload.get("kotoba.cli/code")
if not ok or code != "emitted":
    raise SystemExit("compile failed: " + json.dumps(payload))
path = sys.argv[2]
with open(path, "rb") as fh:
    magic = fh.read(4)
if magic != b"\x00asm":
    raise SystemExit("output is not wasm: " + path)
print("emitted " + path, file=sys.stderr)
' "$json" "$wasm"
  printf '%s\n' "$wasm"
}

run_wasm() {
  local wasm="$1"
  node --input-type=module - "$wasm" <<'JS'
import fs from "node:fs";
const wasmPath = process.argv[2];
const bytes = fs.readFileSync(wasmPath);
const module = new WebAssembly.Module(bytes);
const imports = WebAssembly.Module.imports(module);
if (imports.length) {
  throw new Error("unexpected host imports: " + JSON.stringify(imports));
}
const instance = new WebAssembly.Instance(module, {});
if (typeof instance.exports.main !== "function") {
  throw new Error("missing export main; have " + Object.keys(instance.exports));
}
const value = instance.exports.main();
const asNumber = typeof value === "bigint" ? Number(value) : value;
console.log("main => " + asNumber);
if (asNumber !== 0) {
  process.exit(1);
}
JS
}

run_one() {
  local entry="$1"
  local name="$2"
  local wasm
  wasm="$(compile_entry "$entry" "$name")"
  echo
  echo "run $name"
  run_wasm "$wasm"
}

run_one "$KOTOBA_DIR/example/ping_write.kotoba" ping_write

for case in constants ping ping_id2 action reboot read read_body \
            write1 write1_body write2 write4 \
            reg_write sync_read sync_write stuff nostuff status errors; do
  run_one "$KOTOBA_DIR/tests/cases/${case}.kotoba" "$case"
done

echo "all kotoba Protocol 2.0 fixture tests passed"
