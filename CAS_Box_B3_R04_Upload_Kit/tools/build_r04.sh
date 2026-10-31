#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLI="${ARDUINO_CLI:-${HOME}/.local/bin/arduino-cli}"
CONFIG="${ARDUINO_CLI_CONFIG:-${HOME}/.arduino-cli/arduino-cli.yaml}"
OUT="${ROOT}/build"

command -v "$CLI" >/dev/null || { echo "arduino-cli not found: $CLI" >&2; exit 1; }
mkdir -p "$OUT/box" "$OUT/camera"

"$CLI" compile --config-file "$CONFIG" --jobs 4 \
  --fqbn esp32:esp32:esp32 \
  --build-path "$OUT/box" \
  "$ROOT/cas_box"

"$CLI" compile --config-file "$CONFIG" --jobs 4 \
  --fqbn esp32:esp32:esp32cam \
  --build-path "$OUT/camera" \
  "$ROOT/camera_s3/CameraStation"

echo
echo "Built successfully. Flash these sketches:"
echo "  Main box:  $OUT/box/cas_box.ino.bin"
echo "  ESP32-CAM: $OUT/camera/CameraStation.ino.bin"
