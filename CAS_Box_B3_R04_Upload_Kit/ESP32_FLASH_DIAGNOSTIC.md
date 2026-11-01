# Main ESP32 flash diagnostic

The supplied serial output shows the upload verified successfully, then the ROM bootloader repeatedly reports `csum err` while loading the application. That occurs before the Arduino program can run, so it is not yet evidence of a watchdog in `loop()`.

## Capture the diagnostic bundle

Do not power the motors for this test. Install the serial dependency once:

```bash
python3 -m pip install pyserial esptool
```

Run this from the `CAS_Box_B3_R04_Upload_Kit` directory:

```bash
python3 tools/diagnose_main_esp32.py \
  --port /dev/ttyUSB0 \
  --expected-bin build/box/cas_box.ino.bin \
  --out diagnostic_main_esp32
```

If the compiled binary is not present, omit `--expected-bin`; the serial and flash identification files are still useful.

Send the complete `diagnostic_main_esp32/` directory. Do not send only a screenshot.

## Safe hardware isolation before repeating the flash

1. Disconnect rover/motor power and all external sensors from the main ESP32.
2. Leave only the USB-UART connection and a stable 5 V supply.
3. Select `ESP32 Dev Module` in Arduino IDE.
4. Set `Erase All Flash Before Sketch Upload` to `Enabled`.
5. Set upload speed to `115200`.
6. Upload the source sketch once.
7. Reset and capture the serial output.

If the error persists with all peripherals disconnected, the next test is a different USB-UART adapter or ESP32 board. If the readback hash does not match the expected binary, suspect flash hardware, supply, wiring, or the adapter. If the hash matches but ROM boot still reports checksum errors, the flash mode/flash configuration becomes the next suspect.

Do not change watchdogs or safety code to hide this failure. The firmware cannot start the Wi-Fi AP until the ROM loader can reliably read the application.
