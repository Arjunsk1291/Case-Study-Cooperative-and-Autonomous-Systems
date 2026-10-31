# R04 verified build and flashing guide

This guide is for the `phase1-indoor-reliability` branch. The rover firmware is not flashed or changed.

## Build

From this directory:

```bash
./tools/build_r04.sh
```

The verified board targets are:

- Main CAS Box: `esp32:esp32:esp32` — ESP32 Dev Module.
- Camera: `esp32:esp32:esp32cam` — AI Thinker ESP32-CAM.

The current build was compiled with Arduino CLI 1.5.1 and ESP32 core 3.3.12.

## Flash the main CAS Box

1. Disconnect rover motor power. Keep the rover firmware untouched.
2. Connect the main ESP32 USB/UART adapter.
3. Identify the port with `arduino-cli board list`.
4. Hold BOOT if the adapter requires it, then run:

```bash
arduino-cli upload --config-file "$HOME/.arduino-cli/arduino-cli.yaml" \
  -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 \
  --input-dir ./build/box
```

Replace `/dev/ttyUSB0` with the actual port. Open a serial monitor at 115200 baud. The box must create `CAS-Box-04` at `192.168.14.1`.

## Flash the AI-Thinker ESP32-CAM

1. Use a 5 V supply or USB-UART adapter capable of stable camera current.
2. Connect GPIO0 to GND only while entering the bootloader.
3. Identify the camera port with `arduino-cli board list`.
4. Upload:

```bash
arduino-cli upload --config-file "$HOME/.arduino-cli/arduino-cli.yaml" \
  -p /dev/ttyUSB1 --fqbn esp32:esp32:esp32cam \
  --input-dir ./build/camera
```

Replace `/dev/ttyUSB1` with the actual camera port. Remove GPIO0 from GND and reset the camera before testing. Open its serial monitor at 115200 baud.

## Start order and test

1. Power the main box and wait for the `CAS-Box-04` access point.
2. Power/reset the rover so it joins the box AP.
3. Power/reset the camera with GPIO0 disconnected from GND.
4. Join the phone to `CAS-Box-04` using `CASbox04` and open `http://192.168.14.1`.
5. Confirm the pre-flight indicators before starting a log.
6. Request one image and verify the camera status becomes ready and the phone receives a JPEG.

Do not declare movement or image capture successful from HTTP status alone. Confirm wheel movement physically and verify the saved JPEG begins with `FF D8` and ends with `FF D9`.
