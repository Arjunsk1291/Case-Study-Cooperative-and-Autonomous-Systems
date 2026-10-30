#pragma once
/*
 * Copy to camera_board_config.h only after identifying the exact ESP32-S3 camera board.
 * No GPIOs are guessed here. Obtain the official pin map for the actual module.
 *
 * Required pin mapping: PWDN, RESET, XCLK, SIOD, SIOC, D0-D7, VSYNC, HREF, PCLK.
 * Fill every field in configureCasCamera() using the board manufacturer's schematic.
 */
#define CAS_CAMERA_BOARD_PINMAP_CONFIGURED 0
#define CAS_CAMERA_BOARD_ID "UNCONFIGURED_CAMERA_BOARD"
#define CAS_CAMERA_LAPTOP_HOST "192.168.17.X"
#define CAS_CAMERA_TOKEN "CHANGE_ME"

#include "esp_camera.h"
inline bool configureCasCamera(camera_config_t&) { return false; }
