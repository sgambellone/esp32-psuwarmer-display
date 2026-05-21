#pragma once
// =============================================================================
// display_driver.h — Elecrow CrowPanel 5" (ESP32-S3-WROOM-1-N4R8)
// Display : 800×480 RGB parallel, ILI6122+ILI5960
// Touch   : GT911 capacitive (I2C, IO19=SDA, IO20=SCL)
// Backlight: GPIO2 via LovyanGFX Light_PWM
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

#define SCREEN_WIDTH   800
#define SCREEN_HEIGHT  480

void display_driver_init();
void display_driver_tick();
void display_set_brightness(uint8_t level);  // 0=off, 255=full