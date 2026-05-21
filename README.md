# ESP32 PSU Warmer Display

Two-board embedded controller for a battery warmer with ZK-6522L PSU integration.

## Hardware
- **CrowPanel** (Elecrow DIS07050H, ESP32-S3): touchscreen UI, 4-page LVGL interface
- **Feather** (Adafruit ESP32-S3): PID heating, MAX31865 PT1000 ×2, SD logging, WiFi AP

## Projects
- `crowpanel/` — LovyanGFX 1.2.19 + LVGL 8.4.0 display firmware
- `feather/` — Headless controller firmware

## Build
Open each subfolder as a PlatformIO project in VS Code.