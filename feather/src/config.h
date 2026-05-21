#pragma once

// =============================================================================
// config.h — Battery Warmer Controller (ESP32-S3 Feather, headless)
//
// Hardware:
//   Board    : Adafruit Feather ESP32-S3  (#5477, N4R2, 2MB PSRAM)
//   Display  : Elecrow CrowPanel 5" — connected via UART2 (Serial2)
//   Sensors  : Adafruit MAX31865 PT1000 RTD x2 (hardware SPI)
//   EnvSensor: Adafruit BME680 (temp/humidity/pressure/gas) via STEMMA QT
//   Heaters  : Mosfetti 4-ch MOSFET board (PWM, active-high)
//   PSU      : ZK-6522L via TTL UART (Serial1)
//   Storage  : Adafruit XTSD 2GB SPI flash (A4/GPIO14 CS)
//
// UART assignment:
//   Serial  (USB-CDC) — debug / PlatformIO monitor
//   Serial1 (UART1)   — ZK-6522L PSU Modbus (PSU_TX_PIN / PSU_RX_PIN)
//   Serial2 (UART2)   — CrowPanel link      (CROWPANEL_TX_PIN / CROWPANEL_RX_PIN)
// =============================================================================

// ─── SPI bus (MAX31865 sensors + SD card) ────────────────────────────────────
#define PIN_SPI_SCK     36
#define PIN_SPI_MOSI    35
#define PIN_SPI_MISO    37

// ─── SD card / XTSD flash module ─────────────────────────────────────────────
// Adafruit XTSD 2GB SPI flash — wired to A4 (GPIO14) for clean routing.
// Uses standard Arduino SD library, same as a microSD breakout.
#define SD_CS_PIN       14    // A4

// ─── MAX31865 PT1000 sensors ──────────────────────────────────────────────────
#define NUM_CHANNELS        2
#define MAX31865_CS_CH1     11         // [VERIFY] GPIO for CH1 MAX31865 CS
#define MAX31865_CS_CH2     12         // [VERIFY] GPIO for CH2 MAX31865 CS
#define RTD_NOMINAL         1000.0f    // PT1000 = 1000Ω at 0°C
#define RTD_REFERENCE       4300.0f    // 4300Ω ref resistor on Adafruit MAX31865

// ─── Heater PWM outputs ───────────────────────────────────────────────────────
#define HEATER_CH1_PIN     18          // A0
#define HEATER_CH2_PIN     17          // A1
// PWM frequency: 20Hz chosen to eliminate EMI coupling from the heating element
// into the PT1000 sense wires sharing the same GX12 cable.
// At 1kHz, the 12V PWM produced ~12,000 spikes/sec on the sense lines, pushing
// FORCE+/FORCE−/RTDIN± outside the MAX31865's ±0.5V headroom → fault 0x04.
// A 22Ω resistive pad has a thermal time constant of several seconds — it is
// completely insensitive to whether the PWM runs at 20Hz or 1kHz.  The PID
// output (0–100% duty) maps identically; only the switching rate changes.
// At 20Hz: 50ms period, switching energy in MAX31865 bandwidth drops ~50×.
//
// IMPORTANT — resolution must be 12-bit for 20Hz to be achievable:
// ESP32-S3 LEDC uses a 10-bit clock divider (max = 1023).
//   8-bit + 80MHz APB → minimum frequency = 305 Hz (divider = 15625 → FAILS)
//  12-bit + 80MHz APB → minimum frequency =  19 Hz (divider =   977 → OK)
// Using 8-bit at 20Hz causes ledcSetup() to fail silently and the LEDC channel
// is never configured → GPIO stays LOW → Mosfetti never turns on.
#define PWM_FREQ_HZ         20
#define PWM_RESOLUTION      12        // 12-bit = 4096 steps; required for 20Hz
#define PWM_MAX_DUTY        4095      // 2^12 - 1; duty = raw_float * 4095
#define LEDC_CH_HEATER1     0
#define LEDC_CH_HEATER2     1

// ─── PSU: ZK-6522L via TTL UART ── Serial1 ───────────────────────────────────
// Wiring (3 wires, no RS-485 chip):
//   PSU GND  ──────────────────────  Feather GND
//   PSU TX   ──[10kΩ]──┬──────────  Feather PSU_RX_PIN
//                       └──[20kΩ]── GND   (5V → 3.3V divider)
//   PSU RX   ←─────────────────────  Feather PSU_TX_PIN  (3.3V; direct)
// PSU connector XH1.25: Temp · GND · RX · TX · +5V (connect GND/RX/TX only)
// Ensure RX–GND jumper on PSU motherboard is OPEN = digital Modbus mode.
#define PSU_TX_PIN      16     // [VERIFY] Feather TX → PSU RX
#define PSU_RX_PIN      15     // [VERIFY] Feather RX ← PSU TX (via divider)
#define PSU_BAUD        115200
#define PSU_SLAVE_ID    1
#define PSU_V_MIN       0.0f
#define PSU_V_MAX       12.0f  // software ceiling — battery charging max
#define PSU_I_MAX       20.0f  // current setpoint written to PSU on boot
#define PSU_V_OVP       13.0f  // OVP protection register ceiling (1V headroom)
#define PSU_I_OCP       21.0f  // OCP protection register ceiling (1A headroom)
#define PSU_POLL_MS     200

// ─── CrowPanel UART link ── Serial2 ──────────────────────────────────────────
// Uses the dedicated hardware RX/TX pins at the bottom of the Feather board.
// GPIO39 = TX (labeled "TX" on board silkscreen)
// GPIO38 = RX (labeled "RX" on board silkscreen)
// DO NOT use GPIO33 (NeoPixel data) or GPIO21 (NeoPixel power enable).
// 3 wires: TX, RX, GND.  Both sides 3.3V — no level shift needed.
// CrowPanel UART1 header: GPIO19 TX (confirmed), RX pin [VERIFY schematic]
#define CROWPANEL_TX_PIN    39     // Feather dedicated TX → CrowPanel UART1 RX
#define CROWPANEL_RX_PIN    38     // Feather dedicated RX ← CrowPanel UART1 TX
#define CROWPANEL_BAUD      115200
#define CROWPANEL_POLL_MS   200

// ─── WiFi — Access Point mode ─────────────────────────────────────────────────
// The Feather creates its own WiFi network — no router needed.
// Connect to WIFI_AP_SSID, open http://192.168.4.1/dash
// or use mDNS:  http://psuwarmer.local/dash
#define WIFI_AP_SSID        "PSU-Warmer"
#define WIFI_AP_PASSWORD    "warmer123"
#define WIFI_HTTP_PORT      80
#define WIFI_POLL_MS        100

// ─── PID parameters ───────────────────────────────────────────────────────────
#define PID_KP_DEFAULT      0.15f
#define PID_KI_DEFAULT      0.002f
#define PID_KD_DEFAULT      0.05f
#define PID_IMAX            0.10f
#define PID_OUT_MIN         -0.50f
#define PID_OUT_MAX         1.0f
#define PID_INTERVAL_MS     1000

// ─── Temperature settings ────────────────────────────────────────────────────
#define SETPOINT_DEFAULT_F  100.0f
#define SETPOINT_MIN_F       90.0f
#define SETPOINT_MAX_F      115.0f
#define TEMP_STEP_F           1.0f
#define SAFETY_CUTOFF_F     120.0f
#define SENSOR_FAIL_LIMIT     10

// ─── Feedforward learner ─────────────────────────────────────────────────────
#define FF_SLOTS            26
#define THERMAL_LAG_SECONDS  10.0f
#define FF_LEARN_RATE        0.02f
#define FF_STEADY_BAND_F     0.5f
#define FF_BOOTSTRAP_PCT     0.18f
#define FF_SAVE_INTERVAL_MS  60000

// ─── Logging ─────────────────────────────────────────────────────────────────
#define LOG_INTERVAL_MS      500

// ─── BME680 Environmental Sensor — STEMMA QT (Wire: SDA=GPIO3, SCL=GPIO4) ────
// Adafruit BME680 breakout default I2C address (SDO low = 0x77, SDO high = 0x76)
#define BME680_I2C_ADDR         0x77
#define BME680_POLL_MS          3000   // 3s — gas heater needs ~150ms per reading

// EMA smoothing — adjust to taste
// Time constant ≈ poll_interval / α  (e.g. 0.1 @ 3s → ~30s)
#define BME680_EMA_ALPHA_FAST   0.1f   // temperature, humidity (moderate smoothing)
#define BME680_EMA_ALPHA_SLOW   0.05f  // pressure, gas resistance (heavier smoothing)

// ─── NVS ─────────────────────────────────────────────────────────────────────
#define NVS_NAMESPACE   "bw_ctrl"

// ─── Serial debug helper ──────────────────────────────────────────────────────
#define DBG_PRINTF(fmt, ...) do { char _b[256]; snprintf(_b, sizeof(_b), fmt, ##__VA_ARGS__); Serial.print(_b); } while(0)