# PSU Warmer — Battery Warmer Controller with Touchscreen PSU Interface

A dual-channel PID battery warmer controller built on an **Adafruit ESP32-S3 Feather**, extended with a **5" Elecrow CrowPanel Advance touchscreen** as a PSU controller UI. Designed for RC racing — warms LiPo batteries to target temperature before race sessions, with real-time logging and a web dashboard.

---

## Hardware

### Feather (Main Controller)
| Component | Detail |
|-----------|--------|
| MCU | Adafruit ESP32-S3 Feather (2MB PSRAM, 4MB Flash) |
| Heating channels | 2× PWM-controlled MOSFETs, 20Hz/12-bit |
| Temperature sensors | 2× MAX31865 PT1000 via SPI |
| Environmental sensor | BME680 (temp/humidity/pressure/gas) via I2C (SDA=3, SCL=4) |
| PSU interface | ZK-6522L programmable DC PSU via UART1 (TX=16, RX=15) |
| CrowPanel link | Serial2 remapped to IO43(TX)/IO44(RX) — shares CH340K debug pins, works fine in practice |
| SD card logging | GPIO14 CS |
| SPI bus | SCK=36, MOSI=35, MISO=37 |
| MAX31865 CS | CH1=11, CH2=12 |
| Heater PWM | CH1=18, CH2=17 |
| WiFi | AP mode — SSID: `PSU-Warmer`, Password: `warmer123` |
| Web dashboard | http://192.168.4.1/dash (mDNS: http://psuwarmer.local/dash) |

### CrowPanel Advance 5" (Display)
| Component | Detail |
|-----------|--------|
| Board | Elecrow DIS02050A V1.1, IPS, ST7262, 16MB flash, 8MB PSRAM |
| Display driver | LovyanGFX 1.2.19, 15MHz PCLK, SRAM draw buffers (800×10 lines) |
| Touch | GT911 capacitive, I2C at 0x5D on GPIO15/16 via LovyanGFX I2C_NUM_0 |
| **GT911 I2C frequency** | **100kHz** — critical, 400kHz causes cold power-up touch failure |
| Backlight | STC8H1K28 at I2C 0x30, bit-bang I2C post-tft.begin() |
| Backlight protocol | V1.1: 0x05=off, 0x06–0x0F=dim–bright, 0x10=max |
| RTC | PCF8563 at I2C 0x51 — battery-backed UTC clock |
| Feather link | IO43(RX)/IO44(TX) — UART remapped via GPIO matrix |
| GPIO19/20 | **Unusable for UART** — ESP32-S3 USB PHY hardwired to these pins |

### GX12 5-Pin Connector (Warming Pad Harness)
| Pin | Function |
|-----|----------|
| 1, 2 | Heating element (22Ω, to MOSFET output) |
| 3, 4 | PT1000 sense pair (~0Ω, to MAX31865 F+/F−) |
| 5 | PT1000 RTD return wire (~1000Ω+, to MAX31865 RTD+) |

---

## Software Architecture

### Build System
- **PlatformIO** with Arduino framework
- Two separate PlatformIO environments: `feather_esp32s3` and `crowpanel_5`
- Key libraries: LovyanGFX 1.2.19, LVGL 8.4.0, ModbusMaster, MAX31865, BME680

### Feather Firmware (`feather/src/`)
| File | Purpose |
|------|---------|
| `main.cpp` | Main loop, PID control, sensor polling, telemetry |
| `wifi_server.h` | HTTP server, web dashboard SPA, all API endpoints |
| `logger.h` | SD card CSV logger with date-based filenames |
| `config.h` | Pin definitions and timing constants |
| `psu.h` | ZK-6522L PSU driver class |

### CrowPanel Firmware (`crowpanel/src/`)
| File | Purpose |
|------|---------|
| `main.cpp` | LVGL UI, telemetry parsing, RTC management, touch handling |
| `display_driver.cpp` | LovyanGFX init, GT911 touch, PCF8563 RTC read/write, backlight |
| `display_driver.h` | Public interface for display functions |
| `lv_conf.h` | LVGL configuration |

---

## Telemetry Protocol (Feather → CrowPanel, 200ms interval)

```json
{
  "c1": {"t": 75.3, "s": 100.0, "d": 23, "st": "HEATING", "r": 0.34},
  "c2": {"t": 74.1, "s": 100.0, "d": 21, "st": "HEATING", "r": 0.31},
  "psu": {"v": 12.00, "a": 1.50, "w": 18.0, "vin": 14.2, "sv": 12.00, "on": 1, "cc": 0, "ok": 1},
  "ts": 1748503200,
  "env": {"t": 72.3, "h": 45.1, "p": 1013.2, "g": 125.4}
}
```

- `ts` is a **true UTC Unix epoch** — used by Feather logger for ISO timestamps and log file naming
- `env` object is optional, only present when BME680 is wired and seeded

## Command Protocol (CrowPanel → Feather)

```json
{"cmd": "psu_sv",    "v": 12.00}   // set PSU voltage
{"cmd": "psu_on",    "v": 1}       // PSU output on/off
{"cmd": "c1_en",     "v": 1}       // channel enable
{"cmd": "c1_sp",     "v": 102.0}   // channel setpoint °F
{"cmd": "rtc_epoch", "v": 1748503200}  // CrowPanel → Feather: boot epoch sync
{"cmd": "set_rtc",   "v": 1748503200, "tz": -240}  // Feather → CrowPanel: set time
```

---

## RTC / Timekeeping Architecture

**PCF8563** (battery-backed, on CrowPanel GPIO15/16) is the primary time source.

### How it works:
1. **Set Time** — user clicks ⏰ Set Time in web dashboard → browser sends true UTC epoch + timezone offset in minutes → Feather forwards `set_rtc` command to CrowPanel → CrowPanel suspends LVGL touch briefly → writes PCF8563 immediately via WireBL → re-enables touch → stores timezone offset in NVS
2. **Boot** — CrowPanel reads PCF8563 via WireBL (before `tft.begin()`) → sends `rtc_epoch` to Feather → periodic resend every 3s until Feather echoes back non-zero `ts` field
3. **Feather** — receives `rtc_epoch`, stores UTC epoch + `millis()` reference in RAM → free-runs clock from that point → includes `ts` field in all telemetry
4. **Display** — UTC epoch + timezone offset applied at render time in `_fmtNowDatetime()`

### Critical I2C constraints:
- WireBL (I2C_NUM_1) used for PCF8563 and backlight — **must call WireBL.end() before tft.begin()**
- LovyanGFX takes GPIO15/16 for GT911 (I2C_NUM_0) after tft.begin()
- PCF8563 VL flag checked on read — time discarded if chip lost power
- **GT911 must run at 100kHz** — 400kHz causes I2C bus instability on cold power-up
- Runtime PCF8563 writes: suspend LVGL indev → WireBL.begin() → write → WireBL.end() → re-enable indev

---

## SD Card Logging

### File naming
- With epoch: `LOG_YYYYMMDD_HHMMSS.CSV` (UTC timestamp at file creation)
- Without epoch: `LOG_NORTC_NNNN.CSV` (sequential, scans SD for highest existing)

### CSV columns (27 total)
```
t_ms, iso_time,
ch1_tempF, ch1_setptF, ch1_errorF, ch1_duty_pct, ch1_ff_pct, ch1_dtdt, ch1_status,
ch2_tempF, ch2_setptF, ch2_errorF, ch2_duty_pct, ch2_ff_pct, ch2_dtdt, ch2_status,
psu_vout, psu_aout, psu_watts, psu_vin, psu_setv, psu_on, psu_cc,
env_tempF, env_humidity, env_pressHpa, env_gasKohm
```

### Logger behavior
- **Lazy file creation** — waits up to 5s from first `log()` call for a valid epoch before falling back to NORTC
- **Mid-session epoch switch** — if NORTC file is open and epoch arrives, flushes and opens a new dated file
- **RAM buffer** — 4KB buffer, flushed every 10 rows or 5 seconds
- **SD guard** — 100ms busy flag after each flush to protect MAX31865 SPI reads
- **Rotation** — new file created at 50MB; oldest file deleted when SD free space < 100MB

---

## Web Dashboard

Base URL: `http://192.168.4.1` or `http://psuwarmer.local`

| Endpoint | Description |
|----------|-------------|
| `/dash` | Main SPA dashboard (4 tabs: Dashboard, PSU, Warmer, Env) |
| `/api` | JSON telemetry + log status + current UTC time |
| `/logview` | Log file browser — sidebar file list, inline 300-row table |
| `/logview?file=LOG_*.CSV` | View specific log file |
| `/log` | Download current log file as CSV |
| `/log?file=LOG_*.CSV` | Download specific log file |
| `/settime?t=<utc>&tz=<min>` | Set RTC time (UTC epoch + tz offset in minutes east) |
| `/clearlog` | Delete all log files, returns `{"ok":true,"deleted":N}` |
| `/fftable` | Feedforward learning table viewer |
| `/diag` | Diagnostics page |
| `/control` | Channel/PSU control endpoint |

### Footer links
⚙ FF Table | 🔧 Diag | 📋 View Log | ⬇ Download Log | ⏰ Set Time | 🗑 Clear Logs

---

## Known Issues / Quirks

### CrowPanel specific
- **USB CDC on boot must be disabled** — `build_unflags = -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1` in platformio.ini. Serial monitor works via CH340K on UART0.
- **First touch press ignored** — 1200ms ghost filter in `lvgl_touch_read()` callback absorbs GT911 power-on ghost state
- **GT911 at 100kHz is mandatory** — higher frequencies cause cold power-up touch failure (Elecrow forum confirmed)
- **IO43/IO44 shared with CH340K** — debug prints from CrowPanel appear on the UART link but Feather ignores non-JSON data gracefully
- **Backlight bit-bang I2C** — must use direct GPIO bit-bang after `tft.begin()`, not WireBL

### Feather specific
- **GPIO19/20 unusable for UART** — ESP32-S3 USB PHY is hardwired to those pins at silicon level
- **SD flush guard** — `logger.isBusy()` must be checked before MAX31865 reads to prevent SPI bus contention

---

## ZK-6522L PSU Interface

- **Interface:** 5V TTL UART (not RS-485, not Modbus RTU)
- **Baud rate:** 115,200
- **Register scaling:** voltage/current ÷100, power ÷10
- Connected to Feather Serial1 (TX=16, RX=15)

---

## Planned Features (Future Sessions)

- **OTA firmware updates** — ArduinoOTA over WiFi AP, eliminates USB flashing
- **WiFi STA + NTP** — connect to phone hotspot, auto-sync time (eliminates Set Time button)
- **Weather API** — OpenWeatherMap integration for track ambient conditions
- **Live data view** — real-time log table in web dashboard (poll-based, no persistent connection)
- **Live graphs** — temperature and power charts in web dashboard
- **RP2040 USB-HID bridge** — iCharger DUO/DX series integration via I2C on Feather STEMMA QT

---

## Secrets / Credentials

WiFi AP credentials are hardcoded (local AP only, not home network):
```cpp
#define WIFI_AP_SSID     "PSU-Warmer"
#define WIFI_AP_PASSWORD "warmer123"
```

**Before adding home WiFi/hotspot credentials for NTP:** move credentials to `secrets.h` and add to `.gitignore`.

---

## Development Notes

- Platform decisions should be driven by actual pin tables, not marketing specs
- Verify hardware interfaces empirically before assuming defaults (this project pivoted: analog → Modbus → TTL UART for PSU comms)
- Integrated RGB display boards consume nearly all GPIO — SPI panels are better for GPIO-intensive projects
- When hitting CrowPanel-specific issues, check the [Elecrow forums](https://forum.elecrow.com/) first — many quirks are documented there
