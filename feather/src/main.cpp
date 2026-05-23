// =============================================================================
// main.cpp — PSU Warmer Controller (ESP32-S3 Feather, headless)
//
// WiFi: Access Point mode.
//   Connect to "PSU-Warmer" / "warmer123"
//   Dashboard: http://192.168.4.1/dash  or  http://psuwarmer.local/dash
//
// UART assignment:
//   Serial  (USB-CDC) — debug / PlatformIO monitor
//   Serial1 (UART1)   — ZK-6522L PSU  (psu.begin() handles Serial1.begin())
//   Serial2 (UART2)   — CrowPanel 5"  (CROWPANEL_TX_PIN / CROWPANEL_RX_PIN)
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SD.h>
#include <Adafruit_BME680.h>

#include "config.h"
#include "sensors.h"
#include "channel.h"
#include "psu.h"
#include "logger.h"
#include "wifi_server.h"

// ── Global objects ────────────────────────────────────────────────────────────
static WarmSensorBus   bus;
static HeatingChannel  ch1(0, HEATER_CH1_PIN, LEDC_CH_HEATER1, "ff_ch0");
static HeatingChannel  ch2(1, HEATER_CH2_PIN, LEDC_CH_HEATER2, "ff_ch1");
static ZkPsu           psu;
static WarmLogger      logger;
static WarmWiFiServer  wifiSrv;
static Adafruit_BME680 bme;

// ── Timing ────────────────────────────────────────────────────────────────────
static unsigned long _lastPIDms       = 0;
static unsigned long _lastPSUms       = 0;
static unsigned long _lastLogms       = 0;
static unsigned long _lastWiFims      = 0;
static unsigned long _lastCrowPanelms = 0;
static uint32_t      _rtcEpoch     = 0;   // Unix epoch received from CrowPanel RTC
static unsigned long _rtcBootMs    = 0;   // millis() when epoch was last synced
static unsigned long _lastDiagms      = 0;
static unsigned long _lastBMEms       = 0;
static uint8_t       _initPhase       = 0;

// ── BME680 state ──────────────────────────────────────────────────────────────
static bool  _bmeOk        = false;
static float _bmeTempF     = 0.f;    // EMA-smoothed °F
static float _bmeHumidity  = 0.f;    // EMA-smoothed %RH
static float _bmePressHpa  = 0.f;    // EMA-smoothed hPa
static float _bmeGasKohm   = 0.f;    // EMA-smoothed kΩ (higher = cleaner air)
static bool  _bmeSeeded    = false;  // true after first valid reading seeds the EMA
static uint32_t _bmeEndTime  = 0;    // millis() when current measurement should be ready
static bool  _bmeMeasuring   = false; // true while waiting for endReading()

// ── Change-detection for serial CSV ──────────────────────────────────────────
struct ChSnapshot {
    const char* status = "?";
    float       tempF  = -9999.0f;
    float       duty   = -1.0f;
};
static ChSnapshot _snap[2];
static bool       _firstLog = true;

static bool channelChanged(const HeatingChannel& ch, const ChSnapshot& snap) {
    if (strcmp(ch.statusStr(), snap.status) != 0) return true;
    float t = isnan(ch.displayTempF) ? -999.0f : ch.displayTempF;
    if (fabsf(t - snap.tempF) >= 0.3f)      return true;
    if (fabsf(ch.duty - snap.duty) >= 0.01f) return true;
    return false;
}

// ── CrowPanel telemetry (Serial2) ─────────────────────────────────────────────
static void sendCrowPanelTelemetry() {
    float t1 = isnan(ch1.displayTempF) ? -999.0f : ch1.displayTempF;
    float t2 = isnan(ch2.displayTempF) ? -999.0f : ch2.displayTempF;
    // Compute current epoch from last RTC sync + elapsed millis
    uint32_t nowEpoch = (_rtcEpoch > 0)
        ? _rtcEpoch + (uint32_t)((millis() - _rtcBootMs) / 1000)
        : 0;

    char buf[320];
    int len = snprintf(buf, sizeof(buf),
        "{\"c1\":{\"t\":%.1f,\"s\":%.1f,\"d\":%d,\"st\":\"%s\",\"r\":%.2f},"
         "\"c2\":{\"t\":%.1f,\"s\":%.1f,\"d\":%d,\"st\":\"%s\",\"r\":%.2f},"
         "\"psu\":{\"v\":%.2f,\"a\":%.2f,\"w\":%.1f,\"vin\":%.1f,\"sv\":%.2f,"
                  "\"on\":%d,\"cc\":%d,\"ok\":%d},"
         "\"ts\":%lu",
        t1, ch1.setpointF, (int)(ch1.duty*100.0f), ch1.statusStr(), ch1.dTdtF * 60.0f,
        t2, ch2.setpointF, (int)(ch2.duty*100.0f), ch2.statusStr(), ch2.dTdtF * 60.0f,
        psu.measV, psu.measA, psu.measW, psu.inputV, psu.setV,
        psu.outputOn?1:0, psu.isCC?1:0, psu.commsOk?1:0,
        (unsigned long)nowEpoch);
    // Append env field only when BME680 has valid seeded data
    if (_bmeOk && _bmeSeeded && len > 0 && len < (int)sizeof(buf) - 60) {
        len += snprintf(buf + len, sizeof(buf) - len,
            ",\"env\":{\"t\":%.1f,\"h\":%.1f,\"p\":%.1f,\"g\":%.1f}",
            _bmeTempF, _bmeHumidity, _bmePressHpa, _bmeGasKohm);
    }
    if (len > 0 && len < (int)sizeof(buf) - 2) {
        buf[len++] = '}';
        buf[len++] = '\n';
        buf[len]   = '\0';
    }
    Serial2.print(buf);
}

static void handleCrowPanelCommand(char* buf) {
    char* cmdPtr = strstr(buf, "\"cmd\":\"");
    if (!cmdPtr) return;
    cmdPtr += 7;
    char cmd[24] = {};
    int i = 0;
    while (*cmdPtr && *cmdPtr != '"' && i < 23) cmd[i++] = *cmdPtr++;
    float v = 0.0f;
    char* vPtr = strstr(buf, "\"v\":");
    if (vPtr) v = atof(vPtr + 4);

    if      (strcmp(cmd, "psu_sv") == 0) psu.setVoltage(v);
    else if (strcmp(cmd, "psu_on") == 0) psu.setOutput(v > 0.5f);
    else if (strcmp(cmd, "c1_sp")  == 0) ch1.adjustSetpoint(v - ch1.setpointF);
    else if (strcmp(cmd, "c1_en")  == 0) ch1.enabled = (v > 0.5f);
    else if (strcmp(cmd, "c2_sp")  == 0) ch2.adjustSetpoint(v - ch2.setpointF);
    else if (strcmp(cmd, "c2_en")  == 0) ch2.enabled = (v > 0.5f);
    else if (strcmp(cmd, "set_rtc") == 0) {
        // Web GUI → Feather → CrowPanel: CrowPanel confirms RTC written
        // (No action needed here — web GUI already updated _rtcEpoch via poll)
    }
    else if (strcmp(cmd, "rtc_epoch") == 0) {
        // Parse directly as uint32_t — float path loses precision on 10-digit epochs.
        // millis() is captured at exact receipt time as the clock reference.
        // This is the canonical source of truth — not overwritten by loop polling.
        const char* ep = strstr(buf, "\"v\":");
        uint32_t rxEpoch = ep ? (uint32_t)atol(ep + 4) : 0;
        if (rxEpoch > 1577836800UL) {  // sanity: after Jan 2020
            _rtcEpoch  = rxEpoch;
            _rtcBootMs = millis();
            logger.setEpoch(_rtcEpoch, _rtcBootMs);
            DBG_PRINTF("[rtc] epoch %lu synced (ms ref=%lu)\n",
                       (unsigned long)_rtcEpoch, _rtcBootMs);
        } else {
            DBG_PRINTF("[rtc] rtc_epoch rejected: %lu\n", (unsigned long)rxEpoch);
        }
    }
    else DBG_PRINTF("[panel] Unknown cmd: %s\n", cmd);
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    { unsigned long t = millis(); while (!Serial && millis()-t < 1000) delay(5); }

    Serial.println("\n========================================");
    Serial.println(" Battery Warmer v2.0 — Headless / AP");
    Serial.println("========================================");
    Serial.printf("CPU: %lu MHz   Flash: %uMB   Heap: %u\n",
                  ESP.getCpuFreqMHz(),
                  ESP.getFlashChipSize()/(1024*1024),
                  ESP.getFreeHeap());

    // ── SPI ───────────────────────────────────────────────────────────────────
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // ── I2C / BME680 ──────────────────────────────────────────────────────────
    // STEMMA QT uses default Wire pins: SDA=GPIO3, SCL=GPIO4
    Wire.begin();
    _bmeOk = bme.begin(BME680_I2C_ADDR, &Wire);
    if (_bmeOk) {
        // Configure oversampling and filter
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);  // 320°C for 150ms — standard VOC measurement
        Serial.printf("[init] BME680 found at 0x%02X\n", BME680_I2C_ADDR);
    } else {
        Serial.println("[init] BME680 WARN: not found — check wiring");
    }

    // ── SD card ───────────────────────────────────────────────────────────────
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[SD] WARN: not found");
    } else {
        uint8_t ct = SD.cardType();
        const char* cs = (ct==CARD_MMC)?"MMC":(ct==CARD_SD)?"SD":(ct==CARD_SDHC)?"SDHC":"?";
        Serial.printf("[SD] Ready — %s  %lluMB\n", cs, SD.cardSize()/(1024*1024));
    }

    // ── Sensors + channels ────────────────────────────────────────────────────
    Serial.println("[init] Sensors...");
    bus.begin();
    bus.diagnose();
    Serial.println("[init] Channels...");
    ch1.begin();
    ch2.begin();

    // ── PSU — Serial1 ─────────────────────────────────────────────────────────
    Serial.println("[init] PSU...");
    psu.begin();

    // ── CrowPanel — Serial2 ───────────────────────────────────────────────────
    Serial2.begin(CROWPANEL_BAUD, SERIAL_8N1, CROWPANEL_RX_PIN, CROWPANEL_TX_PIN);
    Serial.printf("[init] CrowPanel UART2  TX=GPIO%d  RX=GPIO%d\n",
                  CROWPANEL_TX_PIN, CROWPANEL_RX_PIN);

    // ── WiFi AP ───────────────────────────────────────────────────────────────
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    delay(500);
    Serial.printf("[wifi] AP ready: SSID=\"%s\"  IP=%s\n",
                  WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

    // ── mDNS ──────────────────────────────────────────────────────────────────
    if (MDNS.begin("psuwarmer")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("[wifi] mDNS: http://psuwarmer.local/dash");
    } else {
        Serial.println("[wifi] mDNS: failed — use IP address instead");
    }
    Serial.printf("[wifi] Dashboard: http://%s/dash\n",
                  WiFi.softAPIP().toString().c_str());

    Serial.printf("[init] Done — heap: %u\n", ESP.getFreeHeap());
    Serial.println("t_ms,ch1_tempF,ch1_setptF,ch1_errorF,ch1_duty_pct,"
                   "ch1_ff_pct,ch1_dtdt,ch1_status,"
                   "ch2_tempF,ch2_setptF,ch2_errorF,ch2_duty_pct,"
                   "ch2_ff_pct,ch2_dtdt,ch2_status");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    unsigned long now = millis();

    // ── Deferred: SD logger → HTTP server ────────────────────────────────────
    if (_initPhase == 0 && now > 500) {
        bool logOk = logger.begin();
        Serial.printf("[init] Logger: %s\n", logOk ? "OK" : "FAILED");
        _initPhase = 1;
    } else if (_initPhase == 1 && now > 1000) {
        wifiSrv.begin();
        wifiSrv.setLogger(&logger);
        wifiSrv.setPsu(&psu);
        wifiSrv.setCrowPanelSerial(&Serial2);
        logger.flush();
        _initPhase = 2;
        Serial.println("[wifi] HTTP server listening");
    }

    // ── PID at 1Hz ────────────────────────────────────────────────────────────
    // Skip MAX31865 reads while the SD logger is busy (flush in progress or
    // within 100ms of completing). The XTSD NAND flash's internal program
    // cycle after an SD write can disturb the 3.3V rail or shared SPI bus,
    // causing transient MAX31865 fault codes (0x04 overvoltage / 0x18 RTD).
    // Skipping one 1Hz PID sample is harmless — the previous temperature
    // reading remains valid for the PID update.
    if (now - _lastPIDms >= PID_INTERVAL_MS) {
        if (!logger.isBusy()) {
            ch1.injectTemp(bus.readCelsius(0));
            ch2.injectTemp(bus.readCelsius(1));
        }
        ch1.updatePID(now);
        ch2.updatePID(now);
        _lastPIDms = now;
    }

    // ── BME680 — non-blocking measurement ────────────────────────────────────
    // beginReading() triggers the sensor and returns immediately.
    // endReading() is polled on subsequent loop() calls — no blocking.
    // This keeps the HTTP server and UART fully responsive during the
    // ~150ms gas heater warmup that performReading() used to block on.
    if (_bmeOk) {
        if (!_bmeMeasuring && now - _lastBMEms >= BME680_POLL_MS) {
            _bmeEndTime   = bme.beginReading();
            _bmeMeasuring = (_bmeEndTime != 0);
        }
        if (_bmeMeasuring && millis() >= _bmeEndTime) {
            if (bme.endReading()) {
                float newTempF    = bme.temperature * 9.0f / 5.0f + 32.0f;
                float newHumidity = bme.humidity;
                float newPressHpa = bme.pressure / 100.0f;
                float newGasKohm  = bme.gas_resistance / 1000.0f;
                if (!_bmeSeeded) {
                    _bmeTempF    = newTempF;
                    _bmeHumidity = newHumidity;
                    _bmePressHpa = newPressHpa;
                    _bmeGasKohm  = newGasKohm;
                    _bmeSeeded   = true;
                } else {
                    _bmeTempF    = BME680_EMA_ALPHA_FAST * newTempF    + (1.0f - BME680_EMA_ALPHA_FAST) * _bmeTempF;
                    _bmeHumidity = BME680_EMA_ALPHA_FAST * newHumidity + (1.0f - BME680_EMA_ALPHA_FAST) * _bmeHumidity;
                    _bmePressHpa = BME680_EMA_ALPHA_SLOW * newPressHpa + (1.0f - BME680_EMA_ALPHA_SLOW) * _bmePressHpa;
                    _bmeGasKohm  = BME680_EMA_ALPHA_SLOW * newGasKohm  + (1.0f - BME680_EMA_ALPHA_SLOW) * _bmeGasKohm;
                }
            }
            wifiSrv.setEnv(_bmeTempF, _bmeHumidity, _bmePressHpa, _bmeGasKohm, _bmeSeeded);
            _bmeMeasuring = false;
            _lastBMEms    = now;
        }
    }

    // ── PSU poll at 5Hz ───────────────────────────────────────────────────────
    if (now - _lastPSUms >= PSU_POLL_MS) {
        psu.poll();
        _lastPSUms = now;
    }

    // ── CrowPanel telemetry + command parse ───────────────────────────────────
    if (now - _lastCrowPanelms >= CROWPANEL_POLL_MS) {
        sendCrowPanelTelemetry();
        _lastCrowPanelms = now;
    }
    static char _cpBuf[128];
    static int  _cpPos = 0;
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') {
            _cpBuf[_cpPos] = '\0';
            if (_cpPos > 2) handleCrowPanelCommand(_cpBuf);
            _cpPos = 0;
        } else if (_cpPos < 127) {
            _cpBuf[_cpPos++] = c;
        }
    }

    // ── CSV log at 2Hz ────────────────────────────────────────────────────────
    if (now - _lastLogms >= LOG_INTERVAL_MS) {
        float t1 = isnan(ch1.displayTempF) ? -999.0f : ch1.displayTempF;
        float t2 = isnan(ch2.displayTempF) ? -999.0f : ch2.displayTempF;
        logger.log(now,
                   _rtcEpoch > 0 ? _rtcEpoch + (uint32_t)((millis() - _rtcBootMs) / 1000) : 0,
            t1, ch1.setpointF, ch1.duty, ch1.ffTerm(), ch1.dTdtF, ch1.statusStr(),
            t2, ch2.setpointF, ch2.duty, ch2.ffTerm(), ch2.dTdtF, ch2.statusStr(),
            psu.measV, psu.measA, psu.measW, psu.inputV, psu.setV,
            psu.outputOn, psu.isCC,
            _bmeSeeded,
            _bmeTempF, _bmeHumidity, _bmePressHpa, _bmeGasKohm);
        if (_firstLog || channelChanged(ch1,_snap[0]) || channelChanged(ch2,_snap[1])) {
            Serial.printf("%lu,%.2f,%.1f,%.2f,%.1f,%.1f,%.4f,%s,"
                              "%.2f,%.1f,%.2f,%.1f,%.1f,%.4f,%s\n",
                now,
                t1, ch1.setpointF, (t1>-998.0f)?ch1.setpointF-t1:0.0f,
                ch1.duty*100.0f, ch1.ffTerm()*100.0f, ch1.dTdtF, ch1.statusStr(),
                t2, ch2.setpointF, (t2>-998.0f)?ch2.setpointF-t2:0.0f,
                ch2.duty*100.0f, ch2.ffTerm()*100.0f, ch2.dTdtF, ch2.statusStr());
            _snap[0].status = ch1.statusStr(); _snap[0].tempF = t1; _snap[0].duty = ch1.duty;
            _snap[1].status = ch2.statusStr(); _snap[1].tempF = t2; _snap[1].duty = ch2.duty;
            _firstLog = false;
        }
        _lastLogms = now;
    }

    // ── HTTP server poll ──────────────────────────────────────────────────────
    if (_initPhase >= 2 && now - _lastWiFims >= WIFI_POLL_MS) {
        wifiSrv.poll(&ch1, &ch2);
        // Keep web server epoch in sync
        // Pick up epoch set via web GUI /settime (server stores it separately)
        // Pick up epoch set via web GUI /settime
        uint32_t webEpoch = wifiSrv.rtcSetEpoch();
        if (webEpoch > 0 && webEpoch != _rtcEpoch) {
            // Use current millis() as reference — rtcSetMs() is stale by
            // the time we poll it, which would cause the clock to jump ahead.
            // webEpoch is already timezone-adjusted from the browser JS.
            _rtcEpoch  = webEpoch;
            _rtcBootMs = millis();
            logger.setEpoch(_rtcEpoch, _rtcBootMs);
            DBG_PRINTF("[rtc] epoch from web GUI: %lu\n", (unsigned long)_rtcEpoch);
        }
        // Update web server display epoch — lightweight, just two assignments
        if (_rtcEpoch > 0) wifiSrv.setCurrentEpoch(_rtcEpoch, _rtcBootMs);
        // Note: logger.setEpoch() is NOT called here — calling it every loop
        // iteration resets _epochMs to now, preventing the clock from advancing.
        _lastWiFims = now;
    }

    // ── 60s diagnostics ───────────────────────────────────────────────────────
    if (now - _lastDiagms >= 60000UL) {
        _lastDiagms = now;
        Serial.println("[diag] ─────────────────────────────────────");
        Serial.printf("[diag] Uptime: %lus  Heap: %u  AP clients: %d\n",
                      now/1000, ESP.getFreeHeap(), WiFi.softAPgetStationNum());
        Serial.printf("[diag] PSU: %s  Vout=%.2f  Aout=%.2f  W=%.1f  Vin=%.1f  setV=%.2f  on=%d  cc=%d\n",
                      psu.statusStr(), psu.measV, psu.measA, psu.measW,
                      psu.inputV, psu.setV, psu.outputOn, psu.isCC);
        for (int i = 0; i < 2; i++) {
            HeatingChannel& ch = (i==0) ? ch1 : ch2;
            Serial.printf("[diag] CH%d: %s  %.1fF→%.0fF  duty=%.0f%%  ff=%.0f%%\n",
                          i+1, ch.statusStr(),
                          isnan(ch.displayTempF)?-999.0f:ch.displayTempF,
                          ch.setpointF, ch.duty*100.0f, ch.ffTerm()*100.0f);
        }
        if (_bmeSeeded) {
            Serial.printf("[diag] ENV: %.1fF  %.1f%%RH  %.1fhPa  %.1fkΩ (%s)\n",
                          _bmeTempF, _bmeHumidity, _bmePressHpa, _bmeGasKohm,
                          _bmeGasKohm >= 300.f ? "Excellent" :
                          _bmeGasKohm >= 150.f ? "Good"      :
                          _bmeGasKohm >=  50.f ? "Fair"      :
                          _bmeGasKohm >=  25.f ? "Poor"      : "Bad");
        } else {
            Serial.println("[diag] ENV: awaiting first BME680 reading");
        }
        Serial.println("[diag] ─────────────────────────────────────");
    }
}