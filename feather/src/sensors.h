#pragma once
#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include "config.h"

// =============================================================================
// WarmSensorBus — two MAX31865 PT1000 RTD amplifiers on hardware SPI
//
// Ported from GIGA R1 sensors.h.
// GIGA used software SPI (mbed); ESP32-S3 uses hardware SPI via SPI.begin()
// in main.cpp — Adafruit_MAX31865 uses hardware SPI when constructed with
// only the CS pin argument.
//
// Hardware (same as GIGA R1):
//   Adafruit MAX31865 breakout (#3648) + PT1000 sensor (#3984)
//   Board mods required:
//     1. Replace 430Ω ref resistor → 4300Ω
//     2. Bridge the 2/3W solder jumper for your wire configuration
//
// Wiring to ESP32-S3 Feather:
//   MAX31865 CLK  → SCK  (GPIO36)
//   MAX31865 SDI  → MOSI (GPIO35)
//   MAX31865 SDO  → MISO (GPIO37)
//   MAX31865 CS   → MAX31865_CS_CH1 / CH2  (see config.h)
//   MAX31865 VIN  → 3.3V
//   MAX31865 GND  → GND
// =============================================================================

class WarmSensorBus {
public:
    // Hardware SPI: constructor takes CS pin only
    WarmSensorBus()
        : _rtd0(MAX31865_CS_CH1)
        , _rtd1(MAX31865_CS_CH2) {}

    void begin() {
        // Change MAX31865_3WIRE to MAX31865_2WIRE or MAX31865_4WIRE to match
        // your actual sensor wiring. GIGA R1 used 3-wire PT1000 sensors.
        _rtd0.begin(MAX31865_3WIRE);
        _rtd1.begin(MAX31865_3WIRE);
        Serial.println("WarmSensorBus: MAX31865 x2 init (PT1000, hardware SPI)");
        Serial.printf("  CS1=GPIO%d  CS2=GPIO%d\n", MAX31865_CS_CH1, MAX31865_CS_CH2);
    }

    void diagnose() {
        Serial.println("=== MAX31865 Diagnostics ===");
        _diagChannel(0, _rtd0);
        _diagChannel(1, _rtd1);
        Serial.println("=== End Diagnostics ===");
    }

    // Returns temperature in Celsius, or NAN on fault/out-of-range
    float readCelsius(int chIdx) {
        Adafruit_MAX31865& rtd = (chIdx == 0) ? _rtd0 : _rtd1;

        uint8_t fault = rtd.readFault();
        if (fault) {
            // Only log fault once per fault event to avoid flooding
            if (fault != _lastFault[chIdx]) {
                DBG_PRINTF("RTD CH%d: fault 0x%02X\n", chIdx+1, fault);
                _lastFault[chIdx] = fault;
            }
            rtd.clearFault();
            return NAN;
        }
        _lastFault[chIdx] = 0;  // clear on successful read

        uint16_t raw = rtd.readRTD();
        // PT1000 at -10°C ≈ 7325 counts, at 80°C ≈ 10200 counts
        if (raw < 6000 || raw > 11500) return NAN;

        float tempC = rtd.temperature(RTD_NOMINAL, RTD_REFERENCE);
        if (tempC < -10.0f || tempC > 80.0f) return NAN;

        return tempC;
    }

    int count() const { return 2; }

private:
    Adafruit_MAX31865 _rtd0;
    Adafruit_MAX31865 _rtd1;
    uint8_t _lastFault[2] = {0, 0};  // track fault state to avoid log flooding

    void _diagChannel(int ch, Adafruit_MAX31865& rtd) {
        Serial.printf("  CH%d: ", ch + 1);
        uint8_t fault = rtd.readFault();
        if (fault) {
            Serial.printf("FAULT 0x%02X\n", fault);
            if (fault & MAX31865_FAULT_HIGHTHRESH) Serial.println("    -> High threshold");
            if (fault & MAX31865_FAULT_LOWTHRESH)  Serial.println("    -> Low threshold");
            if (fault & MAX31865_FAULT_REFINLOW)   Serial.println("    -> REFIN- > 0.85 x VBIAS");
            if (fault & MAX31865_FAULT_REFINHIGH)  Serial.println("    -> REFIN- < 0.85 x VBIAS");
            if (fault & MAX31865_FAULT_RTDINLOW)   Serial.println("    -> RTDIN- < 0.85 x VBIAS (open RTD?)");
            if (fault & MAX31865_FAULT_OVUV)       Serial.println("    -> Over/Under voltage");
            rtd.clearFault();
            return;
        }
        uint16_t raw   = rtd.readRTD();
        float    ratio = raw / 32768.0f;
        float    tempC = rtd.temperature(RTD_NOMINAL, RTD_REFERENCE);
        Serial.printf("OK | raw=0x%04X ratio=%.4f temp=%.2fC (%.2fF)\n",
                      raw, ratio, tempC, tempC * 9.0f / 5.0f + 32.0f);
    }
};