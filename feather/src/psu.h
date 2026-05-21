#pragma once
// =============================================================================
// psu.h — ZK-6522L PSU driver
//
// Physical interface (confirmed from official ZK-6522C manual):
//   TTL UART at 5V — NO RS-485, NO level-shift chip needed.
//
// Wiring (3 wires + voltage divider):
//   PSU GND  ──────────────────────  ESP32 GND
//   PSU TX   ──[10kΩ]──┬──────────  ESP32 RX (PSU_RX_PIN)
//                       └──[20kΩ]── GND       ← divides 5V → 3.33V
//   PSU RX   ←─────────────────────  ESP32 TX (PSU_TX_PIN)
//              (3.3V > 2.0V TTL VIH; direct connection is reliable)
//
// Connector: XH1.25 5-pin on PSU  →  Temp · GND · RX · TX · +5V
//   Connect only GND, RX, TX.  Leave Temp and +5V unconnected.
//
// IMPORTANT: ensure the RX–GND jumper on the PSU motherboard is NOT shorted.
//   Shorted = analogue pot mode (no serial).  Open = digital Modbus mode.
//
// Default serial settings (confirmed from manual):
//   115200 baud  |  8N1  |  Modbus slave address 1
//   Both are configurable via PSU menu → Addr / Baud registers.
//
// Register map (verified from official ZK-6522C manual, page 6):
//   0x0000  V-SET    Voltage setpoint       R/W  raw ÷ 100 = V
//   0x0001  I-SET    Current setpoint       R/W  raw ÷ 100 = A
//   0x0002  VOUT     Measured output V      R    raw ÷ 100 = V
//   0x0003  IOUT     Measured output A      R    raw ÷ 100 = A
//   0x0004  POWER    Measured output W      R    raw ÷ 10  = W
//   0x0005  UIN      Measured input V       R    raw ÷ 100 = V
//   0x0010  PROTECT  Protection status      R    0=OK 1=OVP 2=OCP 3=OPP...
//   0x0011  CVCC     CV / CC mode           R    0=CV 1=CC
//   0x0012  ONOFF    Output enable          R/W  0=off 1=on
//
// Supported function codes: 0x03 (read), 0x06 (write single), 0x10 (write multi)
// =============================================================================

#include <Arduino.h>
#include <ModbusMaster.h>
#include "config.h"

class ZkPsu {
public:
    // ── Live readings (updated by poll()) ─────────────────────────────────────
    float   measV     = 0.0f;   // measured output voltage (V)
    float   measA     = 0.0f;   // measured output current (A)
    float   measW     = 0.0f;   // measured output power   (W)
    float   inputV    = 0.0f;   // measured input voltage  (V)

    // ── Setpoint / state (synced with PSU on startup, updated locally on write)
    float   setV      = 0.0f;   // voltage setpoint (V)
    bool    outputOn  = false;  // true = output enabled
    bool    isCC      = false;  // true = constant-current mode
    uint8_t protCode  = 0;      // 0=normal, 1=OVP, 2=OCP, 3=OPP, 4=LVP...

    // ── Communication health ──────────────────────────────────────────────────
    bool    commsOk    = false;
    int     errStreak  = 0;
    int     goodStreak = 0;        // consecutive successes needed before clearing NO COMM
    unsigned long _lastRetryMs = 0;

    // ─────────────────────────────────────────────────────────────────────────
    ZkPsu() {}

    // Call once in setup(), after SPI/Wire init but before UI.
    void begin() {
        // Serial1 reserved for PSU; Serial2 reserved for CrowPanel link.
        Serial1.begin(PSU_BAUD, SERIAL_8N1, PSU_RX_PIN, PSU_TX_PIN);
        _node.begin(PSU_SLAVE_ID, Serial1);
        // No direction-pin callbacks needed — straight TTL UART, not RS-485.

        // Give the PSU serial port ~200ms to stabilise after UART init,
        // then read current state so local variables match the PSU.
        delay(200);
        _readMeasurements();
        _readStatus();
        _readSetpoint();
        _applyLimits();

        Serial.printf("[PSU] Init — comms=%s  V=%.2f  A=%.2f  on=%d\n",
                      commsOk ? "OK" : "FAIL", measV, measA, (int)outputOn);
    }

    // Call at ~5Hz from the main loop (PSU_POLL_MS = 200ms).
    // When comms are healthy, reads run at full rate.
    // When commsOk is false, reads back off to every 5s so Modbus timeout-waits
    // (~2s each × 3 reads = ~6s blocked) don't starve the HTTP server.
    void poll() {
        if (!commsOk) {
            if (millis() - _lastRetryMs < 5000UL) return;
            _lastRetryMs = millis();
        }
        _readMeasurements();
        _readStatus();
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    // Set output voltage setpoint.  Clamped to [PSU_V_MIN, PSU_V_MAX].
    // Raw value = volts × 100  (e.g. 24.00V → 2400).
    void setVoltage(float v) {
        v    = constrain(v, PSU_V_MIN, PSU_V_MAX);
        setV = v;
        _writeReg(REG_VSET, (uint16_t)roundf(v * 100.0f));
    }

    // Enable or disable the PSU output.
    // State is confirmed on the next poll() rather than assumed locally,
    // so the UI reflects the actual PSU state rather than a wishful write.
    void setOutput(bool on) {
        _writeReg(REG_ONOFF, on ? 1u : 0u);
    }

    // Human-readable status string for UI display.
    const char* statusStr() const {
        if (!commsOk)      return "NO COMM";
        if (protCode != 0) return _protName(protCode);
        if (!outputOn)     return "OFF";
        return isCC ? "CC" : "CV";
    }

private:
    ModbusMaster _node;

    // ── Register addresses ────────────────────────────────────────────────────
    static constexpr uint16_t REG_VSET    = 0x0000;
    static constexpr uint16_t REG_ISET    = 0x0001;
    static constexpr uint16_t REG_VOUT    = 0x0002;
    static constexpr uint16_t REG_IOUT    = 0x0003;
    static constexpr uint16_t REG_POWER   = 0x0004;
    static constexpr uint16_t REG_UIN     = 0x0005;
    static constexpr uint16_t REG_PROTECT = 0x0010;
    static constexpr uint16_t REG_CVCC    = 0x0011;
    static constexpr uint16_t REG_ONOFF   = 0x0012;
    static constexpr uint16_t REG_OVP     = 0x0053;  // S-OVP overvoltage protection
    static constexpr uint16_t REG_OCP     = 0x0054;  // S-OCP overcurrent protection
    static constexpr uint16_t REG_SINI    = 0x005D;  // S-INI power-on output switch (0=off, 1=on)

    // ── Read helpers ──────────────────────────────────────────────────────────

    // Batch-read VOUT (0x0002), IOUT (0x0003), POWER (0x0004) in one frame.
    void _readMeasurements() {
        uint8_t r = _node.readHoldingRegisters(REG_VOUT, 3);
        if (r == ModbusMaster::ku8MBSuccess) {
            measV = _node.getResponseBuffer(0) * 0.01f;
            measA = _node.getResponseBuffer(1) * 0.01f;
            measW = _node.getResponseBuffer(2) * 0.10f;
            errStreak = 0;
            goodStreak++;
            if (goodStreak >= 2) {   // require 2 consecutive successes to clear NO COMM
                commsOk = true;      // prevents single flaky response from bouncing state
            }
        } else {
            goodStreak = 0;
            _onError(r, "meas");
        }
    }

    // Batch-read PROTECT (0x0010), CVCC (0x0011), ONOFF (0x0012) — contiguous.
    // Then separately read UIN (0x0005) — not contiguous with the above.
    void _readStatus() {
        uint8_t r = _node.readHoldingRegisters(REG_PROTECT, 3);
        if (r == ModbusMaster::ku8MBSuccess) {
            protCode = (uint8_t)_node.getResponseBuffer(0);
            isCC     = (_node.getResponseBuffer(1) == 1);
            outputOn = (_node.getResponseBuffer(2) == 1);
        } else {
            _onError(r, "status");
        }

        r = _node.readHoldingRegisters(REG_UIN, 1);
        if (r == ModbusMaster::ku8MBSuccess) {
            inputV = _node.getResponseBuffer(0) * 0.01f;
        }
    }

    // Read V-SET once at startup to sync local setV with what the PSU has stored.
    void _readSetpoint() {
        uint8_t r = _node.readHoldingRegisters(REG_VSET, 1);
        if (r == ModbusMaster::ku8MBSuccess) {
            setV = _node.getResponseBuffer(0) * 0.01f;
        }
    }

    // ── Write helper ──────────────────────────────────────────────────────────
    void _writeReg(uint16_t reg, uint16_t val) {
        uint8_t r = _node.writeSingleRegister(reg, val);
        if (r != ModbusMaster::ku8MBSuccess) {
            DBG_PRINTF("[PSU] Write fail  reg=0x%04X val=%u  err=0x%02X\n",
                       reg, val, r);
        }
    }

    // ── Hardware limits — written only when current value differs ────────────
    // PSU registers are NVM-backed. Writing unconditionally on every boot wastes
    // write cycles. _writeIfDiff() reads first and skips the write if the value
    // is already correct — NVM is only touched when something actually changed.
    void _applyLimits() {
        if (!commsOk) return;
        _writeIfDiff(REG_ISET, (uint16_t)(PSU_I_MAX * 100.0f));  // 20.00A → 2000
        _writeIfDiff(REG_OVP,  (uint16_t)(PSU_V_OVP * 100.0f));  // 13.00V → 1300
        _writeIfDiff(REG_OCP,  (uint16_t)(PSU_I_OCP * 100.0f));  // 21.00A → 2100
        _writeIfDiff(REG_SINI, 0);                                 // boot output = OFF
        DBG_PRINTF("[PSU] Limits verified\n");
    }

    void _writeIfDiff(uint16_t reg, uint16_t desired) {
        uint8_t r = _node.readHoldingRegisters(reg, 1);
        if (r == ModbusMaster::ku8MBSuccess &&
            _node.getResponseBuffer(0) == desired) {
            return;  // already correct — no NVM write needed
        }
        _writeReg(reg, desired);
        DBG_PRINTF("[PSU] reg 0x%04X updated to %u\n", reg, desired);
    }

    // ── Error tracking ────────────────────────────────────────────────────────
    void _onError(uint8_t errCode, const char* ctx) {
        goodStreak = 0;
        errStreak++;
        if (errStreak == 3) {
            DBG_PRINTF("[PSU] Comms lost (%s) err=0x%02X  streak=%d\n",
                       ctx, errCode, errStreak);
            commsOk = false;   // threshold=3: ~6s to detect (3 poll cycles × ~2s timeout)
        }
    }

    // ── Protection code → name ────────────────────────────────────────────────
    static const char* _protName(uint8_t code) {
        switch (code) {
            case  1: return "OVP";
            case  2: return "OCP";
            case  3: return "OPP";
            case  4: return "LVP";
            case  5: return "OAH";
            case  6: return "OHP";
            case  7: return "OTP";
            case  8: return "OEP";
            case  9: return "OWH";
            case 10: return "ICP";
            case 11: return "IVP";
            default: return "FAULT";
        }
    }
};