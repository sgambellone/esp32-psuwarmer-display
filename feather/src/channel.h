#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "pid.h"
#include "feedforward.h"

// =============================================================================
// HeatingChannel — owns one PID, one FF learner, and one LEDC PWM output.
//
// Ported from GIGA R1 channel.h — zero control algorithm changes.
// Platform changes:
//   mbed::PwmOut  → ESP32 LEDC (ledcSetup / ledcWrite)
//   mbed KVStore  → ESP32 Preferences (NVS)
// =============================================================================

class HeatingChannel {
public:
    int   index;
    bool  enabled;
    float setpointF;
    float tempF;           // EMA-smoothed (alpha=0.4) — used by control loop
    float displayTempF;    // extra-smoothed (alpha=0.1) — used by UI only
    float duty;
    bool  sensorOk;
    bool  safetyTripped;
    int   failCount;
    int   _okStreak;

    PIDController      pid;
    FeedforwardLearner ff;
    float              dTdtF = 0.0f;
    float              _lastRawTempF = NAN;
    int                _rejectStreak = 0;

    // ledcChannel: the LEDC peripheral channel (0–7, must be unique per instance)
    HeatingChannel(int idx, uint8_t pwmPin, uint8_t ledcChannel, const char* ffKey)
        : index(idx),
          enabled(false),
          setpointF(SETPOINT_DEFAULT_F),
          tempF(NAN),
          displayTempF(NAN),
          duty(0),
          sensorOk(false),
          safetyTripped(false),
          failCount(0),
          _okStreak(0),
          ff(ffKey),
          _pwmPin(pwmPin),
          _ledcChannel(ledcChannel)
    {
        snprintf(_kvKeySP, sizeof(_kvKeySP), "sp_ch%d", idx);
    }

    void begin() {
        ledcSetup(_ledcChannel, PWM_FREQ_HZ, PWM_RESOLUTION);
        ledcAttachPin(_pwmPin, _ledcChannel);
        ledcWrite(_ledcChannel, 0);

        ff.load();
        _loadSetpoint();

        DBG_PRINTF("CH%d ready | GPIO%d | LEDC ch%d | setpoint %.0fF | FF@%.0fF: %.0f%%\n",
                   index + 1, _pwmPin, _ledcChannel, setpointF, setpointF,
                   ff.getSlot((int)setpointF - (int)SETPOINT_MIN_F) * 100);
    }

    void setDuty(float d) {
        float prev = duty;
        duty = constrain(d, 0.0f, 1.0f);
        ledcWrite(_ledcChannel, (uint32_t)(duty * PWM_MAX_DUTY));
        // Log heater on/off transitions
        if (prev < 0.01f && duty >= 0.01f)
            DBG_PRINTF("CH%d: heater ON  (%.0f%%)\n", index+1, duty*100.0f);
        else if (prev >= 0.01f && duty < 0.01f)
            DBG_PRINTF("CH%d: heater OFF\n", index+1);
    }

    void injectTemp(float tempCelsius) {
        if (!isnan(tempCelsius)) {
            float newTempF = tempCelsius * 9.0f / 5.0f + 32.0f;

            // Outlier rejection — 5°F threshold (see GIGA R1 comments)
            if (!isnan(_lastRawTempF) && fabsf(newTempF - _lastRawTempF) > 5.0f) {
                _rejectStreak++;
                DBG_PRINTF("CH%d: rejected outlier %.1fF -> %.1fF (delta=%.1fF, streak=%d)\n",
                           index+1, _lastRawTempF, newTempF, newTempF - _lastRawTempF, _rejectStreak);

                if (_rejectStreak >= 5) {
                    char msg[64];
                    snprintf(msg, sizeof(msg),
                             "CH%d: force re-seed after %d rejections: %.1fF\n",
                             index+1, _rejectStreak, newTempF);
                    Serial.print(msg);
                    tempF = newTempF;
                    displayTempF = newTempF;
                    _lastRawTempF = newTempF;
                    dTdtF = 0.0f;
                    _rejectStreak = 0;
                    sensorOk = true; failCount = 0;
                }
                return;
            }
            _rejectStreak = 0;
            _lastRawTempF = newTempF;

            if (!isnan(tempF)) {
                float instantRate = (newTempF - tempF) / (PID_INTERVAL_MS / 1000.0f);
                dTdtF = 0.1f * instantRate + 0.9f * dTdtF;
                tempF = 0.4f * newTempF + 0.6f * tempF;
            } else {
                tempF = newTempF;
            }

            if (!isnan(displayTempF)) {
                displayTempF = 0.1f * tempF + 0.9f * displayTempF;
            } else {
                displayTempF = tempF;
            }
            _okStreak++;
            failCount = 0;
            // Require 3 consecutive good reads before clearing SENS ERR —
            // prevents single-sample SPI glitches from causing flickering
            if (!sensorOk) {
                if (_okStreak >= 3) {
                    DBG_PRINTF("CH%d: sensor recovered after %d good reads\n", index+1, _okStreak);
                    sensorOk = true;
                    _okStreak = 0;
                }
            } else {
                sensorOk  = true;
                _okStreak = 0;
            }
        } else {
            failCount++;
            _okStreak = 0;
            if (failCount == SENSOR_FAIL_LIMIT) {
                DBG_PRINTF("CH%d: sensor error after %d failed reads\n",
                           index+1, SENSOR_FAIL_LIMIT);
            }
            if (failCount >= SENSOR_FAIL_LIMIT) {
                sensorOk    = false;
                // Clear cached temps so the UI shows --- immediately
                // rather than holding the last good reading indefinitely
                tempF       = NAN;
                displayTempF = NAN;
                _lastRawTempF = NAN;
                dTdtF       = 0.0f;
            }
        }
    }

    void updatePID(unsigned long nowMs) {
        if (!enabled || safetyTripped) {
            setDuty(0);
            if (!enabled) safetyTripped = false;   // reset latch on disable
            return;
        }
        if (!sensorOk || isnan(tempF)) { setDuty(0); return; }

        if (tempF >= SAFETY_CUTOFF_F) {
            safetyTripped = true;
            setDuty(0);
            DBG_PRINTF("CH%d SAFETY CUTOFF at %.1fF\n", index + 1, tempF);
            return;
        }

        float error = setpointF - tempF;

        if (error > 10.0f) {
            // Bang-bang zone — far from setpoint, always full power
            float risingRate    = max(0.0f, dTdtF);
            float predictedTemp = tempF + risingRate * THERMAL_LAG_SECONDS;
            if (predictedTemp >= setpointF) {
                setDuty(0);
                return;
            }
            setDuty(1.0f);

        } else {
            // FF-biased bidirectional PID
            // Only allow FF learning during steady HOLDING — not during HEATING.
            // HEATING state applies far-above-equilibrium duty which inflates the FF table.
            bool nearTarget = (fabsf(error) < 1.0f);
            float ffOut = ff.update(setpointF, tempF,
                                    nearTarget ? duty : -1.0f,  // -1 signals no-learn
                                    dTdtF);
            if (ffOut < FF_BOOTSTRAP_PCT) ffOut = FF_BOOTSTRAP_PCT;

            if (dTdtF > 0.0f && tempF < setpointF) {
                float predictedTemp = tempF + dTdtF * THERMAL_LAG_SECONDS;
                if (predictedTemp >= setpointF) {
                    pid.resetSoft(error);
                    setDuty(ffOut);
                    return;
                }
            }

            float pidOut = pid.compute(setpointF, tempF, nowMs);
            setDuty(constrain(ffOut + pidOut, 0.0f, PID_OUT_MAX));
        }
    }

    void adjustSetpoint(float deltaF) {
        float prev = setpointF;
        setpointF = constrain(setpointF + deltaF, SETPOINT_MIN_F, SETPOINT_MAX_F);
        pid.reset();
        _saveSetpoint();
        DBG_PRINTF("CH%d: setpoint %.0fF -> %.0fF\n", index+1, prev, setpointF);
    }

    float ffTerm() const {
        int slot = (int)roundf(setpointF) - (int)SETPOINT_MIN_F;
        if (slot < 0 || slot >= FF_SLOTS) return 0.0f;
        return ff.getSlot(slot);
    }

    const char* statusStr() const {
        if (!enabled)                   return "DISABLED";
        if (safetyTripped)              return "CUTOFF";
        if (!sensorOk || isnan(tempF))  return "SENS ERR";
        float error = setpointF - tempF;
        if (duty < 0.01f)               return "IDLE";
        if (error >  0.5f)              return "HEATING";
        if (error < -0.5f)              return "REDUCING";
        return "HOLDING";
    }

private:
    uint8_t  _pwmPin;
    uint8_t  _ledcChannel;
    char     _kvKeySP[12];    // "sp_ch0" or "sp_ch1"

    void _loadSetpoint() {
        Preferences prefs;
        prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
        float saved = prefs.getFloat(_kvKeySP, -1.0f);
        prefs.end();

        if (saved >= SETPOINT_MIN_F && saved <= SETPOINT_MAX_F) {
            setpointF = saved;
            DBG_PRINTF("CH%d setpoint loaded: %.0fF\n", index + 1, setpointF);
        } else {
            setpointF = SETPOINT_DEFAULT_F;
            DBG_PRINTF("CH%d setpoint default: %.0fF\n", index + 1, setpointF);
        }
    }

    void _saveSetpoint() {
        Preferences prefs;
        prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
        prefs.putFloat(_kvKeySP, setpointF);
        prefs.end();
    }
};