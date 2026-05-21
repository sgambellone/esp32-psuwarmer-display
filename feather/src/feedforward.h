#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// =============================================================================
// FeedforwardLearner — EMA duty lookup table, one slot per °F
//
// Ported from GIGA R1 feedforward.h — zero algorithm changes.
// KVStore (mbed) replaced with ESP32 Preferences (NVS).
//
// The table learns the steady-state duty cycle required to hold each
// setpoint temperature and feeds it forward to reduce PID overshoot.
// =============================================================================

class FeedforwardLearner {
public:
    // kvKey is a short string used as the NVS key, e.g. "ff_ch0"
    explicit FeedforwardLearner(const char* kvKey)
        : _lastSaveMs(0), _stableCount(0) {
        strncpy(_kvKey, kvKey, sizeof(_kvKey) - 1);
        _kvKey[sizeof(_kvKey) - 1] = '\0';
        memset(_table, 0, sizeof(_table));
    }

    void load() {
        Preferences prefs;
        prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
        size_t stored = prefs.getBytesLength(_kvKey);
        if (stored == sizeof(_table)) {
            prefs.getBytes(_kvKey, _table, sizeof(_table));
            int learned = 0;
            for (int i = 0; i < FF_SLOTS; i++)
                if (_table[i] > 0.001f) learned++;
            DBG_PRINTF("FF [%s]: loaded %d/%d slots\n", _kvKey, learned, FF_SLOTS);
        } else {
            DBG_PRINTF("FF [%s]: no saved data, starting fresh\n", _kvKey);
            memset(_table, 0, sizeof(_table));
        }
        prefs.end();
        _lastSaveMs = millis();
    }

    void save() {
        Preferences prefs;
        prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
        prefs.putBytes(_kvKey, _table, sizeof(_table));
        prefs.end();
        DBG_PRINTF("FF [%s]: saved to NVS\n", _kvKey);
        _lastSaveMs = millis();
    }

    // Call every PID cycle; returns the FF duty term to add to PID output.
    // dTdtF = smoothed temperature rate of change in °F/sec (from channel)
    // Pass currentDuty=-1.0f to suppress learning while still returning the current table value
    float update(float setpointF, float measuredF, float currentDuty, float dTdtF) {
        int slot = _slot(setpointF);
        if (slot < 0) return 0;
        if (currentDuty < 0.0f) return _table[slot];  // no-learn sentinel

        // FF training gates — bidirectional around setpoint (see GIGA R1 comments)
        bool nearSetpoint  = ((setpointF - measuredF) <= FF_STEADY_BAND_F) &&
                             (measuredF < setpointF + 0.5f);
        bool notSaturated  = (currentDuty > 0.02f) &&
                             (currentDuty < PID_OUT_MAX - 0.05f);
        bool atEquilibrium = fabsf(dTdtF) < 0.05f;  // tightened: 0.15→0.05 prevents learning during heating events

        if (nearSetpoint && notSaturated && atEquilibrium) {
            _stableCount++;
        } else if (notSaturated) {
            _stableCount = 0;
        }
        // duty=0 (brief IDLE) — preserve count

        if (_stableCount >= 2) {
            float old = _table[slot];
            float newVal = old + FF_LEARN_RATE * (currentDuty - old);
            if (fabsf(newVal - old) > 0.001f) {
                DBG_PRINTF("FF [%s] slot%d (%.0fF): %.1f%% -> %.1f%%\n",
                           _kvKey, slot, setpointF, old*100.0f, newVal*100.0f);
            }
            _table[slot] = newVal;

            if (millis() - _lastSaveMs >= FF_SAVE_INTERVAL_MS) {
                save();
            }
        }

        return _table[slot];
    }

    float getSlot(int i)  const { return (i >= 0 && i < FF_SLOTS) ? _table[i] : 0; }
    void  clear()               { memset(_table, 0, sizeof(_table)); save(); }

private:
    char          _kvKey[16];
    float         _table[FF_SLOTS];
    unsigned long _lastSaveMs;
    int           _stableCount;

    int _slot(float setpointF) const {
        int idx = (int)roundf(setpointF) - (int)SETPOINT_MIN_F;
        return (idx >= 0 && idx < FF_SLOTS) ? idx : -1;
    }
};