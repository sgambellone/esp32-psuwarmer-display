#pragma once
#include <Arduino.h>
#include "config.h"

// =============================================================================
// PIDController — discrete PID with anti-windup and integral freeze
//
// Ported from GIGA R1 pid.h — zero algorithm changes.
// mbed includes removed; all logic is platform-independent.
// =============================================================================

class PIDController {
public:
    float kp, ki, kd;
    float outMin, outMax;

    PIDController(float kp    = PID_KP_DEFAULT,
                  float ki    = PID_KI_DEFAULT,
                  float kd    = PID_KD_DEFAULT,
                  float oMin  = PID_OUT_MIN,
                  float oMax  = PID_OUT_MAX)
        : kp(kp), ki(ki), kd(kd),
          outMin(oMin), outMax(oMax),
          _integral(0), _lastError(0),
          _lastTime(0), _firstRun(true) {}

    float compute(float setpoint, float measured, unsigned long nowMs) {
        if (_firstRun) {
            _lastTime  = nowMs;
            _lastError = setpoint - measured;
            _firstRun  = false;
            return 0.0f;
        }

        float dt = (nowMs - _lastTime) / 1000.0f;
        if (dt <= 0) return 0.0f;
        dt = min(dt, 2.0f);   // cap at 2s — prevents windup after long idle
        _lastTime = nowMs;

        float error = setpoint - measured;

        // Freeze integral when far from setpoint (anti-windup)
        if (fabsf(error) < 10.0f) {
            _integral += error * dt;
            if (ki > 1e-6f) {
                float iLim = PID_IMAX / ki;
                _integral = constrain(_integral, -iLim, iLim);
            } else {
                _integral = 0;
            }
        }

        float pTerm     = kp * error;
        float iTerm     = ki * _integral;
        float derivative = (error - _lastError) / dt;
        float dTerm     = kd * derivative;
        _lastError = error;
        _lastP = pTerm; _lastI = iTerm; _lastD = dTerm;

        return constrain(pTerm + iTerm + dTerm, outMin, outMax);
    }

    void  reset()      { _integral = 0; _lastError = 0; _firstRun = true; _lastP = _lastI = _lastD = 0; }
    void  resetSoft(float currentError) { _integral = 0; _lastError = currentError; _firstRun = true; }
    void  refreshTime(unsigned long nowMs) { _lastTime = nowMs; _firstRun = false; }
    float integral() const { return _integral; }
    float lastP()    const { return _lastP; }
    float lastI()    const { return _lastI; }
    float lastD()    const { return _lastD; }

private:
    float         _integral;
    float         _lastError;
    float         _lastP = 0, _lastI = 0, _lastD = 0;
    unsigned long _lastTime;
    bool          _firstRun;
};
