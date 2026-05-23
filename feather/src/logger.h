#pragma once
#include <Arduino.h>
#include <time.h>
#include <SD.h>
#include "config.h"

// =============================================================================
// WarmLogger — CSV logger with date-based filenames and file browser support.
//
// File naming:
//   LOG_YYYYMMDD_HHMMSS.CSV  — when epoch known (e.g. LOG_20260522_173045.CSV)
//   LOG_NORTC_NNNN.CSV       — fallback when RTC not set
//
// Design:
//   File is created LAZILY on the first log() call, not in begin().
//   This ensures the epoch from CrowPanel (arrives ~200ms after boot, first
//   log at 500ms) is available for the filename before the file is created.
//
//   setEpoch() is called by main loop whenever epoch is available.
//   It is also used for rotation naming when a file fills up.
//
// SD card CS = SD_CS_PIN (GPIO14).
// =============================================================================

static const char* CSV_HEADER =
    "t_ms,iso_time,"
    "ch1_tempF,ch1_setptF,ch1_errorF,ch1_duty_pct,ch1_ff_pct,ch1_dtdt,ch1_status,"
    "ch2_tempF,ch2_setptF,ch2_errorF,ch2_duty_pct,ch2_ff_pct,ch2_dtdt,ch2_status,"
    "psu_vout,psu_aout,psu_watts,psu_vin,psu_setv,psu_on,psu_cc,"
    "env_tempF,env_humidity,env_pressHpa,env_gasKohm\n";

class WarmLogger {
public:
    WarmLogger() : _sdReady(false), _fileOpen(false), _enabled(false),
                   _rowsTotal(0), _nortcSeq(0), _bufPos(0),
                   _lastFlushMs(0), _flushBusyUntilMs(0), _bytesWritten(0),
                   _firstLogMs(0), _epochRef(0), _epochMs(0) {
        _filePath[0] = '\0';
        memset(_ramBuf, 0, sizeof(_ramBuf));
    }

    // ── begin() — validate SD card only; does not create any file ─────────────
    bool begin() {
        if (!SD.begin(SD_CS_PIN)) {
            Serial.println("[logger] SD card not found");
            return false;
        }
        _sdReady = true;
        _enabled = true;
        Serial.printf("[logger] SD ready (%lluMB free of %lluMB)\n",
                      (SD.totalBytes() - SD.usedBytes()) / (1024*1024),
                      SD.totalBytes() / (1024*1024));
        return true;
    }

    // ── setEpoch() — update epoch reference; triggers file switch if needed ────
    void setEpoch(uint32_t epoch, unsigned long epochMs) {
        if (epoch == _epochRef) return;  // no change
        _epochRef = epoch;
        _epochMs  = epochMs;

        // If we're already writing a NORTC file and now have a valid epoch,
        // flush and switch to a dated file so the rest of the session is named.
        // The NORTC file keeps whatever was logged before the epoch arrived.
        if (_fileOpen && _epochRef > 0 && strstr(_filePath, "NORTC")) {
            Serial.printf("[logger] epoch arrived — switching from %s to dated file\n",
                          _filePath + 1);
            flush();
            _fileOpen = false;  // next log() call opens dated file
        }
    }

    // ── log() — write a row; creates file on first call ───────────────────────
    void log(unsigned long nowMs, uint32_t rowEpoch,
             float ch1TempF, float ch1SetptF, float ch1Duty,
             float ch1FF,    float ch1DTdt,   const char* ch1Status,
             float ch2TempF, float ch2SetptF, float ch2Duty,
             float ch2FF,    float ch2DTdt,   const char* ch2Status,
             float psuVout,  float psuAout,   float psuWatts,
             float psuVin,   float psuSetV,   bool psuOn, bool psuCC,
             bool  envValid,
             float envTempF, float envHumidity,
             float envPressHpa, float envGasKohm)
    {
        if (!_enabled) return;

        // Lazily create the file on first call.
        // Wait up to 5 seconds FROM THE FIRST log() CALL (not from boot —
        // the Feather's WiFi init can take 3-5s, so millis()>5000 at first call
        // is common and would skip the wait entirely with an absolute check).
        // After 5s from first log(), fall back to LOG_NORTC rather than block.
        if (!_fileOpen) {
            if (_firstLogMs == 0) _firstLogMs = millis();
            if (_epochRef > 0 || millis() - _firstLogMs > 5000UL) {
                _openNewFile();
                if (!_fileOpen) return;  // SD error — give up
            }
            // Epoch not yet available — buffer row; _flushToSD() is a no-op
            // while !_fileOpen so nothing is lost.
        }

        // ISO 8601 timestamp from row epoch
        char iso[24] = "----";
        if (rowEpoch > 0) {
            time_t t = (time_t)rowEpoch;
            struct tm* tm = gmtime(&t);
            if (tm) strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", tm);
        }

        char row[320];
        snprintf(row, sizeof(row),
            "%lu,%s,%.2f,%.1f,%.2f,%.1f,%.1f,%.4f,%s,"
               "%.2f,%.1f,%.2f,%.1f,%.1f,%.4f,%s,"
               "%.2f,%.2f,%.1f,%.2f,%.2f,%d,%d,"
               "%.1f,%.1f,%.1f,%.1f\n",
            nowMs, iso,
            ch1TempF, ch1SetptF,
            (ch1TempF > -998.0f) ? ch1SetptF - ch1TempF : 0.0f,
            ch1Duty*100.0f, ch1FF*100.0f, ch1DTdt, ch1Status,
            ch2TempF, ch2SetptF,
            (ch2TempF > -998.0f) ? ch2SetptF - ch2TempF : 0.0f,
            ch2Duty*100.0f, ch2FF*100.0f, ch2DTdt, ch2Status,
            psuVout, psuAout, psuWatts, psuVin, psuSetV,
            psuOn ? 1 : 0, psuCC ? 1 : 0,
            envValid ? envTempF    : 0.0f,
            envValid ? envHumidity : 0.0f,
            envValid ? envPressHpa : 0.0f,
            envValid ? envGasKohm  : 0.0f);

        int len = strlen(row);
        if (_bufPos + len < (int)sizeof(_ramBuf)) {
            memcpy(_ramBuf + _bufPos, row, len);
            _bufPos += len;
            _rowsTotal++;
        }

        // Flush every 10 rows or 5 seconds
        unsigned long now2 = millis();
        if (_bufPos > 0 && (_rowsTotal % 10 == 0 || now2 - _lastFlushMs > 5000))
            _flushToSD();

        if (_rowsTotal % 100 == 0)
            Serial.printf("[logger] %d rows → %s\n", _rowsTotal, _filePath + 1);
    }

    // ── isBusy() — 100ms guard after SD flush for MAX31865 protection ─────────
    bool isBusy() const { return millis() < _flushBusyUntilMs; }

    // ── flush() — force write buffer to SD ────────────────────────────────────
    void flush() { if (_bufPos > 0 && _fileOpen) _flushToSD(); }

    // ── resetLogs() — delete all LOG_*.CSV files and start fresh ──────────────
    int resetLogs() {
        flush();
        _fileOpen     = false;
        _bytesWritten = 0;
        _rowsTotal    = 0;
        _bufPos       = 0;
        _filePath[0]  = '\0';

        // Scan root directory and delete all LOG_*.CSV files
        File root = SD.open("/");
        int deleted = 0;
        File entry = root.openNextFile();
        while (entry) {
            const char* n = entry.name();
            entry.close();
            if (strncmp(n, "LOG_", 4) == 0) {
                int nl = strlen(n);
                if (nl >= 4 && strcmp(n + nl - 4, ".CSV") == 0) {
                    char p[36]; snprintf(p, sizeof(p), "/%s", n);
                    SD.remove(p);
                    deleted++;
                }
            }
            entry = root.openNextFile();
        }
        root.close();
        Serial.printf("[logger] resetLogs: deleted %d files\n", deleted);
        // Next log() call will open a fresh file
        return deleted;
    }

    // ── serveFile() — stream a named file (or current) to WiFiClient ──────────
    // filename: just the name e.g. "LOG_20260522_173045.CSV" (no leading /)
    // Pass nullptr to serve the current log file.
    void serveFile(WiFiClient& client, const char* filename = nullptr) {
        if (!_sdReady) {
            client.print("HTTP/1.1 503 Service Unavailable\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nSD not ready");
            return;
        }
        flush();  // ensure all rows are on disk before serving

        char path[36];
        if (filename && filename[0]) {
            snprintf(path, sizeof(path), "/%s", filename);
        } else if (_fileOpen && _filePath[0]) {
            strncpy(path, _filePath, sizeof(path) - 1);
        } else {
            client.print("HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nNo log file");
            return;
        }

        File f = SD.open(path, FILE_READ);
        if (!f) {
            client.printf("HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nFile not found: %s", path);
            return;
        }
        uint32_t sz = f.size();
        const char* dispName = path + 1;  // skip leading /
        client.printf("HTTP/1.1 200 OK\r\nContent-Type: text/csv\r\nContent-Length: %u\r\n"
                      "Content-Disposition: attachment; filename=\"%s\"\r\nConnection: close\r\n\r\n",
                      sz, dispName);
        client.flush();
        uint8_t buf[256]; bool ok = true;
        while (ok && f.available()) {
            int n = f.read(buf, sizeof(buf)); if (n <= 0) break;
            size_t sent = 0;
            while (sent < (size_t)n) {
                size_t w = client.write(buf + sent, n - sent);
                if (!w) { ok = false; break; }
                sent += w;
            }
            client.flush(); yield();
        }
        f.close();
        Serial.printf("[logger] served %s (%lu bytes)\n", path, (unsigned long)sz);
    }

    // ── listFiles() — fill caller's array with sorted LOG filenames ───────────
    // Returns count of files found (max = capacity).
    // Each entry is a 32-char buffer: just the filename, no leading /.
    int listFiles(char (*out)[32], int capacity) {
        int count = 0;
        File root = SD.open("/");
        File entry = root.openNextFile();
        while (entry && count < capacity) {
            const char* n = entry.name();
            int nl = strlen(n);
            entry.close();
            if (strncmp(n, "LOG_", 4) == 0 && nl >= 4 &&
                strcmp(n + nl - 4, ".CSV") == 0) {
                strncpy(out[count], n, 31);
                out[count][31] = '\0';
                count++;
            }
            entry = root.openNextFile();
        }
        root.close();
        // Simple insertion sort by name — LOG_YYYYMMDD sorts chronologically
        for (int i = 1; i < count; i++) {
            char tmp[32]; strncpy(tmp, out[i], 32);
            int j = i - 1;
            while (j >= 0 && strcmp(out[j], tmp) > 0) {
                strncpy(out[j+1], out[j], 32); j--;
            }
            strncpy(out[j+1], tmp, 32);
        }
        return count;
    }

    // ── Accessors ──────────────────────────────────────────────────────────────
    bool        enabled()    const { return _enabled; }
    bool        fileOpen()   const { return _fileOpen; }
    int         rowsTotal()  const { return _rowsTotal; }
    const char* filepath()   const { return _filePath; }
    int         bufferUsed() const {
        if (!_fileOpen || !_filePath[0]) return 0;
        File f = SD.open(_filePath, FILE_READ);
        if (!f) return 0;
        int sz = f.size(); f.close(); return sz;
    }

private:
    static constexpr uint32_t MAX_LOG_BYTES = 50UL * 1024 * 1024;  // 50MB per file
    static constexpr uint32_t MIN_FREE_MB   = 100;

    bool  _sdReady, _fileOpen, _enabled;
    int   _rowsTotal, _nortcSeq, _bufPos;
    char  _filePath[32];
    char  _ramBuf[4096];
    unsigned long _lastFlushMs, _flushBusyUntilMs;
    unsigned long _firstLogMs;        // millis() at first log() — epoch wait is relative to this
    uint32_t      _bytesWritten;
    uint32_t      _epochRef;      // last known epoch
    unsigned long _epochMs;       // millis() when _epochRef was set

    // Current computed epoch (for rotation naming)
    uint32_t _nowEpoch() const {
        if (_epochRef == 0) return 0;
        return _epochRef + (uint32_t)((millis() - _epochMs) / 1000);
    }

    // Build filename from epoch into buf (32 bytes)
    void _makeFilename(char* buf, size_t len, uint32_t epoch) {
        if (epoch > 0) {
            time_t t = (time_t)epoch;
            struct tm* tm = gmtime(&t);
            if (tm) {
                snprintf(buf, len, "/LOG_%04d%02d%02d_%02d%02d%02d.CSV",
                         1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday,
                         tm->tm_hour, tm->tm_min, tm->tm_sec);
                return;
            }
        }
        // Fallback: find highest existing NORTC sequence number and increment
        if (_nortcSeq == 0) {
            File root = SD.open("/");
            File entry = root.openNextFile();
            while (entry) {
                const char* n = entry.name();
                entry.close();
                if (strncmp(n, "LOG_NORTC_", 10) == 0) {
                    int seq = atoi(n + 10);  // parse number after "LOG_NORTC_"
                    if (seq > _nortcSeq) _nortcSeq = seq;
                }
                entry = root.openNextFile();
            }
            root.close();
        }
        _nortcSeq++;
        snprintf(buf, len, "/LOG_NORTC_%04d.CSV", _nortcSeq);
    }

    // Create a new file with CSV header
    void _openNewFile() {
        _makeFilename(_filePath, sizeof(_filePath), _nowEpoch());
        _deleteOldestIfNeeded();
        File f = SD.open(_filePath, FILE_WRITE);
        if (!f) {
            Serial.printf("[logger] Failed to create %s\n", _filePath);
            return;
        }
        f.print(CSV_HEADER);
        f.close();
        _bytesWritten = 0;
        _fileOpen     = true;
        Serial.printf("[logger] Opened %s (free: ~%lluMB)\n",
                      _filePath + 1,
                      (SD.totalBytes() - SD.usedBytes()) / (1024*1024));
    }

    // Flush RAM buffer to SD
    void _flushToSD() {
        if (!_fileOpen) return;
        File f = SD.open(_filePath, FILE_APPEND);
        if (f) {
            _bytesWritten += (uint32_t)_bufPos;
            f.write((const uint8_t*)_ramBuf, _bufPos);
            f.close();
        }
        _bufPos       = 0;
        _lastFlushMs  = millis();
        _flushBusyUntilMs = _lastFlushMs + 100;  // MAX31865 guard

        // Rotate if file is full
        if (_bytesWritten >= MAX_LOG_BYTES) {
            Serial.printf("[logger] %s full (%.0fMB), rotating\n",
                          _filePath + 1, _bytesWritten / 1048576.0f);
            _fileOpen = false;  // next log() call opens new file with fresh timestamp
        }
    }

    // Delete oldest LOG file if free space below threshold
    void _deleteOldestIfNeeded() {
        uint64_t freeMB = (SD.totalBytes() - SD.usedBytes()) / (1024*1024);
        if (freeMB >= MIN_FREE_MB) return;

        // Find oldest LOG_*.CSV in root by name (alphabetical = chronological)
        char oldest[32] = {};
        File root = SD.open("/");
        File entry = root.openNextFile();
        while (entry) {
            const char* n = entry.name();
            int nl = strlen(n);
            entry.close();
            if (strncmp(n, "LOG_", 4) == 0 && nl >= 4 &&
                strcmp(n + nl - 4, ".CSV") == 0) {
                if (!oldest[0] || strcmp(n, oldest) < 0)
                    strncpy(oldest, n, sizeof(oldest) - 1);
            }
            entry = root.openNextFile();
        }
        root.close();

        if (oldest[0]) {
            char p[36]; snprintf(p, sizeof(p), "/%s", oldest);
            // Don't delete the file we're currently writing to
            if (strcmp(p, _filePath) != 0) {
                SD.remove(p);
                Serial.printf("[logger] Deleted %s (free was %lluMB)\n", oldest, freeMB);
            }
        }
    }
};