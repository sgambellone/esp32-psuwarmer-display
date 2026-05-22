#pragma once
#include <Arduino.h>
#include <SD.h>
#include "config.h"

// =============================================================================
// WarmLogger — logs CSV to SD card, serves latest file over WiFi.
// SD card CS = SD_CS_PIN (pin 5 on 3651 V2 FeatherWing).
// File naming: LOG_0001.CSV, LOG_0002.CSV ... increments on each boot.
// =============================================================================

class WarmLogger {
public:
    WarmLogger() : _enabled(false), _rowsTotal(0), _fileNum(0), _bufPos(0), _lastFlushMs(0) { memset(_ramBuf, 0, sizeof(_ramBuf)); }

    bool begin() {
        if (!SD.begin(SD_CS_PIN)) {
            Serial.println("LOGGER: SD card not found");
            return false;
        }
        Serial.printf("LOGGER: SD card ready (%lluMB)\n",
                      SD.cardSize() / (1024 * 1024));

        // Find next available log file number
        _fileNum = 1;
        while (_fileNum < 9999) {
            char path[20];
            snprintf(path, sizeof(path), "/LOG_%04d.CSV", _fileNum);
            if (!SD.exists(path)) break;
            _fileNum++;
        }
        snprintf(_filePath, sizeof(_filePath), "/LOG_%04d.CSV", _fileNum);

        // Write CSV header
        File f = SD.open(_filePath, FILE_WRITE);
        if (!f) {
            Serial.printf("LOGGER: failed to create %s\n", _filePath);
            return false;
        }
        f.print("t_ms,"
                "ch1_tempF,ch1_setptF,ch1_errorF,ch1_duty_pct,ch1_ff_pct,ch1_dtdt,ch1_status,"
                "ch2_tempF,ch2_setptF,ch2_errorF,ch2_duty_pct,ch2_ff_pct,ch2_dtdt,ch2_status,"
                "psu_vout,psu_aout,psu_watts,psu_vin,psu_setv,psu_on,psu_cc,"
                "env_tempF,env_humidity,env_pressHpa,env_gasKohm\n");
        f.close();

        _enabled = true;
        Serial.printf("[logger] Logging to %s\n", _filePath);
        Serial.printf("[logger] SD free: ~%lluMB\n",
                      (SD.totalBytes() - SD.usedBytes()) / (1024*1024));
        return true;
    }

    void log(unsigned long nowMs,
             float ch1TempF, float ch1SetptF, float ch1Duty,
             float ch1FF,    float ch1DTdt,   const char* ch1Status,
             float ch2TempF, float ch2SetptF, float ch2Duty,
             float ch2FF,    float ch2DTdt,   const char* ch2Status,
             // PSU
             float psuVout,  float psuAout,   float psuWatts,
             float psuVin,   float psuSetV,   bool psuOn, bool psuCC,
             // BME680 environmental
             bool  envValid,
             float envTempF, float envHumidity,
             float envPressHpa, float envGasKohm)
    {
        if (!_enabled) return;

        char row[320];
        snprintf(row, sizeof(row),
            "%lu,%.2f,%.1f,%.2f,%.1f,%.1f,%.4f,%s,"
               "%.2f,%.1f,%.2f,%.1f,%.1f,%.4f,%s,"
               "%.2f,%.2f,%.1f,%.2f,%.2f,%d,%d,"
               "%.1f,%.1f,%.1f,%.1f\n",
            nowMs,
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

        // Append to RAM buffer — flush to SD periodically, not every row.
        // SD.open/close takes 5-20ms and blocks the loop causing UI jank.
        int len = strlen(row);
        if (_bufPos + len < (int)sizeof(_ramBuf)) {
            memcpy(_ramBuf + _bufPos, row, len);
            _bufPos += len;
            _rowsTotal++;
        }

        // Flush to SD every 10 rows or 5 seconds, whichever comes first
        unsigned long nowMs2 = millis();
        if (_bufPos > 0 && (_rowsTotal % 10 == 0 || nowMs2 - _lastFlushMs > 5000)) {
            _flushToSD();
        }

        if (_rowsTotal % 100 == 0) {
            Serial.printf("[logger] %d rows written to %s\n", _rowsTotal, _filePath);
        }
    }

    // Returns true during and for 100ms after an SD flush.
    // Used by feather_main.cpp to pause MAX31865 reads during the window
    // when the XTSD flash's internal NAND program cycle can disturb the
    // 3.3V rail or the shared SPI bus.
    bool isBusy() const { return millis() < _flushBusyUntilMs; }

    // Serve the current log file over WiFi — streams directly from SD
    void serveFile(WiFiClient& client) {
        if (!_enabled) return;

        // Flush any buffered rows to SD first — ensures Content-Length is
        // accurate and the file contains all rows up to this moment.
        if (_bufPos > 0) _flushToSD();

        File f = SD.open(_filePath, FILE_READ);
        if (!f) {
            client.printf("HTTP/1.1 500 Internal Server Error\r\n"
                          "Content-Type: text/plain\r\nConnection: close\r\n\r\n"
                          "Log file open failed");
            return;
        }

        uint32_t fileSize = f.size();
        client.printf("HTTP/1.1 200 OK\r\n"
                      "Content-Type: text/csv\r\n"
                      "Content-Length: %u\r\n"
                      "Content-Disposition: attachment; filename=\"%s\"\r\n"
                      "Connection: close\r\n\r\n",
                      fileSize, _filePath + 1);   // skip leading /
        client.flush();  // push headers to browser immediately

        uint8_t buf[256];
        bool ok = true;
        while (ok && f.available()) {
            int n = f.read(buf, sizeof(buf));
            if (n <= 0) break;
            // Retry partial writes — WiFiClient::write() may not accept all
            // bytes in one call if the TCP send buffer is temporarily full.
            size_t sent = 0;
            while (sent < (size_t)n) {
                size_t w = client.write(buf + sent, n - sent);
                if (w == 0) { ok = false; break; }
                sent += w;
            }
            client.flush();  // push chunk through lwIP immediately — without
                             // this, Nagle algorithm holds data in the buffer
                             // and the browser sees 0 bytes until disconnect.
            yield();         // give WiFi/TCP task time to transmit
        }
        f.close();
        Serial.printf("LOGGER: served %s (%u bytes) via WiFi\n",
                      _filePath, fileSize);
    }

    // Force flush — call on clean shutdown
    void flush() { if (_bufPos > 0) _flushToSD(); }

    bool        enabled()    const { return _enabled; }
    int         rowsTotal()  const { return _rowsTotal; }
    int         bufferUsed() const {
        if (!_enabled) return 0;
        File f = SD.open(_filePath, FILE_READ);
        if (!f) return 0;
        int sz = f.size();
        f.close();
        return sz;
    }
    const char* filepath()   const { return _filePath; }
    const char* buffer()     const { return nullptr; }

private:
    static constexpr uint32_t MAX_LOG_BYTES = 50UL * 1024 * 1024; // 50MB per file (~17hrs)
    static constexpr uint32_t MIN_FREE_MB   = 100;                 // delete oldest when below
    bool  _enabled;
    int   _rowsTotal;
    int   _fileNum;
    char  _filePath[24];
    char  _ramBuf[4096];
    int   _bufPos        = 0;
    unsigned long _lastFlushMs      = 0;
    unsigned long _flushBusyUntilMs = 0;  // millis() deadline for isBusy()
    uint32_t      _bytesWritten     = 0;  // bytes written to current file

    void _flushToSD() {
        File f = SD.open(_filePath, FILE_APPEND);
        if (f) {
            _bytesWritten += (uint32_t)_bufPos;
            f.write((const uint8_t*)_ramBuf, _bufPos);
            f.close();
        }
        _bufPos = 0;
        _lastFlushMs = millis();
        // Hold busy flag for 100ms after the SPI transaction ends.
        // The XTSD NAND flash programs internally after the SPI write
        // completes, drawing a current spike that can disturb the 3.3V
        // rail and corrupt a simultaneous MAX31865 SPI read.
        _flushBusyUntilMs = _lastFlushMs + 100;

        // Rotate if current file has reached the size limit
        if (_bytesWritten >= MAX_LOG_BYTES) {
            Serial.printf("[logger] %s full (%luMB), rotating\n",
                          _filePath, (unsigned long)(_bytesWritten / (1024*1024)));
            _openNextFile();
        }
    }

    // ── Open next numbered log file, deleting oldest if card is nearly full ──
    void _openNextFile() {
        _fileNum++;
        snprintf(_filePath, sizeof(_filePath), "/log_%03d.csv", _fileNum);

        // If free space is below threshold, delete the oldest file that exists
        uint64_t freeMB = (SD.totalBytes() - SD.usedBytes()) / (1024*1024);
        if (freeMB < MIN_FREE_MB) {
            for (int del = 1; del < _fileNum; del++) {
                char delPath[32];
                snprintf(delPath, sizeof(delPath), "/log_%03d.csv", del);
                if (SD.exists(delPath)) {
                    SD.remove(delPath);
                    Serial.printf("[logger] Deleted %s (free was %lluMB)\n",
                                  delPath, freeMB);
                    break;  // delete one at a time — rotation will handle the rest
                }
            }
        }

        // Write CSV header to new file
        File f = SD.open(_filePath, FILE_WRITE);
        if (f) {
            f.println("t_ms,c1_tempF,c1_setptF,c1_error,c1_duty,c1_ff,"
                      "c1_dtdt,c1_status,c2_tempF,c2_setptF,c2_error,"
                      "c2_duty,c2_ff,c2_dtdt,c2_status,psu_vout,psu_aout,"
                      "psu_watts,psu_vin,psu_setv,psu_on,psu_cc,"
                      "env_tempF,env_humidity,env_pressHpa,env_gasKohm");
            f.close();
        }
        _bytesWritten = 0;
        _rowsTotal    = 0;
        Serial.printf("[logger] Opened new file: %s (free: ~%lluMB)\n",
                      _filePath,
                      (SD.totalBytes() - SD.usedBytes()) / (1024*1024));
    }
};