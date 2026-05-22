#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "logger.h"
#include "channel.h"
#include "psu.h"

// =============================================================================
// WarmWiFiServer — redesigned to match CrowPanel LVGL UI palette & layout.
//
// Changes from previous version:
//   • Full color palette match: black bg, orange CH accent, CrowPanel exact colors
//   • 4-tab SPA: Dashboard | PSU | Warmer | Env  (mirrors CrowPanel pages)
//   • Dashboard: PSU strip (V/A/W/Vin/Set/ON) + two compact channel cards
//   • PSU page: 4 big readout tiles + preset buttons + adjust row + ON/OFF btn
//   • Warmer page: two large channel cards (full height)
//   • Env page: 4 BME680 sensor cards (Temp / Humidity / Pressure / Air Quality)
//   • EnvData struct + setEnv() — call from main.cpp after each BME680 read
//   • /api now includes "env" block
//
// main.cpp one-line addition:
//   After bme.endReading() succeeds, add:
//     wifiSrv.setEnv(_bmeTempF, _bmeHumidity, _bmePressHpa, _bmeGasKohm, _bmeSeeded);
//
// Endpoints (unchanged):
//   GET /dash      → SPA (4 tabs)
//   GET /api       → JSON telemetry
//   GET /control   → command endpoint
//   GET /log       → CSV download
//   GET /fftable   → FF table
//   GET /diag      → diagnostics
//   GET /status    → plain-text
// =============================================================================

struct EnvData {
    float tempF    = 0.f;
    float humidity = 0.f;
    float pressHpa = 0.f;
    float gasKohm  = 0.f;
    bool  valid    = false;
};

class WarmWiFiServer {
public:
    WarmWiFiServer()
        : _server(WIFI_HTTP_PORT), _ready(false),
          _logger(nullptr), _psu(nullptr),
          _ch1(nullptr), _ch2(nullptr),
          _reqCount(0), _lastHeartbeatMs(0) {}

    void begin() {
        _server.begin();
        _ready = true;
        _reqCount = 0;
        Serial.printf("[http] Server listening — http://%s/dash\n",
                      WiFi.softAPIP().toString().c_str());
    }

    void setLogger(WarmLogger* l) { _logger = l; }
    void setPsu(ZkPsu* p)         { _psu = p; }
    bool ready() const            { return _ready; }

    // Called from main.cpp after each successful BME680 endReading().
    void setEnv(float t, float h, float p, float g, bool valid) {
        _env.tempF    = t;
        _env.humidity = h;
        _env.pressHpa = p;
        _env.gasKohm  = g;
        _env.valid    = valid;
    }

    void poll(HeatingChannel* ch1, HeatingChannel* ch2) {
        _ch1 = ch1; _ch2 = ch2;
        if (!_ready) return;

        if (millis() - _lastHeartbeatMs >= 300000UL) {
            Serial.printf("[wifi] Heartbeat | uptime=%lus | reqs=%lu | clients=%d\n",
                          millis()/1000, _reqCount, WiFi.softAPgetStationNum());
            _lastHeartbeatMs = millis();
        }

        WiFiClient client = _server.accept();
        if (!client) return;
        _reqCount++;

        unsigned long t0 = millis();
        while (!client.available() && client.connected() && millis()-t0 < 150) delay(1);
        if (!client.available()) { _serveRedirect(client, "/dash"); client.stop(); return; }

        char req[128]; int reqLen = 0;
        unsigned long t = millis();
        while (client.connected() && millis()-t < 80 && reqLen < 127) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n') break;
                if (c != '\r') req[reqLen++] = c;
                t = millis();
            }
        }
        req[reqLen] = '\0';
        while (client.available()) client.read();

        const char* path = strchr(req, ' ');
        if (!path) { _serveRedirect(client, "/dash"); client.stop(); return; }
        path++;

        unsigned long t0r = millis();
        if      (strncmp(path, "/api",     4) == 0) _serveApi(client);
        else if (strncmp(path, "/fftable", 8) == 0) _serveFFTable(client);
        else if (strncmp(path, "/diag",    5) == 0) _serveDiag(client);
        else if (strncmp(path, "/dash",    5) == 0) _serveDash(client);
        else if (strncmp(path, "/control", 8) == 0) _serveControl(client, req);
        else if (strncmp(path, "/log",     4) == 0) _serveLog(client);
        else if (strncmp(path, "/status",  7) == 0) _serveStatus(client);
        else                                         _serveRedirect(client, "/dash");

        if (strncmp(path, "/api", 4) != 0) {
            char cleanPath[48];
            strncpy(cleanPath, path, 47); cleanPath[47] = '\0';
            char* sp = strchr(cleanPath, ' ');
            if (sp) *sp = '\0';
            Serial.printf("[http] %s  (%lums)\n", cleanPath, millis() - t0r);
        }
        client.flush();
        client.stop();
    }

private:
    WiFiServer      _server;
    bool            _ready;
    WarmLogger*     _logger;
    ZkPsu*          _psu;
    HeatingChannel* _ch1;
    HeatingChannel* _ch2;
    EnvData         _env;
    unsigned long   _reqCount;
    unsigned long   _lastHeartbeatMs;

    // ── Helpers ───────────────────────────────────────────────────────────────
    void _sendHeaders(WiFiClient& c, int code, const char* mime, long len = -1) {
        c.printf("HTTP/1.1 %d %s\r\n", code, code==200?"OK":"Not Found");
        c.printf("Content-Type: %s\r\n", mime);
        if (len >= 0) c.printf("Content-Length: %ld\r\n", len);
        c.print("Connection: close\r\n");
        c.print("Access-Control-Allow-Origin: *\r\n");
        if (strncmp(mime,"application/json",16)==0 || strncmp(mime,"text/html",9)==0)
            c.print("Cache-Control: no-cache, no-store\r\n");
        c.print("\r\n");
    }

    void _serveRedirect(WiFiClient& c, const char* loc) {
        c.printf("HTTP/1.1 302 Found\r\nLocation: %s\r\nConnection: close\r\n\r\n", loc);
    }

    // ── /api — JSON telemetry ─────────────────────────────────────────────────
    void _serveApi(WiFiClient& c) {
        _sendHeaders(c, 200, "application/json");

        auto chJson = [&](WiFiClient& cc, HeatingChannel* ch, int n) {
            float tf = isnan(ch->displayTempF) ? -999.0f : ch->displayTempF;
            cc.printf(
                "\"ch%d\":{"
                "\"temp\":%.2f,\"setpt\":%.1f,\"duty\":%.1f,\"ff\":%.1f,"
                "\"dtdt\":%.4f,\"status\":\"%s\","
                "\"enabled\":%s,\"safety\":%s}",
                n, tf, ch->setpointF, ch->duty*100.0f, ch->ffTerm()*100.0f,
                ch->dTdtF, ch->statusStr(),
                ch->enabled ? "true" : "false",
                ch->safetyTripped ? "true" : "false");
        };

        c.print("{");
        chJson(c, _ch1, 1);
        c.print(",");
        chJson(c, _ch2, 2);

        if (_psu) {
            bool psuOk = _psu->commsOk && (_psu->errStreak == 0);
            c.printf(
                ",\"psu\":{"
                "\"v\":%.2f,\"a\":%.2f,\"w\":%.1f,\"vin\":%.1f,"
                "\"sv\":%.2f,\"on\":%s,\"cc\":%s,\"ok\":%s,\"status\":\"%s\","
                "\"prot\":%d}",
                _psu->measV, _psu->measA, _psu->measW, _psu->inputV,
                _psu->setV,
                _psu->outputOn ? "true" : "false",
                _psu->isCC     ? "true" : "false",
                psuOk          ? "true" : "false",
                _psu->statusStr(),
                (int)_psu->protCode);
        } else {
            c.print(",\"psu\":null");
        }

        // Env block (BME680) — valid:false when sensor not ready
        if (_env.valid) {
            c.printf(",\"env\":{\"t\":%.1f,\"h\":%.1f,\"p\":%.1f,\"g\":%.1f,\"valid\":true}",
                     _env.tempF, _env.humidity, _env.pressHpa, _env.gasKohm);
        } else {
            c.print(",\"env\":{\"valid\":false}");
        }

        bool logOn = _logger && _logger->enabled();
        c.printf(",\"usb\":%s,\"logrows\":%d,\"logkb\":%d}",
                 logOn ? "true" : "false",
                 logOn ? _logger->rowsTotal() : 0,
                 logOn ? _logger->bufferUsed()/1024 : 0);
    }

    // ── /control — command endpoint ───────────────────────────────────────────
    void _serveControl(WiFiClient& c, const char* req) {
        _sendHeaders(c, 200, "application/json");

        if (_psu) {
            if (strstr(req, "cmd=psu_on"))     { _psu->setOutput(true);  c.print("{\"ok\":true}"); return; }
            if (strstr(req, "cmd=psu_off"))    { _psu->setOutput(false); c.print("{\"ok\":true}"); return; }
            if (strstr(req, "cmd=psu_sp_up"))  { _psu->setVoltage(_psu->setV + 1.0f); c.print("{\"ok\":true}"); return; }
            if (strstr(req, "cmd=psu_sp_dn"))  { _psu->setVoltage(_psu->setV - 1.0f); c.print("{\"ok\":true}"); return; }
            if (strstr(req, "cmd=psu_sp_up1")) { _psu->setVoltage(_psu->setV + 0.1f); c.print("{\"ok\":true}"); return; }
            if (strstr(req, "cmd=psu_sp_dn1")) { _psu->setVoltage(_psu->setV - 0.1f); c.print("{\"ok\":true}"); return; }
            const char* sv = strstr(req, "cmd=psu_sv&v=");
            if (sv) { _psu->setVoltage(atof(sv + 13)); c.print("{\"ok\":true}"); return; }
        }

        const char* chPtr  = strstr(req, "ch=");
        const char* cmdPtr = strstr(req, "cmd=");
        if (!chPtr || !cmdPtr) { c.print("{\"ok\":false}"); return; }

        int ch = atoi(chPtr + 3);
        HeatingChannel* hch = (ch == 1) ? _ch1 : (ch == 2) ? _ch2 : nullptr;
        if (!hch) { c.print("{\"ok\":false}"); return; }

        if      (strstr(cmdPtr, "cmd=enable"))  hch->enabled = true;
        else if (strstr(cmdPtr, "cmd=disable")) hch->enabled = false;
        else if (strstr(cmdPtr, "cmd=sp_up"))   hch->adjustSetpoint( TEMP_STEP_F);
        else if (strstr(cmdPtr, "cmd=sp_dn"))   hch->adjustSetpoint(-TEMP_STEP_F);
        else if (strstr(cmdPtr, "cmd=tog"))     hch->enabled = !hch->enabled;

        c.print("{\"ok\":true}");
    }

    // ── /status — plain text ──────────────────────────────────────────────────
    void _serveStatus(WiFiClient& c) {
        _sendHeaders(c, 200, "text/plain");
        c.printf("Battery Warmer  uptime=%lus\n", millis()/1000);
        for (int i = 0; i < 2; i++) {
            HeatingChannel* ch = (i==0) ? _ch1 : _ch2;
            float t = isnan(ch->displayTempF) ? -999.0f : ch->displayTempF;
            c.printf("CH%d: %s  %.1fF->%.0fF  duty=%.0f%%\n",
                     i+1, ch->statusStr(), t, ch->setpointF, ch->duty*100.0f);
        }
        if (_psu)
            c.printf("PSU: %s  V=%.2f  A=%.2f  setV=%.2f\n",
                     _psu->statusStr(), _psu->measV, _psu->measA, _psu->setV);
    }

    // ── /log — CSV download ───────────────────────────────────────────────────
    void _serveLog(WiFiClient& c) {
        if (!_logger || !_logger->enabled()) {
            _sendHeaders(c, 404, "text/plain");
            c.print("Logger not active");
            return;
        }
        _logger->serveFile(c);
    }

    // ── /fftable — feedforward table viewer ───────────────────────────────────
    void _serveFFTable(WiFiClient& c) {
        _sendHeaders(c, 200, "text/html");
        c.print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>FF Table</title><style>"
            "body{font-family:monospace;background:#000;color:#d0d0d0;padding:1rem}"
            "h2{color:#ff8040;margin-bottom:.5rem}"
            "table{border-collapse:collapse;margin-top:.5rem}"
            "td,th{padding:.25rem .7rem;border:1px solid #444}"
            "th{background:#282828;color:#909090;letter-spacing:.06em;font-size:.8rem}"
            "tr:nth-child(even){background:#141414}"
            "a{color:#55aaff;text-decoration:none;font-size:.85rem}"
            "</style></head><body>"));
        c.print(F("<h2>Feedforward Learned Table</h2>"
            "<p><a href='/dash'>&larr; Dashboard</a></p>"
            "<table><tr><th>SLOT</th><th>SETPT (&deg;F)</th>"
            "<th>CH1 FF%</th><th>CH2 FF%</th></tr>"));
        for (int s = 0; s < FF_SLOTS; s++) {
            float sp = SETPOINT_MIN_F + s;
            c.printf("<tr><td>%d</td><td>%.0f</td><td>%.1f%%</td><td>%.1f%%</td></tr>",
                     s, sp,
                     _ch1 ? _ch1->ff.getSlot(s)*100.0f : 0.0f,
                     _ch2 ? _ch2->ff.getSlot(s)*100.0f : 0.0f);
        }
        c.print(F("</table></body></html>"));
    }

    // ── /diag — diagnostics ───────────────────────────────────────────────────
    void _serveDiag(WiFiClient& c) {
        _sendHeaders(c, 200, "text/html");
        c.print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Diagnostics</title><style>"
            "body{font-family:monospace;background:#000;color:#d0d0d0;padding:1rem}"
            "h2{color:#ff8040}pre{background:#1a1a1a;padding:.75rem;"
            "border-radius:6px;border:1px solid #444;overflow-x:auto}"
            "a{color:#55aaff;text-decoration:none;font-size:.85rem}"
            "</style></head><body>"
            "<h2>&#x1F527; Diagnostics</h2>"
            "<p><a href='/dash'>&larr; Dashboard</a></p>"
            "<pre id='d'>Loading...</pre><script>"
            "async function load(){"
            "const r=await fetch('/api');"
            "const d=await r.json();"
            "document.getElementById('d').textContent=JSON.stringify(d,null,2);}"
            "load();setInterval(load,3000);"
            "</script></body></html>"));
    }

    // ── /dash — main SPA (4 tabs matching CrowPanel pages) ───────────────────
    void _serveDash(WiFiClient& c) {
        _sendHeaders(c, 200, "text/html");

        // ── HEAD + CSS reset + variables ──────────────────────────────────────
        c.print(F("<!DOCTYPE html><html><head>"
            "<meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>PSU Warmer</title><style>"
            "*{box-sizing:border-box;margin:0;padding:0}"
            "body{background:#000;color:#fff;"
            "font-family:ui-sans-serif,system-ui,-apple-system,sans-serif;"
            "display:flex;flex-direction:column;height:100dvh;overflow:hidden}"
            ":root{"
            "--bg:#000;--card:#282828;--tile:#1c1c1c;--bdr:#444;"
            "--btn:#585858;--bbd:#777;"
            "--acc:#ff8040;--grn:#33cc66;--txt:#fff;--dim:#909090;"
            "--mut:#d0d0d0;--red:#ff4444;--blu:#55aaff;--amb:#e3b341;"
            "--on:#1a3a1a;--on-bd:#1a4a1a;"
            "--cc:#2e2000;--cc-bd:#4a3000;"
            "--err:#2e0e0e;--err-bd:#4a1a1a;"
            "--act:#0d1a2e}"));

        // ── Header ────────────────────────────────────────────────────────────
        c.print(F(
            "header{background:#111;border-bottom:1px solid var(--bdr);"
            "padding:.45rem .75rem;display:flex;align-items:center;"
            "justify-content:space-between;flex-shrink:0;gap:.5rem;min-height:46px}"
            ".htitle{font-size:.88rem;font-weight:700;color:var(--acc);white-space:nowrap;"
            "letter-spacing:.04em}"
            ".tabs{display:flex;gap:.3rem;flex-shrink:0}"
            ".tab{padding:.28rem .65rem;border-radius:5px;background:var(--btn);"
            "border:1px solid var(--bbd);color:var(--dim);font-size:.73rem;"
            "cursor:pointer;font-weight:700;letter-spacing:.05em;white-space:nowrap}"
            ".tab.act{background:var(--act);border-color:var(--blu);color:var(--blu)}"
            ".envhdr{font-size:.8rem;color:var(--mut);white-space:nowrap;text-align:right;"
            "min-width:80px}"));

        // ── Page containers ───────────────────────────────────────────────────
        c.print(F(
            ".page{display:none;flex:1;overflow-y:auto;padding:.65rem;"
            "flex-direction:column;gap:.6rem;min-height:0}"
            ".page.show{display:flex}"));

        // ── Shared: channel card base styles ──────────────────────────────────
        c.print(F(
            ".ch-hdr{display:flex;justify-content:space-between;align-items:center}"
            ".chnm{font-size:.7rem;font-weight:700;letter-spacing:.12em;color:var(--acc)}"
            ".en-btn{padding:.22rem .55rem;border-radius:4px;background:var(--btn);"
            "border:1px solid var(--bbd);color:var(--dim);font-size:.72rem;"
            "font-weight:700;cursor:pointer;min-width:44px}"
            ".en-btn.on{background:var(--on);border-color:var(--on-bd);color:var(--grn)}"
            ".ch-stat-row{display:flex;justify-content:space-between;align-items:center}"
            ".ch-stat{font-size:.75rem;font-weight:700;padding:.14rem .38rem;"
            "border-radius:3px;background:#000;letter-spacing:.04em}"
            ".ch-rate{font-size:.68rem;color:var(--dim)}"
            ".bar-wrap{background:#111;border-radius:3px;height:9px;overflow:hidden}"
            ".bar{height:100%;width:0%;border-radius:3px;transition:width .4s,background-color .4s}"
            ".duty-row{display:flex;justify-content:flex-end}"
            ".duty-pct{font-size:.7rem;color:var(--dim)}"
            ".divider{height:1px;background:var(--bdr);flex-shrink:0}"
            ".tgt-lbl{font-size:.62rem;color:var(--dim);text-align:center;"
            "letter-spacing:.1em;text-transform:uppercase}"
            ".pm{width:38px;height:38px;border-radius:6px;background:var(--tile);"
            "border:1px solid var(--bdr);color:var(--txt);font-size:1.25rem;"
            "cursor:pointer;display:flex;align-items:center;justify-content:center;"
            "flex-shrink:0;line-height:1}"
            ".pm:active{background:#333}"
            ".sp-val{font-size:1.05rem;font-weight:700;min-width:3.4rem;text-align:center}"));

        // ── Dashboard: PSU strip ──────────────────────────────────────────────
        c.print(F(
            ".psu-strip{display:flex;gap:.35rem;align-items:stretch;flex-shrink:0;height:68px}"
            ".stile{flex:1;background:var(--tile);border:1px solid var(--bdr);"
            "border-radius:6px;display:flex;flex-direction:column;"
            "align-items:center;justify-content:center;padding:.3rem .15rem;min-width:0}"
            ".stile.tap{cursor:pointer;border-color:#2a4a7a}"
            ".stile.tap:active{background:#1a2030}"
            ".stv{font-size:1.6rem;font-weight:700;line-height:1;white-space:nowrap;"
            "overflow:hidden;text-overflow:ellipsis;max-width:100%;padding:0 4px}"
            ".stu{font-size:.55rem;color:var(--dim);margin-top:2px;letter-spacing:.06em}"
            ".psu-onbtn{flex-shrink:0;width:86px;border-radius:6px;background:var(--btn);"
            "border:1px solid var(--bbd);color:var(--dim);font-size:.75rem;"
            "font-weight:700;cursor:pointer;height:100%}"
            ".psu-onbtn.cv{background:var(--on);border-color:var(--on-bd);color:var(--grn)}"
            ".psu-onbtn.cc{background:var(--cc);border-color:var(--cc-bd);color:var(--amb)}"
            ".psu-onbtn.err{background:var(--err);border-color:var(--err-bd);color:var(--red)}"));

        // ── Dashboard: channel cards ──────────────────────────────────────────
        c.print(F(
            ".ch-row{display:flex;gap:.5rem;flex:1;min-height:0}"
            ".ch-card{flex:1;background:var(--card);border:1px solid var(--bdr);"
            "border-radius:8px;padding:.7rem;display:flex;flex-direction:column;gap:.38rem}"
            ".ch-temp{font-size:2.6rem;font-weight:700;text-align:center;"
            "color:var(--dim);line-height:1;flex-shrink:0}"
            ".ch-temp.live{color:var(--txt)}"
            // Temperature expands to fill available space and centers its text,
            // which naturally bottom-aligns the status/bar/target group below it
            ".ch-card .ch-temp{flex:1;display:flex;align-items:center;justify-content:center}"
            ".sp-row{display:flex;align-items:center;justify-content:center;gap:.35rem}"));

        // ── PSU page ──────────────────────────────────────────────────────────
        c.print(F(
            ".psu-tiles{display:grid;grid-template-columns:repeat(4,1fr);"
            "gap:.5rem;flex-shrink:0}"
            ".ptile{background:var(--tile);border:1px solid var(--bdr);border-radius:8px;"
            "padding:.9rem .2rem;display:flex;flex-direction:column;"
            "align-items:center;justify-content:center;min-height:110px}"
            ".ptv{font-size:3.2rem;font-weight:700;line-height:1}"
            ".ptu{font-size:.65rem;color:var(--dim);margin-top:6px;letter-spacing:.05em}"
            ".presets{display:grid;grid-template-columns:repeat(9,1fr);"
            "gap:.35rem;flex-shrink:0}"
            ".pvbtn{background:var(--tile);border:1px solid var(--bdr);border-radius:5px;"
            "display:flex;align-items:center;justify-content:center;min-height:48px;"
            "font-size:.82rem;color:var(--dim);cursor:pointer;font-weight:700}"
            ".pvbtn.act{background:var(--act);border-color:var(--blu);color:var(--blu)}"
            ".pvbtn:active{border-color:var(--blu)}"
            ".adj-row{display:flex;align-items:center;gap:.4rem;"
            "flex-shrink:0;justify-content:center}"
            ".abtn{width:70px;height:70px;border-radius:7px;background:var(--tile);"
            "border:1px solid var(--bdr);cursor:pointer;display:flex;"
            "flex-direction:column;align-items:center;justify-content:center;gap:2px}"
            ".anum{font-size:.95rem;color:var(--txt);font-weight:700}"
            ".asym{font-size:1.3rem;color:var(--dim);line-height:1}"
            ".abtn:active{background:#333}"
            ".svbox{flex:1;max-width:200px;background:var(--tile);"
            "border:1px solid #2a4a7a;border-radius:8px;"
            "display:flex;flex-direction:column;align-items:center;"
            "justify-content:center;height:70px;cursor:pointer;padding:.35rem}"
            ".svbox:active{background:#1a2030}"
            ".svlbl{font-size:.58rem;color:var(--dim);letter-spacing:.1em;text-transform:uppercase}"
            ".svval{font-size:1.4rem;font-weight:700}"
            ".psu-main-btn{width:100%;padding:1.1rem;border-radius:8px;"
            "background:var(--btn);border:1px solid var(--bbd);color:var(--dim);"
            "font-size:1.15rem;font-weight:700;cursor:pointer;letter-spacing:.07em}"
            ".psu-main-btn.cv{background:var(--on);border-color:var(--on-bd);color:var(--grn)}"
            ".psu-main-btn.cc{background:var(--cc);border-color:var(--cc-bd);color:var(--amb)}"
            ".psu-main-btn.err{background:var(--err);border-color:var(--err-bd);color:var(--red)}"));

        // ── Warmer page: constrained channel cards (not full-stretch) ────────
        c.print(F(
            ".wch-row{display:flex;gap:1rem;justify-content:center;"
            "align-items:flex-start;padding:.5rem 0;flex-shrink:0}"
            ".wch-card{width:300px;flex-shrink:0;background:var(--card);"
            "border:1px solid var(--bdr);border-radius:8px;padding:.85rem;"
            "display:flex;flex-direction:column;gap:.45rem}"
            ".wch-temp{font-size:3.6rem;font-weight:700;text-align:center;"
            "color:var(--dim);line-height:1;flex-shrink:0}"
            ".wch-temp.live{color:var(--txt)}"
            ".wsp-row{display:flex;align-items:center;justify-content:center;gap:.45rem}"
            ".wpm{width:52px;height:52px;border-radius:8px;background:var(--tile);"
            "border:1px solid var(--bdr);color:var(--txt);font-size:1.55rem;"
            "cursor:pointer;display:flex;align-items:center;justify-content:center;"
            "flex-shrink:0;line-height:1}"
            ".wpm:active{background:#333}"
            ".wspval{font-size:1.35rem;font-weight:700;min-width:4rem;text-align:center}"));

        // ── Env page ──────────────────────────────────────────────────────────
        c.print(F(
            ".env-grid{display:grid;grid-template-columns:1fr 1fr;"
            "gap:.5rem;flex:1;min-height:0}"
            // Card: horizontal split — left=info, divider, right=gauge or tiers
            ".env-card{background:var(--tile);border:1px solid var(--bdr);border-radius:8px;"
            "padding:.75rem;display:flex;flex-direction:row;gap:.55rem;"
            "overflow:hidden;align-items:stretch}"
            ".env-left{display:flex;flex-direction:column;gap:.25rem;flex:1;min-width:0}"
            ".env-vdiv{width:1px;background:#2a2a2a;align-self:stretch;flex-shrink:0}"
            // All 4 cards share the same right-panel width → dividers align perfectly
            ".env-right{flex-shrink:0;width:115px;display:flex;"
            "align-items:center;justify-content:center}"
            // Arc gauge SVG
            ".egauge{width:94px;height:94px;display:block}"
            // Tier bars: same 115px container, width:100% on .tier ensures
            // flex:1 on .tbar-bg resolves correctly; scaleX drives the fill
            ".env-tiers{flex-direction:column;gap:7px;justify-content:center}"
            ".tier{display:flex;align-items:center;gap:.35rem;width:100%}"
            ".tlbl{font-size:.58rem;color:var(--dim);width:34px;flex-shrink:0;"
            "text-align:right;white-space:nowrap}"
            ".tbar-bg{flex:1;height:7px;background:#2e2e2e;border-radius:3px;overflow:hidden}"
            ".tbar{height:7px;border-radius:3px;width:100%;transform:scaleX(0);"
            "transform-origin:left center;transition:transform .4s}"
            // Left-panel text elements
            ".elbl{font-size:.6rem;color:var(--dim);letter-spacing:.1em;font-weight:700;"
            "text-transform:uppercase}"
            // Value and unit are horizontally centred; margin-top:auto centres them
            // vertically in the space between the label (top) and range/badge (bottom)
            ".eval{font-size:2.2rem;font-weight:700;line-height:1.05;"
            "text-align:center;margin-top:auto}"
            ".eunit{font-size:.75rem;color:var(--dim);text-align:center}"
            ".erange{font-size:.6rem;color:var(--dim);margin-top:auto;line-height:1.4;"
            "text-align:center}"
            ".iaq-badge{display:inline-flex;align-items:center;padding:.18rem .55rem;"
            "border-radius:12px;font-size:.72rem;font-weight:700;border:1px solid;"
            "width:fit-content;margin-top:auto;align-self:center}"));

        // ── Status bar ────────────────────────────────────────────────────────
        c.print(F(
            ".sbar{background:#0a0a0a;border-top:1px solid var(--bdr);"
            "padding:.28rem .7rem;display:flex;align-items:center;gap:.6rem;"
            "flex-shrink:0;min-height:28px}"
            ".di{display:flex;align-items:center;gap:3px}"
            ".dot{width:6px;height:6px;border-radius:50%;background:var(--bdr)}"
            ".dot.on{background:var(--grn)}.dot.off{background:var(--red)}"
            ".dlbl{font-size:.58rem;color:var(--dim)}"
            "#logstat{font-size:.65rem;color:var(--dim);margin-left:auto}"
            ".nlink{color:var(--blu);font-size:.68rem;text-decoration:none;margin-left:.6rem}"
            // Color utility classes matching CrowPanel palette
            ".g{color:var(--grn)}.o{color:var(--acc)}.r{color:var(--red)}"
            ".b{color:var(--blu)}.a{color:var(--amb)}.m{color:var(--mut)}"
            // Responsive breakpoints
            "@media(max-width:560px){"
            ".psu-tiles{grid-template-columns:repeat(2,1fr)}"
            ".presets{grid-template-columns:repeat(5,1fr)}"
            ".ch-row{flex-direction:column}"
            ".wch-row{flex-direction:column;align-items:center}"
            ".wch-card{width:100%;max-width:360px}"
            ".env-grid{grid-template-columns:1fr}"
            // Header: shrink title and tabs on mobile; keep env summary visible but smaller
            ".htitle{font-size:.72rem;letter-spacing:.02em}"
            ".envhdr{font-size:.65rem;min-width:0}"
            ".tabs{gap:.15rem}.tab{padding:.22rem .38rem;font-size:.63rem}"
            // PSU strip: smaller tile text so VIN/SET values don't get clipped
            ".stv{font-size:1.05rem;padding:0 2px}"
            ".psu-strip{height:58px}"
            ".psu-onbtn{width:64px;font-size:.68rem}}"
            // ── Desktop: centered fixed-ratio card (1.5× CrowPanel 800×480) ──
            // page-wrap centers the card; page-card is 1200×720 (same 5:3 ratio)
            // On mobile these are transparent flex pass-throughs
            ".page-wrap{flex:1;display:flex;flex-direction:column;min-height:0;overflow:hidden}"
            ".page-card{flex:1;display:flex;flex-direction:column;min-height:0;overflow:hidden}"
            "@media(min-width:1100px){"
            ".page-wrap{flex-direction:row;align-items:center;justify-content:center;"
            "padding:20px;overflow:auto;"
            "background:radial-gradient(ellipse at 50% 50%,#111 0%,#040404 80%)}"
            ".page-card{width:1200px;height:720px;flex:none;"
            "border:1px solid var(--bdr);border-radius:10px;overflow:hidden;"
            "box-shadow:0 0 80px rgba(255,128,64,.05),0 4px 40px rgba(0,0,0,.8);}"
            // ── Desktop-only: PSU page tile readouts much bigger ──────────────
            ".ptv{font-size:3.8rem}"
            ".ptu{font-size:.72rem;margin-top:6px}"
            // ── Desktop-only: dashboard channel card — enlarged for tall cards ─
            // Temperature, status, bar, and target controls all scale up
            ".ch-temp{font-size:4.8rem}"
            ".chnm{font-size:.85rem}"
            ".en-btn{padding:.3rem .7rem;font-size:.8rem}"
            ".ch-stat{font-size:.92rem;padding:.22rem .52rem}"
            ".ch-rate{font-size:.8rem}"
            ".bar-wrap{height:14px}"
            ".duty-pct{font-size:.82rem}"
            ".tgt-lbl{font-size:.76rem;letter-spacing:.12em}"
            ".pm{width:50px;height:50px;font-size:1.5rem}"
            ".sp-val{font-size:1.35rem}}"
            "</style></head>"));

        // ── BODY start + header ───────────────────────────────────────────────
        c.print(F("<body>"
            "<header>"
            "<span class='htitle'>PSU WARMER</span>"
            "<div class='tabs'>"
            "<button class='tab act' id='tb-dash' onclick='pg(\"dash\")'>DASHBOARD</button>"
            "<button class='tab' id='tb-psu'  onclick='pg(\"psu\")'>PSU</button>"
            "<button class='tab' id='tb-warm' onclick='pg(\"warm\")'>WARMER</button>"
            "<button class='tab' id='tb-env'  onclick='pg(\"env\")'>ENV</button>"
            "</div>"
            "<span class='envhdr' id='envhdr'>---</span>"
            "</header>"));

        // ── Dashboard page ────────────────────────────────────────────────────
        c.print(F("<div class='page-wrap'><div class='page-card'>"
            "<div id='pg-dash' class='page show'>"
            "<div class='psu-strip'>"
            "<div class='stile'><div class='stv g' id='ds-v'>---</div>"
            "<div class='stu'>V OUT</div></div>"
            "<div class='stile'><div class='stv o' id='ds-a'>---</div>"
            "<div class='stu'>A OUT</div></div>"
            "<div class='stile'><div class='stv r' id='ds-w'>---</div>"
            "<div class='stu'>WATTS</div></div>"
            "<div class='stile'><div class='stv m' id='ds-vin'>---</div>"
            "<div class='stu'>V IN</div></div>"
            "<div class='stile tap' onclick='pg(\"psu\")'>"
            "<div class='stv' id='ds-sv'>---</div>"
            "<div class='stu'>SET</div></div>"
            "<button class='psu-onbtn' id='ds-on' onclick='psuTog()'>---</button>"
            "</div>"
            "<div class='ch-row'>"));

        // Dashboard channel cards (two identical structures, different IDs)
        for (int z = 1; z <= 2; z++) {
            char buf[800];
            snprintf(buf, sizeof(buf),
                "<div class='ch-card'>"
                "<div class='ch-hdr'>"
                "<span class='chnm'>CH%d</span>"
                "<button class='en-btn' id='de%d' onclick='cEnTog(%d)'>OFF</button>"
                "</div>"
                "<div class='ch-temp' id='dt%d'>---</div>"
                "<div class='ch-stat-row'>"
                "<span class='ch-stat' id='ds%d' style='color:var(--dim)'>DISABLED</span>"
                "<span class='ch-rate' id='dr%d'></span>"
                "</div>"
                "<div class='bar-wrap'><div class='bar' id='db%d'></div></div>"
                "<div class='duty-row'><span class='duty-pct' id='dp%d'>0%%</span></div>"
                "<div class='divider'></div>"
                "<div class='tgt-lbl'>Target</div>"
                "<div class='sp-row'>"
                "<button class='pm' onclick='cSp(%d,-1)'>&#8722;</button>"
                "<span class='sp-val' id='dsp%d'>---</span>"
                "<button class='pm' onclick='cSp(%d,1)'>+</button>"
                "</div></div>",
                z,z,z,z,z,z,z,z,z,z,z);
            c.print(buf);
        }
        c.print(F("</div></div>")); // close ch-row, pg-dash

        // ── PSU page ──────────────────────────────────────────────────────────
        c.print(F("<div id='pg-psu' class='page'>"
            "<div class='psu-tiles'>"
            "<div class='ptile'><div class='ptv g' id='pv'>---</div>"
            "<div class='ptu'>V OUT</div></div>"
            "<div class='ptile'><div class='ptv o' id='pa'>---</div>"
            "<div class='ptu'>A OUT</div></div>"
            "<div class='ptile'><div class='ptv r' id='pw'>---</div>"
            "<div class='ptu'>WATTS</div></div>"
            "<div class='ptile'><div class='ptv m' id='pvin'>---</div>"
            "<div class='ptu'>V IN</div></div>"
            "</div>"
            "<div class='presets'>"
            "<div class='pvbtn' data-v='4'  onclick='psuset(4)'>4V</div>"
            "<div class='pvbtn' data-v='5'  onclick='psuset(5)'>5V</div>"
            "<div class='pvbtn' data-v='6'  onclick='psuset(6)'>6V</div>"
            "<div class='pvbtn' data-v='7'  onclick='psuset(7)'>7V</div>"
            "<div class='pvbtn' data-v='8'  onclick='psuset(8)'>8V</div>"
            "<div class='pvbtn' data-v='9'  onclick='psuset(9)'>9V</div>"
            "<div class='pvbtn' data-v='10' onclick='psuset(10)'>10V</div>"
            "<div class='pvbtn' data-v='11' onclick='psuset(11)'>11V</div>"
            "<div class='pvbtn' data-v='12' onclick='psuset(12)'>12V</div>"
            "</div>"
            "<div class='adj-row'>"
            "<button class='abtn' onclick='psusp(-0.5)'>"
            "<div class='anum'>0.5</div><div class='asym'>&#8722;</div></button>"
            "<button class='abtn' onclick='psusp(-0.1)'>"
            "<div class='anum'>0.1</div><div class='asym'>&#8722;</div></button>"
            "<div class='svbox' onclick='psuKp()'>"
            "<div class='svlbl'>Set Point</div>"
            "<div class='svval' id='psv'>--.- V</div>"
            "</div>"
            "<button class='abtn' onclick='psusp(0.1)'>"
            "<div class='anum'>0.1</div><div class='asym'>+</div></button>"
            "<button class='abtn' onclick='psusp(0.5)'>"
            "<div class='anum'>0.5</div><div class='asym'>+</div></button>"
            "</div>"
            "<button class='psu-main-btn' id='pon' onclick='psuTog()'>---</button>"
            "</div>")); // pg-psu

        // ── Warmer page ───────────────────────────────────────────────────────
        c.print(F("<div id='pg-warm' class='page'><div class='wch-row'>"));
        for (int z = 1; z <= 2; z++) {
            char buf[820];
            snprintf(buf, sizeof(buf),
                "<div class='wch-card'>"
                "<div class='ch-hdr'>"
                "<span class='chnm'>CH%d</span>"
                "<button class='en-btn' id='we%d' onclick='cEnTog(%d)'>OFF</button>"
                "</div>"
                "<div class='wch-temp' id='wt%d'>---</div>"
                "<div class='ch-stat-row'>"
                "<span class='ch-stat' id='wst%d' style='color:var(--dim)'>DISABLED</span>"
                "<span class='ch-rate' id='wdr%d'></span>"
                "</div>"
                "<div class='bar-wrap'><div class='bar' id='wb%d'></div></div>"
                "<div class='duty-row'><span class='duty-pct' id='wdp%d'>0%%</span></div>"
                "<div class='divider'></div>"
                "<div class='tgt-lbl'>Target</div>"
                "<div class='wsp-row'>"
                "<button class='wpm' onclick='cSp(%d,-1)'>&#8722;</button>"
                "<span class='wspval' id='wsp%d'>---</span>"
                "<button class='wpm' onclick='cSp(%d,1)'>+</button>"
                "</div></div>",
                z,z,z,z,z,z,z,z,z,z,z);
            c.print(buf);
        }
        c.print(F("</div></div>")); // wch-row, pg-warm

        // ── Env page ──────────────────────────────────────────────────────────
        // Arc gauge maths: r=38, C=2π×38≈239, 270°arc=179px, gap=60px
        // rotate(45 50 50) starts arc at SE, sweeps CW to NE — C-shape opening right
        c.print(F("<div id='pg-env' class='page'><div class='env-grid'>"

            // ── Temperature ──────────────────────────────────────────────────
            "<div class='env-card'>"
            "<div class='env-left'>"
            "<div class='elbl'>Temperature</div>"
            "<div class='eval g' id='et'>---</div>"
            "<div class='eunit g'>&#xB0;F</div>"
            "<div class='erange'>Range<br>40&#x2013;120&#xB0;F</div>"
            "</div>"
            "<div class='env-vdiv'></div>"
            "<div class='env-right'>"
            "<svg class='egauge' viewBox='0 0 100 100'>"
            "<g transform='rotate(45 50 50)'>"
            "<circle cx='50' cy='50' r='38' fill='none' stroke='#1a2a1a'"
            " stroke-width='10' stroke-dasharray='179 239' stroke-linecap='round'/>"
            "<circle cx='50' cy='50' r='38' fill='none' stroke='#33cc66'"
            " stroke-width='10' stroke-dasharray='0 239' stroke-linecap='round' id='etg'/>"
            "</g>"
            "<text x='50' y='60' text-anchor='middle' fill='#505050'"
            " font-size='11' font-family='sans-serif'>&#xB0;F</text>"
            "</svg>"
            "</div></div>"));

        c.print(F(
            // ── Humidity ─────────────────────────────────────────────────────
            "<div class='env-card'>"
            "<div class='env-left'>"
            "<div class='elbl'>Humidity</div>"
            "<div class='eval b' id='eh'>---</div>"
            "<div class='eunit b'>%RH</div>"
            "<div class='erange'>Comfort<br>30&#x2013;60%</div>"
            "</div>"
            "<div class='env-vdiv'></div>"
            "<div class='env-right'>"
            "<svg class='egauge' viewBox='0 0 100 100'>"
            "<g transform='rotate(45 50 50)'>"
            "<circle cx='50' cy='50' r='38' fill='none' stroke='#1a1e2a'"
            " stroke-width='10' stroke-dasharray='179 239' stroke-linecap='round'/>"
            "<circle cx='50' cy='50' r='38' fill='none' stroke='#55aaff'"
            " stroke-width='10' stroke-dasharray='0 239' stroke-linecap='round' id='ehg'/>"
            "</g>"
            "<text x='50' y='60' text-anchor='middle' fill='#505050'"
            " font-size='11' font-family='sans-serif'>%</text>"
            "</svg>"
            "</div></div>"

            // ── Pressure ─────────────────────────────────────────────────────
            "<div class='env-card'>"
            "<div class='env-left'>"
            "<div class='elbl'>Pressure</div>"
            "<div class='eval m' id='ep'>---</div>"
            "<div class='eunit' style='color:var(--mut)'>hPa</div>"
            "<div class='erange'>Normal: 1013<br>950&#x2013;1050 hPa</div>"
            "</div>"
            "<div class='env-vdiv'></div>"
            "<div class='env-right'>"
            "<svg class='egauge' viewBox='0 0 100 100'>"
            "<g transform='rotate(45 50 50)'>"
            "<circle cx='50' cy='50' r='38' fill='none' stroke='#252525'"
            " stroke-width='10' stroke-dasharray='179 239' stroke-linecap='round'/>"
            "<circle cx='50' cy='50' r='38' fill='none' stroke='#d0d0d0'"
            " stroke-width='10' stroke-dasharray='0 239' stroke-linecap='round' id='epg'/>"
            "</g>"
            "<text x='50' y='60' text-anchor='middle' fill='#505050'"
            " font-size='9' font-family='sans-serif'>hPa</text>"
            "</svg>"
            "</div></div>"));

        c.print(F(
            // ── Air Quality (tier bars instead of arc, matching CrowPanel) ────
            "<div class='env-card'>"
            "<div class='env-left'>"
            "<div class='elbl'>Air Quality</div>"
            "<div class='eval a' id='eg'>---</div>"
            "<div class='eunit' style='color:var(--amb)'>k&#x3A9; gas</div>"
            "<span class='iaq-badge' id='eiaq' style='color:var(--amb)'>---</span>"
            "</div>"
            "<div class='env-vdiv'></div>"
            "<div class='env-right env-tiers'>"
            "<div class='tier'><span class='tlbl'>Excel</span>"
            "<div class='tbar-bg'><div class='tbar' id='tier0'"
            " style='background:#33cc66'></div></div></div>"
            "<div class='tier'><span class='tlbl'>Good</span>"
            "<div class='tbar-bg'><div class='tbar' id='tier1'"
            " style='background:#55ccaa'></div></div></div>"
            "<div class='tier'><span class='tlbl'>Fair</span>"
            "<div class='tbar-bg'><div class='tbar' id='tier2'"
            " style='background:#e3b341'></div></div></div>"
            "<div class='tier'><span class='tlbl'>Poor</span>"
            "<div class='tbar-bg'><div class='tbar' id='tier3'"
            " style='background:#ff8040'></div></div></div>"
            "<div class='tier'><span class='tlbl'>Bad</span>"
            "<div class='tbar-bg'><div class='tbar' id='tier4'"
            " style='background:#ff4444'></div></div></div>"
            "</div></div>"

            "</div></div>")); // env-grid, pg-env

        c.print(F("</div></div>")); // page-card, page-wrap

        // ── Status bar ────────────────────────────────────────────────────────
        c.print(F(
            "<div class='sbar'>"
            "<div class='di'><div class='dot' id='dLog'></div>"
            "<span class='dlbl'>LOG</span></div>"
            "<div class='di'><div class='dot' id='dWiFi'></div>"
            "<span class='dlbl'>WiFi</span></div>"
            "<div class='di'><div class='dot' id='dPsu'></div>"
            "<span class='dlbl'>PSU</span></div>"
            "<span id='logstat'>---</span>"
            "<a class='nlink' href='/fftable'>FF Table</a>"
            "<a class='nlink' href='/diag'>Diag</a>"
            "<a class='nlink' href='/log'>&#x2B07; Log</a>"
            "</div>"));

        // ── JavaScript: utilities ─────────────────────────────────────────────
        c.print(F("<script>"
            "'use strict';"
            "let D=null,C=false,CP='dash';"
            // Page switch
            "function pg(p){"
            "CP=p;"
            "['dash','psu','warm','env'].forEach(n=>{"
            "document.getElementById('pg-'+n).classList.toggle('show',n===p);"
            "const t=document.getElementById('tb-'+n);"
            "if(t)t.classList.toggle('act',n===p);});"
            "}"
            // Temperature display
            "function tf(f,dp){"
            "dp=dp||1;"
            "if(f<=-998)return'---';"
            "return C?((f-32)*5/9).toFixed(dp)+'\\u00b0C':f.toFixed(dp)+'\\u00b0F';}"
            "function tfsp(f){"
            "if(f<=-998)return'---';"
            "return C?Math.round((f-32)*5/9)+'\\u00b0C':Math.round(f)+'\\u00b0F';}"
            // Status text color (matches CrowPanel statusColor())
            "function sc(s){"
            "return{'HEATING':'#ff8040','HOLDING':'#33cc66','REDUCING':'#55aaff',"
            "'CUTOFF':'#ff4444','SENS ERR':'#ff4444'}[s]||'#909090';}"
            // Duty bar gradient color (green→orange→red)
            "function bc(pct){"
            "const h=Math.max(0,120-pct*1.2);"
            "return'hsl('+h+',75%,40%)';}"
            // Generic element text setter (no-op if element missing)
            "function st(id,txt){"
            "const e=document.getElementById(id);if(e)e.textContent=txt;}"));

        // ── JavaScript: paint channel card ────────────────────────────────────
        c.print(F(
            // paintCh(ch, n, pfx)
            // ch = API channel object, n = 1|2, pfx = 'd' (dashboard) or 'w' (warmer)
            "function paintCh(ch,n,pfx){"
            "if(!ch)return;"
            "const en=ch.enabled,ok=ch.temp>-998;"
            // Temperature
            "const tEl=document.getElementById(pfx+'t'+n);"
            "if(tEl){"
            "tEl.textContent=tf(ch.temp);"
            "tEl.className=(pfx==='d'?'ch-temp':'wch-temp')+(en&&ok?' live':'');}"
            // Status
            "const se=document.getElementById(pfx==='d'?'ds'+n:'wst'+n);"
            "if(se){se.textContent=ch.status;se.style.color=sc(ch.status);}"
            // dT/dt rate
            "const re=document.getElementById(pfx==='d'?'dr'+n:'wdr'+n);"
            "if(re){"
            "const r=ch.dtdt*60;"
            "re.textContent=Math.abs(r)>0.1?(r>0?'+':'')+r.toFixed(1)+'\\u00b0/m':\"\";}"
            // Duty bar
            "const bar=document.getElementById(pfx+'b'+n);"
            "if(bar){"
            "bar.style.width=Math.min(ch.duty,100)+'%';"
            "bar.style.backgroundColor=bc(ch.duty);}"
            // Duty %
            "const dp=document.getElementById(pfx==='d'?'dp'+n:'wdp'+n);"
            "if(dp)dp.textContent=ch.duty.toFixed(0)+'%';"
            // Setpoint
            "const sp=document.getElementById(pfx==='d'?'dsp'+n:'wsp'+n);"
            "if(sp)sp.textContent=tfsp(ch.setpt);"
            // Enable button
            "const eb=document.getElementById(pfx==='d'?'de'+n:'we'+n);"
            "if(eb){eb.textContent=en?'ON':'OFF';eb.className='en-btn'+(en?' on':'');}"
            "}"));

        // ── JavaScript: paint PSU (both strip and PSU page) ───────────────────
        c.print(F(
            "function paintPsu(p){"
            "const dsOn=document.getElementById('ds-on');"
            "const pOn=document.getElementById('pon');"
            "const dPsu=document.getElementById('dPsu');"
            "if(!p||!p.ok){"
            // No comm: blank all readouts, show fault state
            "['ds-v','ds-a','ds-w','ds-vin','ds-sv',"
            "'pv','pa','pw','pvin','psv'].forEach("
            "function(id){st(id,'---');});"
            "if(dsOn){dsOn.textContent='NO COMM';dsOn.className='psu-onbtn err';}"
            "if(pOn){pOn.textContent='NO COMM';pOn.className='psu-main-btn err';}"
            "if(dPsu)dPsu.className='dot off';"
            "document.querySelectorAll('.pvbtn').forEach(function(b){"
            "b.classList.remove('act');});"
            "return;}"
            // Good data
            "st('ds-v',p.v.toFixed(1));st('ds-a',p.a.toFixed(1));"
            "st('ds-w',p.w.toFixed(1));st('ds-vin',p.vin.toFixed(1));"
            "st('ds-sv',p.sv.toFixed(2));"
            "st('pv',p.v.toFixed(1));st('pa',p.a.toFixed(1));"
            "st('pw',p.w.toFixed(1));st('pvin',p.vin.toFixed(1));"
            "st('psv',p.sv.toFixed(2)+' V');"
            // Button classes + labels
            "const cls=p.prot?'err':p.on?(p.cc?'cc':'cv'):'';"
            "const lbl=p.prot?p.status:p.on?(p.cc?'ON \\u00b7 CC':'ON \\u00b7 CV'):'OFF';"
            "if(dsOn){dsOn.textContent=lbl;dsOn.className='psu-onbtn '+cls;}"
            "if(pOn){pOn.textContent=lbl;pOn.className='psu-main-btn '+cls;}"
            "if(dPsu)dPsu.className=p.ok?'dot on':'dot off';"
            // Preset highlight
            "document.querySelectorAll('.pvbtn').forEach(function(b){"
            "b.classList.toggle('act',parseFloat(b.dataset.v)===p.sv);});"
            "}"));

        // ── JavaScript: paint Env page ────────────────────────────────────────
        c.print(F(
            // Set a C-shaped SVG arc to pct (0.0–1.0). 179 = full 270° at r=38.
            "function setArc(id,pct){"
            "const e=document.getElementById(id);"
            "if(e)e.setAttribute('stroke-dasharray',"
            "(Math.min(Math.max(pct,0),1)*179).toFixed(1)+' 239');}"
            "function paintEnv(e){"
            "if(!e||!e.valid)return;"
            // IAQ classification matching CrowPanel iaqLabel() / iaqColor()
            "const iaq=e.g>=300?'Excellent':e.g>=150?'Good':"
            "e.g>=50?'Fair':e.g>=25?'Poor':'Bad';"
            "const iqc=e.g>=300?'#33cc66':e.g>=150?'#55ccaa':"
            "e.g>=50?'#e3b341':e.g>=25?'#ff8040':'#ff4444';"
            // Text values
            "st('et',e.t.toFixed(1));st('eh',e.h.toFixed(1));"
            "st('ep',e.p.toFixed(1));st('eg',e.g.toFixed(1));"
            "const egEl=document.getElementById('eg');"
            "if(egEl)egEl.style.color=iqc;"
            "const ib=document.getElementById('eiaq');"
            "if(ib){ib.textContent=iaq;ib.style.color=iqc;}"
            // Header summary
            "document.getElementById('envhdr').textContent="
            "e.t.toFixed(1)+'\\u00b0  '+e.h.toFixed(1)+'%';"
            // Arc gauges (270° sweep, 179px = 100%)
            "setArc('etg',Math.min(Math.max((e.t-40)/80,0),1));"
            "setArc('ehg',Math.min(e.h/100,1));"
            "setArc('epg',Math.min(Math.max((e.p-950)/100,0),1));"
            // Tier bars — each fills proportionally within its own range
            // matching CrowPanel: tierMin[]={300,150,50,25,0}, tierMax[]={500,300,150,50,25}
            "const tm=[300,150,50,25,0],tx=[500,300,150,50,25];"
            "for(let i=0;i<5;i++){"
            "const pct=Math.min(Math.max((e.g-tm[i])/(tx[i]-tm[i]),0),1);"
            "const bar=document.getElementById('tier'+i);"
            "if(bar)bar.style.transform='scaleX('+pct.toFixed(3)+')';}"
            "}"));

        // ── JavaScript: master paint + meta indicators ────────────────────────
        c.print(F(
            "function paint(d){"
            "D=d;"
            "[1,2].forEach(function(n){"
            "paintCh(d['ch'+n],n,'d');"
            "paintCh(d['ch'+n],n,'w');"
            "});"
            "paintPsu(d.psu);"
            "paintEnv(d.env);"
            "const dw=document.getElementById('dWiFi');"
            "if(dw)dw.className='dot on';"
            "const dl=document.getElementById('dLog');"
            "if(dl)dl.className=d.usb?'dot on':'dot off';"
            "st('logstat',d.usb?d.logrows+' rows':'---');"
            "}"));

        // ── JavaScript: control functions ─────────────────────────────────────
        c.print(F(
            // Channel enable toggle
            "async function cEnTog(n){"
            "const en=D&&D['ch'+n]&&D['ch'+n].enabled;"
            "try{await fetch('/control?ch='+n+'&cmd='+(en?'disable':'enable'));"
            "poll();}catch(e){}}"
            // Channel setpoint ±1°F
            "async function cSp(n,d){"
            "try{await fetch('/control?ch='+n+'&cmd='+(d>0?'sp_up':'sp_dn'));"
            "poll();}catch(e){}}"
            // PSU output toggle
            "async function psuTog(){"
            "const on=D&&D.psu&&D.psu.on;"
            "try{await fetch('/control?cmd='+(on?'psu_off':'psu_on'));"
            "poll();}catch(e){}}"
            // PSU setpoint bump (±V)
            "async function psusp(d){"
            "const sv=D&&D.psu?+(D.psu.sv+d).toFixed(2):12;"
            "try{await fetch('/control?cmd=psu_sv&v='+sv);poll();}catch(e){}}"
            // PSU voltage preset
            "async function psuset(v){"
            "try{await fetch('/control?cmd=psu_sv&v='+v);poll();}catch(e){}}"
            // PSU voltage keypad (browser prompt — simple, no flash overhead)
            "function psuKp(){"
            "const cur=D&&D.psu?D.psu.sv.toFixed(2):'';"
            "const s=prompt('Set voltage (4.00\\u201365.00 V):',cur);"
            "if(s!==null){const v=parseFloat(s);"
            "if(v>=4.0&&v<=65.0)psuset(v);}}"
            ));

        // ── JavaScript: API poll loop ─────────────────────────────────────────
        c.print(F(
            "async function poll(){"
            "try{"
            "const r=await fetch('/api',{cache:'no-store'});"
            "if(r.ok)paint(await r.json());"
            "}catch(ex){"
            "const dw=document.getElementById('dWiFi');"
            "if(dw)dw.className='dot off';"
            "}}"
            "poll();setInterval(poll,2000);"
            "</script></body></html>"));
    }
};