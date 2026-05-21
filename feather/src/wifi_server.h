#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "logger.h"
#include "channel.h"
#include "psu.h"

// =============================================================================
// WarmWiFiServer — non-blocking HTTP server.
// Endpoints:
//   GET  /          → redirect to /dash
//   GET  /dash      → live dashboard (channels + PSU panel)
//   GET  /api       → JSON telemetry (channels + PSU)
//   GET  /status    → plain-text status
//   GET  /log       → download CSV log
//   GET  /fftable   → FF table HTML viewer
//   GET  /diag      → heap / PSRAM / uptime diagnostics
//   GET  /control   → ch=N&cmd=enable|disable|sp_up|sp_dn
//                     cmd=psu_on|psu_off|psu_sp_up|psu_sp_dn|psu_sv&v=XX.XX
// =============================================================================

class WarmWiFiServer {
public:
    WarmWiFiServer()
        : _server(WIFI_HTTP_PORT), _ready(false),
          _logger(nullptr), _psu(nullptr),
          _ch1(nullptr), _ch2(nullptr),
          _reqCount(0), _lastHeartbeatMs(0) {}

    // Call once from main.cpp after WiFi.softAP() is confirmed up.
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

        // Channel helper lambda (written as a local struct for C++11 compat)
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

        // PSU block
        if (_psu) {
            // ok = false as soon as ANY error occurs (errStreak > 0), not just after
            // the full commsOk threshold. This means the dashboard shows --- and
            // NO COMM within ~2s of the first timeout rather than ~6s.
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

        // Metadata
        bool logOn = _logger && _logger->enabled();
        c.printf(",\"usb\":%s,\"logrows\":%d,\"logkb\":%d}",
                 logOn ? "true" : "false",
                 logOn ? _logger->rowsTotal() : 0,
                 logOn ? _logger->bufferUsed()/1024 : 0);
    }

    // ── /control — command endpoint ───────────────────────────────────────────
    void _serveControl(WiFiClient& c, const char* req) {
        _sendHeaders(c, 200, "application/json");

        // PSU commands
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

        // Channel commands
        const char* chPtr = strstr(req, "ch=");
        const char* cmdPtr = strstr(req, "cmd=");
        if (!chPtr || !cmdPtr) { c.print("{\"ok\":false}"); return; }

        int ch = atoi(chPtr + 3);
        HeatingChannel* hch = (ch == 1) ? _ch1 : (ch == 2) ? _ch2 : nullptr;
        if (!hch) { c.print("{\"ok\":false}"); return; }

        if (strstr(cmdPtr, "cmd=enable"))  hch->enabled = true;
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

    // ── /fftable — FF table viewer ────────────────────────────────────────────
    void _serveFFTable(WiFiClient& c) {
        _sendHeaders(c, 200, "text/html");
        c.print(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>FF Table</title>"
            "<style>body{font-family:monospace;background:#0d1117;color:#e6edf3;padding:1rem}"
            "table{border-collapse:collapse}td,th{padding:.2rem .6rem;border:1px solid #30363d}"
            "th{background:#151b23}tr:nth-child(even){background:#151b23}</style></head>"
            "<body><h2>Feedforward Learned Table</h2>"));
        c.print(F("<table><tr><th>Slot</th><th>Setpt (&deg;F)</th>"
            "<th>CH1 FF%</th><th>CH2 FF%</th></tr>"));
        for (int s = 0; s < FF_SLOTS; s++) {
            float sp = SETPOINT_MIN_F + s;
            int sl1 = (int)roundf(sp) - (int)SETPOINT_MIN_F;
            float ff1 = _ch1 ? _ch1->ffTerm() : 0;
            float ff2 = _ch2 ? _ch2->ffTerm() : 0;
            // Note: ffTerm() returns current-setpoint slot; for table view
            // we display slot values directly
            (void)sl1; (void)ff1; (void)ff2;
            c.printf("<tr><td>%d</td><td>%.0f</td>"
                     "<td>%.1f%%</td><td>%.1f%%</td></tr>",
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
            "<title>Diagnostics</title>"
            "<style>body{font-family:monospace;background:#0d1117;color:#e6edf3;padding:1rem}"
            "pre{background:#151b23;padding:.75rem;border-radius:6px}</style>"
            "</head><body><h2>&#x1F527; Diagnostics</h2><pre id='d'>Loading...</pre>"
            "<script>"
            "async function load(){"
            "const r=await fetch('/api');"
            "const d=await r.json();"
            "document.getElementById('d').textContent=JSON.stringify(d,null,2);}"
            "load();setInterval(load,3000);"
            "</script></body></html>"));
    }

    // ── /dash — main dashboard ────────────────────────────────────────────────
    void _serveDash(WiFiClient& c) {
        _sendHeaders(c, 200, "text/html");

        // ── HEAD + CSS ─────────────────────────────────────────────────────────
        c.print(F("<!DOCTYPE html><html><head>"
            "<meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>PSU Warmer</title><style>"
            "*{box-sizing:border-box;margin:0;padding:0}"
            "body{background:#0d1117;color:#e6edf3;font-family:-apple-system,"
            "BlinkMacSystemFont,'Segoe UI',sans-serif;display:flex;"
            "flex-direction:column;height:100vh;overflow:hidden}"
            ":root{--bg:#0d1117;--surf:#151b23;--card:#1a2030;--acc:#58a6ff;"
            "--grn:#3fb950;--red:#f85149;--amb:#e3b341;"
            "--txt:#e6edf3;--mut:#8b949e;--dim:#484f58;--bdr:#30363d}"
            "header{background:var(--surf);border-bottom:1px solid var(--bdr);"
            "padding:.55rem .9rem;display:flex;align-items:center;"
            "justify-content:space-between;flex-shrink:0}"
            "header h1{font-size:1rem;font-weight:600}"
            ".hicons{display:flex;align-items:center;gap:.75rem}"
            ".di{display:flex;flex-direction:column;align-items:center;gap:2px}"
            ".dot{width:8px;height:8px;border-radius:50%;background:var(--dim)}"
            ".dot.on{background:var(--grn)}.dot.off{background:var(--red)}"
            ".dlbl{font-size:.55rem;color:var(--mut)}"
            ".usbtn{padding:.3rem .7rem;border-radius:5px;background:var(--bdr);"
            "border:none;color:var(--txt);font-size:.8rem;cursor:pointer}"
            "nav{background:var(--surf);border-bottom:1px solid var(--bdr);"
            "display:flex;align-items:center;gap:.5rem;"
            "padding:.4rem .75rem;flex-shrink:0}"
            "nav a{color:var(--mut);font-size:.78rem;text-decoration:none;"
            "padding:.25rem .6rem;border-radius:4px}"
            "nav a.active{color:var(--acc);background:#1a1a2e}"
            "main{flex:1;overflow-y:auto;padding:1.25rem 1rem;"
            "display:flex;flex-direction:column;gap:.75rem;align-items:center}"
            ".ch-row{display:flex;gap:0;justify-content:center;width:100%;"
            "max-width:640px}"
            ".zone{width:320px;flex-shrink:0;flex-grow:0;display:flex;"
            "flex-direction:column;padding:.85rem;gap:.45rem;"
            "background:var(--card);border:1px solid var(--bdr)}"
            ".zone:first-child{border-radius:10px 0 0 10px}"
            ".zone:last-child{border-radius:0 10px 10px 0;border-left:none}"
            ".zhdr{display:flex;justify-content:space-between;align-items:center}"
            ".znm{font-size:.7rem;font-weight:700;letter-spacing:.1em;"
            "color:var(--mut);text-transform:uppercase}"
            ".onoff{padding:.35rem .9rem;border-radius:6px;font-size:.8rem;"
            "font-weight:700;border:none;cursor:pointer;min-width:56px}"
            ".onoff.on{background:#0e2e0e;color:var(--grn);border:1px solid #1a4a1a}"
            ".onoff.off{background:#1e1e2e;color:var(--mut);border:1px solid var(--bdr)}"
            ".temp{font-size:3.2rem;font-weight:700;text-align:center;"
            "color:var(--dim);line-height:1;padding:.15rem 0;flex-shrink:0}"
            ".temp.live{color:var(--acc)}"
            ".strow{display:flex;justify-content:space-between;align-items:center}"
            ".st{font-size:.8rem;font-weight:700;padding:.2rem .55rem;"
            "border-radius:4px;background:#0d1117;letter-spacing:.04em}"
            ".st.HEATING{color:#e06030}.st.HOLDING{color:#7fbb7f}"
            ".st.REDUCING{color:#4488cc}.st.IDLE{color:var(--mut)}"
            ".st.DISABLED{color:var(--dim)}.st.SENSERR,.st.CUTOFF{color:var(--red)}"
            ".dtdt{font-size:.7rem;color:var(--mut)}"
            ".barwrap{background:var(--bdr);border-radius:3px;height:12px;overflow:hidden}"
            ".barfill{height:100%;width:0%;border-radius:3px;"
            "transition:width .4s,background-color .4s}"
            ".brow{display:flex;align-items:center;gap:.5rem}"
            ".bpct{font-size:.75rem;color:var(--mut);white-space:nowrap;"
            "min-width:2.8rem;text-align:right}"
            ".div{height:1px;background:var(--bdr);margin:.1rem 0;flex-shrink:0}"
            ".tgt{display:flex;flex-direction:column;gap:.3rem;flex:1}"
            ".tgtlbl{font-size:.7rem;color:var(--mut);text-align:center}"
            ".tgtrow{display:flex;align-items:center;justify-content:center;gap:.4rem}"
            ".pm{width:44px;height:44px;border-radius:8px;background:#1a1e2e;"
            "border:1px solid var(--bdr);color:var(--txt);font-size:1.4rem;"
            "cursor:pointer;display:flex;align-items:center;justify-content:center}"
            ".pm:active{background:#252a40}"
            ".spval{font-size:1.1rem;font-weight:700;color:var(--txt);"
            "min-width:3rem;text-align:center}"
            ".ffrow{display:flex;justify-content:space-between;margin-top:auto}"
            ".fflbl{font-size:.65rem;color:var(--dim)}"));

        // ── PSU panel CSS ──────────────────────────────────────────────────────
        c.print(F(
            ".psu-panel{width:100%;max-width:640px;background:var(--card);"
            "border:1px solid var(--bdr);border-radius:10px;padding:.85rem;"
            "display:flex;flex-direction:column;gap:.55rem}"
            ".psu-hdr{display:flex;align-items:center;justify-content:space-between}"
            ".psu-title{font-size:.7rem;font-weight:700;letter-spacing:.1em;"
            "color:var(--mut);text-transform:uppercase}"
            ".psu-tog-btn{padding:.35rem 1rem;border-radius:6px;font-size:.82rem;"
            "font-weight:700;border:1px solid var(--bdr);cursor:pointer;"
            "min-width:76px;background:#1e1e2e;color:var(--mut)}"
            ".psu-tog-btn.on-cv{background:#0e2e0e;color:var(--grn);border-color:#1a4a1a}"
            ".psu-tog-btn.on-cc{background:#2e2000;color:var(--amb);border-color:#4a3000}"
            ".psu-tog-btn.fault{background:#2e0e0e;color:var(--red);border-color:#4a1a1a}"
            ".psu-metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:.4rem}"
            ".psu-metric{display:flex;flex-direction:column;align-items:center;"
            "background:#0d1117;border-radius:6px;padding:.65rem .2rem}"
            ".psu-val{font-size:3.4rem;font-weight:700;color:var(--txt);line-height:1}"
            ".psu-val.c-v{color:var(--grn)}"
            ".psu-val.c-a{color:var(--amb)}"
            ".psu-val.c-w{color:var(--red)}"
            ".psu-unit{font-size:.6rem;color:var(--mut);margin-top:2px}"
            ".psu-presets{display:grid;grid-template-columns:repeat(9,1fr);gap:.3rem}"
            ".pv-btn{background:#1a1e2e;border:1px solid var(--bdr);border-radius:4px;"
            "display:flex;align-items:center;justify-content:center;"
            "aspect-ratio:3/2;font-size:.75rem;color:var(--txt);cursor:pointer}"
            ".pv-btn:hover{border-color:var(--acc);color:var(--acc)}"
            ".pv-btn.active{background:#1a2a3a;border-color:var(--acc);color:var(--acc)}"
            ".psu-sv-row{display:flex;align-items:center;justify-content:center;gap:.4rem}"
            ".psu-svlbl{font-size:.7rem;color:var(--mut)}"
            ".psu-svval{font-size:1.1rem;font-weight:700;min-width:4rem;text-align:center}"
            ".pm-sm{width:36px;height:36px;border-radius:6px;background:#1a1e2e;"
            "border:1px solid var(--bdr);color:var(--txt);font-size:1rem;"
            "cursor:pointer;display:flex;align-items:center;justify-content:center}"
            ".pm-sm:active{background:#252a40}"
            "@media(max-width:500px){"
            ".ch-row{flex-direction:column;align-items:center}"
            ".zone{width:100%;max-width:380px;border-left:1px solid var(--bdr)}"
            ".zone:first-child{border-radius:10px 10px 0 0;border-bottom:none}"
            ".zone:last-child{border-radius:0 0 10px 10px;border-top:none}"
            ".psu-panel{max-width:380px}"
            ".psu-metrics{grid-template-columns:repeat(2,1fr)}"
            ".psu-presets{grid-template-columns:repeat(5,1fr)}}"
            "</style></head>"));

        // ── BODY: header + nav ─────────────────────────────────────────────────
        c.print(F("<body>"
            "<header><h1>&#x1F525; PSU Warmer</h1>"
            "<div class='hicons'>"
            "<div class='di'><div class='dot' id='dLog'></div>"
            "<span class='dlbl'>Log</span></div>"
            "<div class='di'><div class='dot' id='dWiFi'></div>"
            "<span class='dlbl'>WiFi</span></div>"
            "<div class='di'><div class='dot' id='dPsu'></div>"
            "<span class='dlbl'>PSU</span></div>"
            "<button class='usbtn' id='ubtn' onclick='toggleC()'>&#176;F</button>"
            "</div></header>"
            "<nav>"
            "<a href='/dash' class='active'>Dashboard</a>"
            "<a href='/fftable'>FF Table</a>"
            "<a href='/diag'>Diagnostics</a>"
            "<a href='/log'>&#x2B07; Log</a>"
            "<span style='font-size:.72rem;color:var(--dim);margin-left:.25rem'"
            " id='logstat'>---</span>"
            "</nav><main>"));

        // ── PSU panel (first) ──────────────────────────────────────────────────
        c.print(F(
            "<div class='psu-panel'>"
            "<div class='psu-hdr'>"
            "<span class='psu-title'>ZK-6522L Power Supply</span>"
            "<button class='psu-tog-btn' id='psu-tog' onclick='psutog()'>---</button>"
            "</div>"
            "<div class='psu-metrics'>"
            "<div class='psu-metric'><div class='psu-val c-v' id='pv'>---</div>"
            "<div class='psu-unit'>V OUT</div></div>"
            "<div class='psu-metric'><div class='psu-val c-a' id='pa'>---</div>"
            "<div class='psu-unit'>A OUT</div></div>"
            "<div class='psu-metric'><div class='psu-val c-w' id='pw'>---</div>"
            "<div class='psu-unit'>W</div></div>"
            "<div class='psu-metric'><div class='psu-val' id='pvin'>---</div>"
            "<div class='psu-unit'>V IN</div></div>"
            "</div>"
            "<div class='div'></div>"
            "<div class='psu-presets'>"
            "<div class='pv-btn' data-v='4'  onclick='psuset(4)'>4V</div>"
            "<div class='pv-btn' data-v='5'  onclick='psuset(5)'>5V</div>"
            "<div class='pv-btn' data-v='6'  onclick='psuset(6)'>6V</div>"
            "<div class='pv-btn' data-v='7'  onclick='psuset(7)'>7V</div>"
            "<div class='pv-btn' data-v='8'  onclick='psuset(8)'>8V</div>"
            "<div class='pv-btn' data-v='9'  onclick='psuset(9)'>9V</div>"
            "<div class='pv-btn' data-v='10' onclick='psuset(10)'>10V</div>"
            "<div class='pv-btn' data-v='11' onclick='psuset(11)'>11V</div>"
            "<div class='pv-btn' data-v='12' onclick='psuset(12)'>12V</div>"
            "</div>"
            "<div class='tgtlbl'>Voltage Target</div>"
            "<div class='psu-sv-row'>"
            "<div class='pm-sm' onclick='psusp(-0.5)'>&#8722;.5</div>"
            "<div class='pm-sm' onclick='psusp(-0.1)'>&#8722;.1</div>"
            "<span class='psu-svval' id='psv'>--.- V</span>"
            "<div class='pm-sm' onclick='psusp(0.1)'>+.1</div>"
            "<div class='pm-sm' onclick='psusp(0.5)'>+.5</div>"
            "</div>"
            "</div>")); // end psu-panel

        // ── Channel zones (second) ─────────────────────────────────────────────
        c.print(F("<div class='ch-row'>"));
        for (int z = 1; z <= 2; z++) {
            char za[1100], zb[1200];
            snprintf(za, sizeof(za),
                "<div class='zone' id='zone%d'>"
                "<div class='zhdr'><span class='znm'>CH%d</span>"
                "<button class='onoff off' id='tog%d'"
                " onclick='ctrl(%d,\"tog\")'>OFF</button></div>"
                "<div class='temp' id='t%d'>---</div>"
                "<div class='strow'>"
                "<span class='st IDLE' id='st%d'>---</span>"
                "<span class='dtdt' id='dt%d'></span></div>",
                z,z,z,z,z,z,z);
            snprintf(zb, sizeof(zb),
                "<div class='brow'>"
                "<div class='barwrap' style='flex:1'>"
                "<div class='barfill' id='bar%d'></div></div>"
                "<span class='bpct' id='bp%d'>0%%</span></div>"
                "<div class='div'></div>"
                "<div class='tgt'><div class='tgtlbl'>Battery Target</div>"
                "<div class='tgtrow'>"
                "<div class='pm' onclick='ctrl(%d,\"sp_dn\")'>&#8722;</div>"
                "<span class='spval' id='sp%d'>---</span>"
                "<div class='pm' onclick='ctrl(%d,\"sp_up\")'>&#43;</div>"
                "</div></div>"
                "<div class='ffrow'><span class='fflbl'>FF learned</span>"
                "<span class='fflbl' id='ff%d'>0%%</span></div>"
                "</div>",
                z,z,z,z,z,z);
            c.print(za); c.print(zb);
        }
        c.print(F("</div>")); // end ch-row

        // ── JavaScript ────────────────────────────────────────────────────────
        c.print(F("</main><script>"
            "let C=false,last=null;"
            "function tf(f){if(f<=-998)return'---';"
            "return C?((f-32)*5/9).toFixed(1)+'\\u00b0C':f.toFixed(1)+'\\u00b0F';}"
            "function tfSp(f){if(f<=-998)return'---';"
            "return C?Math.round((f-32)*5/9)+'\\u00b0C':Math.round(f)+'\\u00b0F';}"
            "function toggleC(){C=!C;"
            "document.getElementById('ubtn').textContent=C?'\\u00b0C':'\\u00b0F';"
            "paint(last);}"

            // Channel paint
            "function paintChannels(d){"
            "[1,2].forEach(n=>{"
            "const ch=d['ch'+n],en=ch.enabled,ok=ch.temp>-998;"
            "const tEl=document.getElementById('t'+n);"
            "tEl.textContent=tf(ch.temp);"
            "tEl.className='temp'+(en&&ok?' live':'');"
            "const stk=ch.status.replace(/ /g,'');"
            "const sEl=document.getElementById('st'+n);"
            "sEl.textContent=ch.status;sEl.className='st '+stk;"
            "document.getElementById('sp'+n).textContent=tfSp(ch.setpt);"
            "document.getElementById('bp'+n).textContent=ch.duty.toFixed(0)+'%';"
            "const bar=document.getElementById('bar'+n);"
            "bar.style.width=Math.min(ch.duty,100)+'%';"
            "bar.style.backgroundColor='hsl('+(120-ch.duty*1.2)+',85%,45%)';"
            "document.getElementById('ff'+n).textContent=ch.ff.toFixed(1)+'%';"
            "const tog=document.getElementById('tog'+n);"
            "tog.textContent=en?'ON':'OFF';"
            "tog.className='onoff '+(en?'on':'off');"
            "const rpm=ch.dtdt*60;"
            "document.getElementById('dt'+n).textContent="
            "Math.abs(rpm)>0.1?(rpm>0?'+':'')+rpm.toFixed(1)+'\\u00b0/m':'';});}"

            // PSU paint
            "function paintPsu(p){"
            "const tog=document.getElementById('psu-tog');"
            "if(!p||!p.ok){"
            "tog.textContent='NO COMM';tog.className='psu-tog-btn fault';"
            "['pv','pa','pw','pvin','psv'].forEach(id=>{document.getElementById(id).textContent='---';});"
            "document.querySelectorAll('.pv-btn').forEach(b=>b.classList.remove('active'));"
            "document.getElementById('dPsu').className='dot off';return;}"
            "document.getElementById('pv').textContent=p.v.toFixed(1);"
            "document.getElementById('pa').textContent=p.a.toFixed(1);"
            "document.getElementById('pw').textContent=p.w.toFixed(1);"
            "document.getElementById('pvin').textContent=p.vin.toFixed(1);"
            "document.getElementById('psv').textContent=p.sv.toFixed(2)+' V';"
            "if(p.prot){tog.textContent=p.status;tog.className='psu-tog-btn fault';}"
            "else if(!p.on){tog.textContent='OFF';tog.className='psu-tog-btn';}"
            "else if(p.cc){tog.textContent='ON \\u00b7 CC';tog.className='psu-tog-btn on-cc';}"
            "else{tog.textContent='ON \\u00b7 CV';tog.className='psu-tog-btn on-cv';}"
            "document.querySelectorAll('.pv-btn').forEach(b=>{"
            "b.classList.toggle('active',parseFloat(b.dataset.v)===p.sv);});"
            "document.getElementById('dPsu').className=p.ok?'dot on':'dot off';}"

            // Status indicators
            "function paintMeta(d){"
            "document.getElementById('dWiFi').className='dot on';"
            "const logEl=document.getElementById('dLog');"
            "if(logEl)logEl.className=d.usb?'dot on':'dot off';"
            "const ls=document.getElementById('logstat');"
            "if(ls)ls.textContent=d.usb?d.logrows+' rows':'logger off';}"

            // Main paint
            "function paint(d){if(!d)return;last=d;"
            "paintChannels(d);paintPsu(d.psu);paintMeta(d);}"

            // Channel control
            "async function ctrl(ch,cmd){"
            "try{const r=await fetch('/control?ch='+ch+'&cmd='+(cmd==='tog'?"
            "(last&&last['ch'+ch].enabled?'disable':'enable'):cmd));"
            "if(r.ok){const j=await r.json();if(j.ok)poll();}}catch(e){}}"

            // PSU output toggle
            "async function psutog(){"
            "const on=last&&last.psu&&last.psu.on;"
            "try{await fetch('/control?cmd='+(on?'psu_off':'psu_on'));"
            "poll();}catch(e){}}"

            // PSU setpoint bump
            "async function psusp(d){"
            "const sv=(last&&last.psu)?+(last.psu.sv+d).toFixed(2):12;"
            "try{await fetch('/control?cmd=psu_sv&v='+sv);poll();}catch(e){}}"

            // PSU quick preset
            "async function psuset(v){"
            "try{await fetch('/control?cmd=psu_sv&v='+v);poll();}catch(e){}}"

            // Poll loop
            "async function poll(){"
            "try{const r=await fetch('/api',{cache:'no-store'});"
            "if(r.ok)paint(await r.json());}catch(e){"
            "document.getElementById('dWiFi').className='dot off';}}"
            "poll();setInterval(poll,2000);"
            "</script></body></html>"));
    }
};