// =============================================================================
// main.cpp — PSU Warmer Display (Elecrow CrowPanel 5")
//
// 3-page LVGL UI: Dashboard | PSU | Warmer
// Receives JSON telemetry from Feather over UART0 (IO43=TX / IO44=RX)
// Sends commands back on touch events.
//
// UART wiring:
//   CrowPanel J10 TX (IO43) → Feather GPIO38 (RX)
//   CrowPanel J10 RX (IO44) → Feather GPIO39 (TX)
//   CrowPanel J10 GND       → Feather GND
//   CrowPanel J10 3V3       → leave open
//
// Telemetry format (200ms interval from Feather):
//   {"c1":{"t":99.3,"s":100.0,"d":23,"st":"HEATING","r":2.1},
//    "c2":{"t":98.1,"s":100.0,"d":12,"st":"HOLDING","r":0.2},
//    "psu":{"v":12.0,"a":1.5,"w":18.0,"vin":14.2,"sv":12.0,"on":1,"cc":0,"ok":1},
//    "env":{"t":72.3,"h":45.1,"p":1013.2,"g":125.4}}
//         t=°F  h=%RH  p=hPa  g=gas resistance kΩ (higher = cleaner air)
//
// Commands sent on touch:
//   {"cmd":"psu_sv","v":12.00}    {"cmd":"psu_on","v":1}
//   {"cmd":"c1_sp","v":102.0}     {"cmd":"c1_en","v":1}
//   {"cmd":"c2_sp","v":102.0}     {"cmd":"c2_en","v":1}
// =============================================================================

#include <Arduino.h>
#include <lvgl.h>
#include <Preferences.h>
#include "display_driver.h"

// Forward declaration for GT911 hardware drain (defined in display_driver.cpp)
void display_drain_touch();
void display_buzzer(bool on);
void display_speaker(bool on);

// ── UART ──────────────────────────────────────────────────────────────────────
// Feather link uses UART1-OUT connector (IO20-TX1 / IO19-RX1). Neither pin
// conflicts with display GPIOs (3,7,9-14,17,18,21,38-42,45-48).
// Serial (UART0, GPIO43/44, COM45 via USB-JTAG bridge) = debug output only.
// Feather side: CROWPANEL_RX_PIN=38 ← IO20(TX1), CROWPANEL_TX_PIN=39 → IO19(RX1).
#define FEATHER_BAUD    115200
#define FEATHER_TX_PIN  43     // IO43 = TX0 — shared with CH340K, debug prints go to Feather but are ignored
#define FEATHER_RX_PIN  44     // IO44 = RX0 — shared with CH340K, Feather telemetry received here
#define FeatherPort     Serial1

// ── Colour palette ────────────────────────────────────────────────────────────
#define C_BG          lv_color_hex(0x000000)
#define C_CARD        lv_color_hex(0x383838)
#define C_BORDER      lv_color_hex(0x444444)
#define C_BTN         lv_color_hex(0x585858)
#define C_BTNBDR      lv_color_hex(0x777777)
#define C_ACCENT      lv_color_hex(0xff8040)   // orange — CH labels only
#define C_GREEN       lv_color_hex(0x33cc66)
#define C_MUTED       lv_color_hex(0xd0d0d0)
#define C_TEXT        lv_color_hex(0xffffff)
#define C_DIM         lv_color_hex(0x909090)
#define C_RED         lv_color_hex(0xff4444)
#define C_BLUE        lv_color_hex(0x55aaff)
#define C_AMBER       lv_color_hex(0xe3b341)
#define C_BAR_BG      lv_color_hex(0x282828)
#define C_TILE_BG     lv_color_hex(0x282828)
#define C_HDR_BG      lv_color_hex(0x111111)
#define C_BTN_ON      lv_color_hex(0x1a3a1a)
#define C_BTN_CC      lv_color_hex(0x2e2000)
#define C_BTN_ERR     lv_color_hex(0x2e0e0e)
#define C_PRESET_ACT  lv_color_hex(0x0d1a2e)

// ── Screen & layout constants ─────────────────────────────────────────────────
#define SCR_W    800
#define SCR_H    480
#define HDR_H     50
#define PAD       10    // outer padding / card inner padding
#define GAP       16    // gap between rows on PSU page
#define CARD_R     8    // card corner radius

// Dashboard PSU strip
#define DS_H      76    // strip total height
#define DS_TH     56    // tile height within strip
#define DS_ON_W   88    // ON button width in strip (88 makes 5×128px tiles + 6px gaps divide evenly)
#define DS_TILE_G  6    // gap between strip tiles
#define DS_Y      (HDR_H + PAD)

// Dashboard channel cards
#define DC_Y      (DS_Y + DS_H + GAP)
#define DC_H      (SCR_H - DC_Y - PAD)
#define DC_W      ((SCR_W - PAD * 3) / 2)

// Warmer channel cards
#define WC_Y      (HDR_H + PAD)
#define WC_H      (SCR_H - WC_Y - PAD)
#define WC_W      DC_W

// PSU page vertical layout
#define PSU_TH    96     // readout tile height
#define PSU_TY    (HDR_H + PAD)
#define PSU_PY    (PSU_TY + PSU_TH + GAP)   // preset row Y
#define PSU_PH    52     // preset row height
#define PSU_AY    (PSU_PY + PSU_PH + GAP)   // adjust row Y
#define PSU_AH    80     // adjust row height
#define PSU_AB    60     // adjust button size (square)
#define PSU_OY    (PSU_AY + PSU_AH + GAP)   // ON button area Y
#define PSU_OH    (SCR_H - PSU_OY - PAD)    // ON button area height
#define PSU_OBH   70     // ON button height (centred in PSU_OH)
#define PSU_OBW   400    // ON button width (centred)

// ── Telemetry data structs ────────────────────────────────────────────────────
struct ChData {
    float tempF    = -999.f;
    float setptF   = 100.f;
    int   dutyPct  = 0;
    float dTdtFpm  = 0.f;          // °F per minute (field "r", optional)
    char  status[12] = "DISABLED";
    bool  valid    = false;
};

struct PsuData {
    float measV      = 0.f;
    float measA      = 0.f;
    float measW      = 0.f;
    float inputV     = 0.f;
    float setV       = 12.f;
    bool  on         = false;
    bool  cc         = false;
    bool  ok         = false;
    bool  received   = false;  // true once first valid packet has been parsed
    int   failStreak = 0;      // consecutive ok=false packets; go red only after 3
};

struct EnvData {
    float tempF    = 0.f;
    float humidity = 0.f;
    float pressHpa = 0.f;
    float gasKohm  = 0.f;
    bool  valid    = false;
};

static ChData  ch[2];
static PsuData psu;
static EnvData env;
static uint32_t _psuToggleMs = 0;   // debounce: ignore ON/OFF taps within 800ms

// ── Widget handle groups ──────────────────────────────────────────────────────
// Header
static lv_obj_t* _tabBtn[4];
static lv_obj_t* _envLbl;
static lv_obj_t* _brtSlider;       // brightness slider in header
static lv_obj_t* _brtPctLbl;       // "80%" label next to slider
static Preferences _prefs;         // NVS storage for brightness persistence

// Pages
static lv_obj_t* _pageDash;
static lv_obj_t* _pagePsu;
static lv_obj_t* _pageWarm;
static lv_obj_t* _pageEnv;

// Environment page widgets
static lv_obj_t* _envPageTempLbl;     // temperature value label
static lv_obj_t* _envPageTempArc;     // temperature arc gauge
static lv_obj_t* _envPageHumLbl;      // humidity value label
static lv_obj_t* _envPageHumArc;      // humidity arc gauge
static lv_obj_t* _envPagePresLbl;     // pressure value label
static lv_obj_t* _envPagePresArc;     // pressure arc gauge
static lv_obj_t* _envPageGasLbl;      // gas resistance value label
static lv_obj_t* _envPageIaqLbl;      // IAQ badge label
static lv_obj_t* _envPageIaqBadge;    // IAQ badge container (border color updated live)
static lv_obj_t* _envPageIaqBars[5];  // tier fill rects (Excellent→Bad)

// Dashboard — PSU strip
static lv_obj_t* _dPsuV, *_dPsuA, *_dPsuW, *_dPsuVin, *_dPsuSetV;
static lv_obj_t* _dPsuOnBtn, *_dPsuOnLbl;

// PSU page
static lv_obj_t* _psuVout, *_psuAout, *_psuWatt, *_psuVin;
static lv_obj_t* _psuSetVLbl;
static lv_obj_t* _psuPreset[9];
static lv_obj_t* _psuOnBtn, *_psuOnLbl;

static const int PRESET_V[9] = {4,5,6,7,8,9,10,11,12};

// Channel card widget bundle — used for both dashboard & warmer
struct ChCardW {
    lv_obj_t* temp;
    lv_obj_t* status;
    lv_obj_t* dtdt;
    lv_obj_t* bar;      // the fill rect inside the bar background
    lv_obj_t* duty;
    lv_obj_t* setpt;
    lv_obj_t* enBtn;
    lv_obj_t* enLbl;
};
static ChCardW _dcw[2];   // dashboard channel cards
static ChCardW _wcw[2];   // warmer channel cards

// ── JSON helpers ──────────────────────────────────────────────────────────────
static float jsonFloat(const char* json, const char* key) {
    char s[32]; snprintf(s, sizeof(s), "\"%s\":", key);
    const char* p = strstr(json, s);
    if (!p) return 0.f;
    p += strlen(s);
    while (*p == ' ') p++;
    return atof(p);
}
static int jsonInt(const char* json, const char* key) {
    return (int)jsonFloat(json, key);
}
static void jsonStr(const char* json, const char* key, char* out, int len) {
    char s[32]; snprintf(s, sizeof(s), "\"%s\":\"", key);
    const char* p = strstr(json, s);
    if (!p) { strncpy(out, "---", len); return; }
    p += strlen(s);
    int i = 0;
    while (*p && *p != '"' && i < len-1) out[i++] = *p++;
    out[i] = '\0';
}

// ── Command sender ────────────────────────────────────────────────────────────
static void sendCmd(const char* cmd, float v) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"%s\",\"v\":%.2f}\n", cmd, v);
    FeatherPort.print(buf);
    Serial.printf("[cmd] %s", buf);
}

// ── Telemetry parser ──────────────────────────────────────────────────────────
static void parseTelemetry(char* buf) {    // Channel objects
    const char* cKeys[2] = {"c1","c2"};
    for (int i = 0; i < 2; i++) {
        char key[8]; snprintf(key, sizeof(key), "\"%s\":{", cKeys[i]);
        const char* s = strstr(buf, key);
        if (!s) continue;
        const char* e = strchr(s + strlen(key), '}');
        if (!e) continue;
        char obj[160] = {};
        int l = (int)(e - s) + 1;
        if (l >= (int)sizeof(obj)) l = (int)sizeof(obj) - 1;
        strncpy(obj, s, l);
        ch[i].tempF   = jsonFloat(obj, "t");
        ch[i].setptF  = jsonFloat(obj, "s");
        ch[i].dutyPct = jsonInt(obj, "d");
        ch[i].dTdtFpm = jsonFloat(obj, "r");
        jsonStr(obj, "st", ch[i].status, sizeof(ch[i].status));
        // Only trust data when temperature is in a physically plausible range.
        // MAX31865 fault values come through as large negatives (e.g. -99, -200).
        // 0.0 means no data yet (default jsonFloat return).
        ch[i].valid = (ch[i].tempF > -40.f)   &&   // below -40°F = sensor fault
                      (ch[i].tempF != 0.f)     &&   // 0 = no data received yet
                      (ch[i].status[0] != '\0') &&
                      (strcmp(ch[i].status, "---") != 0);
    }
    // PSU object
    const char* ps = strstr(buf, "\"psu\":{");
    if (ps) {
        const char* e = strchr(ps + 7, '}');
        if (e) {
            char obj[200] = {};
            int l = (int)(e - ps) + 1;
            if (l >= (int)sizeof(obj)) l = (int)sizeof(obj) - 1;
            strncpy(obj, ps, l);
            psu.measV  = jsonFloat(obj, "v");
            psu.measA  = jsonFloat(obj, "a");
            psu.measW  = jsonFloat(obj, "w");
            psu.inputV = jsonFloat(obj, "vin");
            psu.setV   = jsonFloat(obj, "sv");
            psu.on     = jsonInt(obj, "on") != 0;
            psu.cc     = jsonInt(obj, "cc") != 0;
            psu.ok     = jsonInt(obj, "ok") != 0;
            psu.received = true;
            // Maintain hysteresis counter: only declare comm-loss after 3 in a row
            if (psu.ok) psu.failStreak = 0;
            else        psu.failStreak++;
        }
    }
    // Env object (optional — present when BME680 is wired)
    const char* es = strstr(buf, "\"env\":{");
    if (es) {
        const char* e = strchr(es + 7, '}');
        if (e) {
            char obj[80] = {};
            int l = (int)(e - es) + 1;
            if (l >= (int)sizeof(obj)) l = (int)sizeof(obj) - 1;
            strncpy(obj, es, l);
            env.tempF    = jsonFloat(obj, "t");
            env.humidity = jsonFloat(obj, "h");
            env.pressHpa = jsonFloat(obj, "p");
            env.gasKohm  = jsonFloat(obj, "g");
            env.valid    = true;
        }
    }
}

// ── Style helpers ─────────────────────────────────────────────────────────────
static void styleCard(lv_obj_t* o) {
    lv_obj_set_style_bg_color(o, C_CARD, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(o, C_BORDER, 0);
    lv_obj_set_style_border_width(o, 1, 0);
    lv_obj_set_style_radius(o, CARD_R, 0);
    lv_obj_set_style_pad_all(o, PAD, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}
static void styleTile(lv_obj_t* o) {
    lv_obj_set_style_bg_color(o, C_TILE_BG, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(o, C_BORDER, 0);
    lv_obj_set_style_border_width(o, 1, 0);
    lv_obj_set_style_radius(o, CARD_R, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}
static void styleBtn(lv_obj_t* o,
                     lv_color_t bg = C_BTN,
                     lv_color_t border = C_BTNBDR) {
    lv_obj_set_style_bg_color(o, bg, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(o, border, 0);
    lv_obj_set_style_border_width(o, 1, 0);
    lv_obj_set_style_radius(o, 6, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
}
static lv_obj_t* mkLbl(lv_obj_t* parent, const char* txt,
                         const lv_font_t* f, lv_color_t c) {
    lv_obj_t* l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, c, 0);
    return l;
}
static lv_color_t statusColor(const char* s) {
    if (strcmp(s, "HEATING")  == 0) return C_ACCENT;
    if (strcmp(s, "HOLDING")  == 0) return C_GREEN;
    if (strcmp(s, "REDUCING") == 0) return C_BLUE;
    if (strcmp(s, "CUTOFF")   == 0) return C_RED;
    if (strcmp(s, "SENS ERR") == 0) return C_RED;
    return C_DIM;
}
static lv_color_t dutyBarColor(int pct) {
    if (pct <= 33) {
        uint8_t t = (uint8_t)(pct * 255 / 33);
        return lv_color_make(t, 0xcc, (uint8_t)(0x44*(255-t)/255));
    } else if (pct <= 66) {
        uint8_t t = (uint8_t)((pct-33)*255/33);
        return lv_color_make(0xff, (uint8_t)(0xcc - (0x62*t/255)), 0);
    } else {
        uint8_t t = (uint8_t)((pct-66)*255/34);
        return lv_color_make(0xff, (uint8_t)(0x6a - (0x6a*t/255)), 0);
    }
}

// ── Page management ───────────────────────────────────────────────────────────
enum Page { PAGE_DASH=0, PAGE_PSU=1, PAGE_WARM=2, PAGE_ENV=3 };
static Page _curPage = PAGE_DASH;
static const char* TAB_LABELS[4] = {"Dashboard", "PSU", "Warmer", "Env"};

static void showPage(Page p) {
    _curPage = p;
    lv_obj_add_flag(_pageDash, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(_pagePsu,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(_pageWarm, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(_pageEnv,  LV_OBJ_FLAG_HIDDEN);
    switch (p) {
        case PAGE_DASH: lv_obj_clear_flag(_pageDash, LV_OBJ_FLAG_HIDDEN); break;
        case PAGE_PSU:  lv_obj_clear_flag(_pagePsu,  LV_OBJ_FLAG_HIDDEN); break;
        case PAGE_WARM: lv_obj_clear_flag(_pageWarm, LV_OBJ_FLAG_HIDDEN); break;
        case PAGE_ENV:  lv_obj_clear_flag(_pageEnv,  LV_OBJ_FLAG_HIDDEN); break;
    }
    for (int i = 0; i < 4; i++) {
        bool a = (i == (int)p);
        lv_obj_set_style_bg_color(_tabBtn[i],     a ? C_PRESET_ACT : C_BTN,    0);
        lv_obj_set_style_border_color(_tabBtn[i], a ? C_BLUE       : C_BTNBDR, 0);
        lv_obj_t* lbl = lv_obj_get_child(_tabBtn[i], 0);
        if (lbl) lv_obj_set_style_text_color(lbl, a ? C_BLUE : C_DIM, 0);
    }
}
static void onTabDash(lv_event_t*) { showPage(PAGE_DASH); }
static void onTabPsu (lv_event_t*) { showPage(PAGE_PSU);  }
static void onTabWarm(lv_event_t*) { showPage(PAGE_WARM); }
static void onTabEnv (lv_event_t*) { showPage(PAGE_ENV);  }

// ── Integer keypad (warmer setpoint, °F) ─────────────────────────────────────
static lv_obj_t* _kpModal   = nullptr;
static lv_obj_t* _kpDisplay = nullptr;
static char      _kpBuf[5]  = {};
static int       _kpLen     = 0;
static int       _kpCh      = 0;

static void kpClose() {
    if (_kpModal) { lv_obj_del(_kpModal); _kpModal = nullptr; _kpDisplay = nullptr; }
}
static void kpUpdateDisplay() {
    if (!_kpDisplay) return;
    if (_kpLen == 0) { lv_label_set_text(_kpDisplay, "---"); return; }
    char tmp[16]; snprintf(tmp, sizeof(tmp), "%s\xC2\xB0""F", _kpBuf);
    lv_label_set_text(_kpDisplay, tmp);
}
static void onKpBtn(lv_event_t* e) {
    char cmd = (char)(intptr_t)lv_event_get_user_data(e);
    if (cmd == 'D') {
        if (_kpLen > 0) { _kpBuf[--_kpLen] = '\0'; kpUpdateDisplay(); }
        return;
    }
    if (cmd == 'O') {
        if (_kpLen > 0) {
            int val = atoi(_kpBuf);
            if (val >= 50 && val <= 250) {
                sendCmd(_kpCh == 0 ? "c1_sp" : "c2_sp", (float)val);
                kpClose();
            } else {
                lv_obj_set_style_text_color(_kpDisplay, C_RED, 0);
                char err[24]; snprintf(err, sizeof(err), "%d\xC2\xB0""F out of range", val);
                lv_label_set_text(_kpDisplay, err);
            }
        }
        return;
    }
    if (_kpLen < 3) {
        _kpBuf[_kpLen++] = cmd; _kpBuf[_kpLen] = '\0';
        lv_obj_set_style_text_color(_kpDisplay, C_ACCENT, 0);
        kpUpdateDisplay();
    }
}
static void kpOpen(int chIdx) {
    if (_kpModal) kpClose();
    _kpCh = chIdx; _kpLen = 0; memset(_kpBuf, 0, sizeof(_kpBuf));

    _kpModal = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_kpModal, SCR_W, SCR_H);
    lv_obj_set_pos(_kpModal, 0, 0);
    lv_obj_set_style_bg_color(_kpModal, C_BG, 0);
    lv_obj_set_style_bg_opa(_kpModal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(_kpModal, 0, 0);
    lv_obj_set_style_radius(_kpModal, 0, 0);
    lv_obj_set_style_pad_all(_kpModal, 0, 0);
    lv_obj_clear_flag(_kpModal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(_kpModal, [](lv_event_t*){ kpClose(); }, LV_EVENT_CLICKED, nullptr);

    const int PW=340, PH=440;
    lv_obj_t* panel = lv_obj_create(_kpModal);
    lv_obj_set_size(panel, PW, PH);
    lv_obj_set_pos(panel, (SCR_W-PW)/2, (SCR_H-PH)/2);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x0e0e1e), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, C_BORDER, 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_radius(panel, CARD_R, 0);
    lv_obj_set_style_pad_all(panel, 12, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    char title[24]; snprintf(title, sizeof(title), "CH%d Target", _kpCh+1);
    lv_obj_t* tl = mkLbl(panel, title, &lv_font_montserrat_16, C_ACCENT);
    lv_obj_set_pos(tl, 0, 0);
    lv_obj_t* hl = mkLbl(panel, "50\xC2\xB0""F  to  250\xC2\xB0""F", &lv_font_montserrat_14, C_DIM);
    lv_obj_set_pos(hl, 0, 22);

    _kpDisplay = mkLbl(panel, "---", &lv_font_montserrat_40, C_ACCENT);
    lv_obj_set_width(_kpDisplay, PW-24);
    lv_obj_set_style_text_align(_kpDisplay, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(_kpDisplay, 0, 50);

    const int BH=52, BW=(PW-24-2*4)/3, GX=4, GY=110;
    const char* lbz[4][3] = {{"7","8","9"},{"4","5","6"},{"1","2","3"},{"DEL","0","APPLY"}};
    const char  cmz[4][3] = {{'7','8','9'},{'4','5','6'},{'1','2','3'},{'D',  '0','O'   }};
    for (int r=0; r<4; r++) for (int c=0; c<3; c++) {
        lv_color_t bg = (cmz[r][c]=='O') ? lv_color_hex(0x0a2a0a)
                      : (cmz[r][c]=='D') ? lv_color_hex(0x2a0a0a) : C_BTN;
        lv_color_t tc = (cmz[r][c]=='O') ? lv_color_hex(0x55ee55)
                      : (cmz[r][c]=='D') ? lv_color_hex(0xee5555) : C_TEXT;
        lv_obj_t* btn = lv_btn_create(panel);
        lv_obj_set_size(btn, BW, BH);
        lv_obj_set_pos(btn, c*(BW+GX), GY + r*(BH+GX));
        styleBtn(btn, bg, C_BTNBDR);
        lv_obj_add_event_cb(btn, onKpBtn, LV_EVENT_PRESSED, (void*)(intptr_t)cmz[r][c]);
        lv_obj_t* l = mkLbl(btn, lbz[r][c], &lv_font_montserrat_20, tc);
        lv_obj_center(l);
    }
}

// ── Decimal keypad (PSU setpoint, V) ─────────────────────────────────────────
static lv_obj_t* _dpModal   = nullptr;
static lv_obj_t* _dpDisplay = nullptr;
static char      _dpBuf[8]  = {};
static int       _dpLen     = 0;
static bool      _dpHasDot  = false;
static int       _dpDecDigs = 0;

static void dpClose() {
    if (_dpModal) { lv_obj_del(_dpModal); _dpModal = nullptr; _dpDisplay = nullptr; }
}
static void dpUpdateDisplay() {
    if (!_dpDisplay) return;
    if (_dpLen == 0) { lv_label_set_text(_dpDisplay, "--.- V"); return; }
    char tmp[16]; snprintf(tmp, sizeof(tmp), "%s V", _dpBuf);
    lv_label_set_text(_dpDisplay, tmp);
}
static void onDpBtn(lv_event_t* e) {
    char cmd = (char)(intptr_t)lv_event_get_user_data(e);
    if (cmd == 'D') {
        if (_dpLen > 0) {
            if (_dpBuf[_dpLen-1] == '.') { _dpHasDot = false; _dpDecDigs = 0; }
            else if (_dpHasDot) _dpDecDigs--;
            _dpBuf[--_dpLen] = '\0';
            dpUpdateDisplay();
        }
        return;
    }
    if (cmd == '.') {
        if (!_dpHasDot && _dpLen > 0 && _dpLen < 5) {
            _dpBuf[_dpLen++] = '.'; _dpBuf[_dpLen] = '\0';
            _dpHasDot = true; dpUpdateDisplay();
        }
        return;
    }
    if (cmd == 'O') {
        if (_dpLen > 0) {
            float val = atof(_dpBuf);
            if (val >= 4.0f && val <= 65.0f) {
                sendCmd("psu_sv", val); dpClose();
            } else {
                lv_obj_set_style_text_color(_dpDisplay, C_RED, 0);
                lv_label_set_text(_dpDisplay, "Out of range");
            }
        }
        return;
    }
    if (_dpLen >= 6) return;
    if (_dpHasDot && _dpDecDigs >= 2) return;
    _dpBuf[_dpLen++] = cmd; _dpBuf[_dpLen] = '\0';
    if (_dpHasDot) _dpDecDigs++;
    lv_obj_set_style_text_color(_dpDisplay, C_TEXT, 0);
    dpUpdateDisplay();
}
static void dpOpen() {
    if (_dpModal) dpClose();
    _dpLen = 0; _dpHasDot = false; _dpDecDigs = 0;
    memset(_dpBuf, 0, sizeof(_dpBuf));

    _dpModal = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_dpModal, SCR_W, SCR_H);
    lv_obj_set_pos(_dpModal, 0, 0);
    lv_obj_set_style_bg_color(_dpModal, C_BG, 0);
    lv_obj_set_style_bg_opa(_dpModal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(_dpModal, 0, 0);
    lv_obj_set_style_radius(_dpModal, 0, 0);
    lv_obj_set_style_pad_all(_dpModal, 0, 0);
    lv_obj_clear_flag(_dpModal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(_dpModal, [](lv_event_t*){ dpClose(); }, LV_EVENT_CLICKED, nullptr);

    const int PW=340, PH=420;
    lv_obj_t* panel = lv_obj_create(_dpModal);
    lv_obj_set_size(panel, PW, PH);
    lv_obj_set_pos(panel, (SCR_W-PW)/2, (SCR_H-PH)/2);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x0e0e1e), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, C_BORDER, 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_radius(panel, CARD_R, 0);
    lv_obj_set_style_pad_all(panel, 12, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* tl = mkLbl(panel, "Set Voltage", &lv_font_montserrat_16, C_BLUE);
    lv_obj_set_pos(tl, 0, 0);
    lv_obj_t* hl = mkLbl(panel, "4.00 V  to  65.00 V", &lv_font_montserrat_14, C_DIM);
    lv_obj_set_pos(hl, 0, 22);

    _dpDisplay = mkLbl(panel, "--.- V", &lv_font_montserrat_40, C_TEXT);
    lv_obj_set_width(_dpDisplay, PW-24);
    lv_obj_set_style_text_align(_dpDisplay, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(_dpDisplay, 0, 50);

    const int BH=52, BW=(PW-24-2*4)/3, GX=4, GY=110;
    const char* lbz[4][3] = {{"7","8","9"},{"4","5","6"},{"1","2","3"},{"DEL","0","."}};
    const char  cmz[4][3] = {{'7','8','9'},{'4','5','6'},{'1','2','3'},{'D', '0','.'}};
    for (int r=0; r<4; r++) for (int c=0; c<3; c++) {
        lv_color_t bg = (cmz[r][c]=='D') ? lv_color_hex(0x2a0a0a) : C_BTN;
        lv_color_t tc = (cmz[r][c]=='D') ? lv_color_hex(0xee5555) : C_TEXT;
        lv_obj_t* btn = lv_btn_create(panel);
        lv_obj_set_size(btn, BW, BH);
        lv_obj_set_pos(btn, c*(BW+GX), GY + r*(BH+GX));
        styleBtn(btn, bg, C_BTNBDR);
        lv_obj_add_event_cb(btn, onDpBtn, LV_EVENT_PRESSED, (void*)(intptr_t)cmz[r][c]);
        lv_obj_t* l = mkLbl(btn, lbz[r][c], &lv_font_montserrat_20, tc);
        lv_obj_center(l);
    }
    // APPLY — full width
    lv_obj_t* applyBtn = lv_btn_create(panel);
    lv_obj_set_size(applyBtn, PW-24, BH);
    lv_obj_set_pos(applyBtn, 0, GY + 4*(BH+GX));
    styleBtn(applyBtn, lv_color_hex(0x0a2a0a), C_GREEN);
    lv_obj_add_event_cb(applyBtn, onDpBtn, LV_EVENT_PRESSED, (void*)(intptr_t)'O');
    lv_obj_t* al = mkLbl(applyBtn, "APPLY", &lv_font_montserrat_20, lv_color_hex(0x55ee55));
    lv_obj_center(al);
}

// ── Header ────────────────────────────────────────────────────────────────────
static void buildHeader() {
    lv_obj_t* hdr = lv_obj_create(lv_scr_act());
    lv_obj_set_size(hdr, SCR_W, HDR_H);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, C_HDR_BG, 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_shadow_width(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    // Title (left)
    lv_obj_t* title = mkLbl(hdr, "PSU Warmer", &lv_font_montserrat_24, C_ACCENT);
    lv_obj_set_pos(title, PAD, (HDR_H - 24) / 2);

    // Tab buttons (centred) — 4 tabs × 96px + 3 gaps × 8px = 408px total
    const int TAB_W=96, TAB_H=36, TAB_G=8;
    const int TAB_X0 = (SCR_W - (TAB_W*4 + TAB_G*3)) / 2;
    lv_event_cb_t cbs[4] = {onTabDash, onTabPsu, onTabWarm, onTabEnv};
    for (int i = 0; i < 4; i++) {
        lv_obj_t* btn = lv_btn_create(hdr);
        lv_obj_set_size(btn, TAB_W, TAB_H);
        lv_obj_set_pos(btn, TAB_X0 + i*(TAB_W+TAB_G), (HDR_H-TAB_H)/2);
        styleBtn(btn);
        lv_obj_add_event_cb(btn, cbs[i], LV_EVENT_PRESSED, nullptr);
        lv_obj_t* l = mkLbl(btn, TAB_LABELS[i], &lv_font_montserrat_14, C_DIM);
        lv_obj_center(l);
        _tabBtn[i] = btn;
    }

    // Env text — plain white, right-aligned, same padding as title on left
    _envLbl = mkLbl(hdr, "---", &lv_font_montserrat_24, C_TEXT);
    lv_obj_set_width(_envLbl, 220);
    lv_obj_set_style_text_align(_envLbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(_envLbl, SCR_W - 220 - PAD, (HDR_H - 24) / 2);
}

// ── Dashboard — PSU strip ─────────────────────────────────────────────────────
static void buildDashPsuStrip(lv_obj_t* parent) {
    lv_obj_t* card = lv_obj_create(parent);
    lv_obj_set_size(card, SCR_W - PAD*2, DS_H);
    lv_obj_set_pos(card, PAD, DS_Y);
    styleCard(card);
    lv_obj_set_style_pad_all(card, 0, 0);  // manual placement inside

    const int IW  = SCR_W - PAD*2 - 2;    // inner width (subtract border)
    const int ON_W = DS_ON_W;
    // Uniform spacing S between every adjacent element AND card edges.
    // 7 gaps × 10 + 5 × 124 + 88 = 778 = IW  ✓  — no rounding, no asymmetry.
    const int S  = 10;
    const int TW = (IW - 7*S - ON_W) / 5;  // 124px per tile

    const char*  units[5] = {"V OUT","A OUT","WATTS","V IN","SET"};
    lv_color_t   cols[5]  = {C_GREEN, C_ACCENT, C_RED, C_MUTED, C_MUTED};
    lv_obj_t**   ptrs[5]  = {&_dPsuV, &_dPsuA, &_dPsuW, &_dPsuVin, &_dPsuSetV};
    const int TY = (DS_H - DS_TH) / 2;

    lv_obj_t* setTile = nullptr;
    for (int t = 0; t < 5; t++) {
        int tx = S + t*(TW + S);
        lv_obj_t* tile = lv_obj_create(card);
        lv_obj_set_size(tile, TW, DS_TH);
        lv_obj_set_pos(tile, tx, TY);
        styleTile(tile);
        *ptrs[t] = mkLbl(tile, "---", &lv_font_montserrat_32, cols[t]);
        lv_obj_align(*ptrs[t], LV_ALIGN_CENTER, 0, -8);
        lv_obj_t* ul = mkLbl(tile, units[t], &lv_font_montserrat_12, C_DIM);
        lv_obj_align(ul, LV_ALIGN_BOTTOM_MID, 0, -4);
        if (t == 4) setTile = tile;
    }
    // SET tile is tappable — opens decimal keypad
    if (setTile) {
        lv_obj_set_style_border_color(setTile, lv_color_hex(0x3366aa), 0);
        lv_obj_add_flag(setTile, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(setTile, [](lv_event_t*){ dpOpen(); }, LV_EVENT_PRESSED, nullptr);
    }

    // ON/OFF button — gap to tile[4] = gap to right card edge = S = 10px
    int on_x = S + 5*(TW + S);   // = 680, right gap = 778-680-88 = 10 = S ✓
    _dPsuOnBtn = lv_btn_create(card);
    lv_obj_set_size(_dPsuOnBtn, ON_W, DS_TH);
    lv_obj_set_pos(_dPsuOnBtn, on_x, TY);
    styleBtn(_dPsuOnBtn);
    lv_obj_set_style_radius(_dPsuOnBtn, 6, 0);
    lv_obj_add_event_cb(_dPsuOnBtn, [](lv_event_t*) {
        uint32_t now = millis();
        if (now - _psuToggleMs < 800) return;
        _psuToggleMs = now;
        sendCmd("psu_on", psu.on ? 0.f : 1.f);
    }, LV_EVENT_CLICKED, nullptr);
    _dPsuOnLbl = mkLbl(_dPsuOnBtn, "---", &lv_font_montserrat_12, C_DIM);
    lv_obj_center(_dPsuOnLbl);
}

// ── Channel card ──────────────────────────────────────────────────────────────
static void buildChCard(lv_obj_t* parent, int x, int y, int w, int h,
                         int chIdx, bool fullSize, ChCardW& ww) {
    void* ud = (void*)(intptr_t)chIdx;

    lv_obj_t* card = lv_obj_create(parent);
    lv_obj_set_size(card, w, h);
    lv_obj_set_pos(card, x, y);
    styleCard(card);

    const int IW = w - PAD*2;
    const int IH = h - PAD*2;

    // CH# label
    char nm[4]; snprintf(nm, sizeof(nm), "CH%d", chIdx+1);
    lv_obj_t* nmLbl = mkLbl(card, nm, &lv_font_montserrat_16, C_ACCENT);
    lv_obj_set_pos(nmLbl, 0, 2);

    // Enable button — compact, top-right
    const int EN_W=68, EN_H=30;
    lv_obj_t* enBtn = lv_btn_create(card);
    lv_obj_set_size(enBtn, EN_W, EN_H);
    lv_obj_set_pos(enBtn, IW-EN_W, 0);
    styleBtn(enBtn);
    lv_obj_set_user_data(enBtn, ud);
    lv_obj_add_event_cb(enBtn, [](lv_event_t* e) {
        int i = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
        bool en = strcmp(ch[i].status, "DISABLED") != 0;
        sendCmd(i==0?"c1_en":"c2_en", en ? 0.f : 1.f);
    }, LV_EVENT_PRESSED, nullptr);
    lv_obj_t* enLbl = mkLbl(enBtn, "OFF", &lv_font_montserrat_14, C_DIM);
    lv_obj_center(enLbl);
    ww.enBtn = enBtn;  ww.enLbl = enLbl;

    const lv_font_t* tempFont = &lv_font_montserrat_48;   // large on both pages
    const int tempH   = 68;
    const int barH    = fullSize ? 12 : 10;
    const int spBtnSz = fullSize ? 70 : 44;
    const lv_font_t* spFont  = fullSize ? &lv_font_montserrat_32 : &lv_font_montserrat_24;
    const int        spFontH = fullSize ? 32 : 24;

    // Distribute remaining vertical space evenly across 7 inter-section gaps
    // so the card always fills top-to-bottom with a natural bottom margin.
    const int FIXED = 32 + tempH + 18 + barH + 18 + 1 + 14 + spBtnSz;
    const int vpad  = (IH - FIXED) / 7;   // pixels per gap (integer)

    const int hdrH  = 32;
    const int tempY = hdrH + vpad;
    const int statY = tempY + tempH + vpad;
    const int barY  = statY + 18 + vpad;
    const int dutyY = barY  + barH + vpad/2;
    const int divY  = dutyY + 18   + vpad;
    const int tgtY  = divY  + 1    + vpad/2;
    const int spY   = tgtY  + 14   + vpad/2;
    // Temperature — big display
    lv_obj_t* tempLbl = mkLbl(card, "---", tempFont, C_TEXT);
    lv_obj_set_width(tempLbl, IW);
    lv_obj_set_style_text_align(tempLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(tempLbl, 0, tempY);
    ww.temp = tempLbl;

    // Status + dTdt on same row
    lv_obj_t* statLbl = mkLbl(card, "---", &lv_font_montserrat_14, C_DIM);
    lv_obj_set_pos(statLbl, 0, statY);
    ww.status = statLbl;

    lv_obj_t* dtLbl = mkLbl(card, "", &lv_font_montserrat_14, C_DIM);
    lv_obj_set_width(dtLbl, IW);
    lv_obj_set_style_text_align(dtLbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(dtLbl, 0, statY);
    ww.dtdt = dtLbl;

    // Duty bar
    lv_obj_t* barBg = lv_obj_create(card);
    lv_obj_set_size(barBg, IW, barH);
    lv_obj_set_pos(barBg, 0, barY);
    lv_obj_set_style_bg_color(barBg, C_BAR_BG, 0);
    lv_obj_set_style_bg_opa(barBg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(barBg, 0, 0);
    lv_obj_set_style_radius(barBg, 4, 0);
    lv_obj_set_style_pad_all(barBg, 0, 0);   // no padding — fill child must reach edges
    lv_obj_clear_flag(barBg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* bar = lv_obj_create(barBg);
    lv_obj_set_size(bar, 0, barH);
    lv_obj_set_pos(bar, 0, 0);
    lv_obj_set_style_bg_color(bar, C_ACCENT, 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_radius(bar, 4, 0);
    ww.bar = bar;

    // Duty % label (right-aligned)
    lv_obj_t* dutyLbl = mkLbl(card, "0%", &lv_font_montserrat_14, C_DIM);
    lv_obj_set_width(dutyLbl, IW);
    lv_obj_set_style_text_align(dutyLbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(dutyLbl, 0, dutyY);
    ww.duty = dutyLbl;

    // Divider
    lv_obj_t* div = lv_obj_create(card);
    lv_obj_set_size(div, IW, 1);
    lv_obj_set_pos(div, 0, divY);
    lv_obj_set_style_bg_color(div, C_BORDER, 0);
    lv_obj_set_style_bg_opa(div, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(div, 0, 0);
    lv_obj_set_style_radius(div, 0, 0);

    // TARGET label
    lv_obj_t* tgtLbl = mkLbl(card, "TARGET", &lv_font_montserrat_12, C_DIM);
    lv_obj_set_pos(tgtLbl, 0, tgtY);

    // Setpoint row: [−] [setpoint value — tappable] [+]
    lv_obj_t* spDn = lv_btn_create(card);
    lv_obj_set_size(spDn, spBtnSz, spBtnSz);
    lv_obj_set_pos(spDn, 0, spY);
    styleBtn(spDn);
    lv_obj_set_user_data(spDn, ud);
    lv_obj_add_event_cb(spDn, [](lv_event_t* e) {
        int i = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
        sendCmd(i==0?"c1_sp":"c2_sp", ch[i].setptF - 1.0f);
    }, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(spDn, [](lv_event_t* e) {
        int i = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
        sendCmd(i==0?"c1_sp":"c2_sp", ch[i].setptF - 1.0f);
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);
    lv_obj_t* dnl = mkLbl(spDn, LV_SYMBOL_MINUS, spFont, C_TEXT);
    lv_obj_center(dnl);

    lv_obj_t* spLbl = mkLbl(card, "---", spFont, C_TEXT);
    lv_obj_set_pos(spLbl, spBtnSz+6, spY + (spBtnSz - spFontH)/2);
    lv_obj_set_width(spLbl, IW - spBtnSz*2 - 12);
    lv_obj_set_style_text_align(spLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(spLbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_user_data(spLbl, ud);
    lv_obj_add_event_cb(spLbl, [](lv_event_t* e) {
        int i = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
        kpOpen(i);
    }, LV_EVENT_PRESSED, nullptr);
    ww.setpt = spLbl;

    lv_obj_t* spUp = lv_btn_create(card);
    lv_obj_set_size(spUp, spBtnSz, spBtnSz);
    lv_obj_set_pos(spUp, IW-spBtnSz, spY);
    styleBtn(spUp);
    lv_obj_set_user_data(spUp, ud);
    lv_obj_add_event_cb(spUp, [](lv_event_t* e) {
        int i = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
        sendCmd(i==0?"c1_sp":"c2_sp", ch[i].setptF + 1.0f);
    }, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(spUp, [](lv_event_t* e) {
        int i = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
        sendCmd(i==0?"c1_sp":"c2_sp", ch[i].setptF + 1.0f);
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);
    lv_obj_t* upl = mkLbl(spUp, LV_SYMBOL_PLUS, spFont, C_TEXT);
    lv_obj_center(upl);
}

// ── Dashboard page ────────────────────────────────────────────────────────────
static void buildDashboardPage() {
    _pageDash = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_pageDash, SCR_W, SCR_H);
    lv_obj_set_pos(_pageDash, 0, 0);
    lv_obj_set_style_bg_opa(_pageDash, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_pageDash, 0, 0);
    lv_obj_set_style_pad_all(_pageDash, 0, 0);
    lv_obj_set_style_shadow_width(_pageDash, 0, 0);
    lv_obj_clear_flag(_pageDash, LV_OBJ_FLAG_SCROLLABLE);

    buildDashPsuStrip(_pageDash);

    for (int i = 0; i < 2; i++)
        buildChCard(_pageDash, PAD + i*(DC_W+PAD), DC_Y, DC_W, DC_H, i, false, _dcw[i]);
}

// ── PSU page ──────────────────────────────────────────────────────────────────
static void buildPsuPage() {
    _pagePsu = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_pagePsu, SCR_W, SCR_H);
    lv_obj_set_pos(_pagePsu, 0, 0);
    lv_obj_set_style_bg_opa(_pagePsu, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_pagePsu, 0, 0);
    lv_obj_set_style_pad_all(_pagePsu, 0, 0);
    lv_obj_set_style_shadow_width(_pagePsu, 0, 0);
    lv_obj_clear_flag(_pagePsu, LV_OBJ_FLAG_SCROLLABLE);

    const int IW = SCR_W - PAD*2;   // 780px

    // ── Readout tiles (4) ────────────────────────────────────────────────────
    const char*  tunits[4] = {"V OUT","A OUT","WATTS","V IN"};
    lv_color_t   tcols[4]  = {C_GREEN, C_ACCENT, C_RED, C_MUTED};
    lv_obj_t**   tptrs[4]  = {&_psuVout, &_psuAout, &_psuWatt, &_psuVin};
    const int TW = (IW - GAP*3) / 4;
    for (int t = 0; t < 4; t++) {
        lv_obj_t* tile = lv_obj_create(_pagePsu);
        lv_obj_set_size(tile, TW, PSU_TH);
        lv_obj_set_pos(tile, PAD + t*(TW+GAP), PSU_TY);
        styleTile(tile);
        *tptrs[t] = mkLbl(tile, "---", &lv_font_montserrat_48, tcols[t]);
        lv_obj_align(*tptrs[t], LV_ALIGN_CENTER, 0, -8);
        lv_obj_t* ul = mkLbl(tile, tunits[t], &lv_font_montserrat_12, C_DIM);
        lv_obj_align(ul, LV_ALIGN_BOTTOM_MID, 0, -4);
    }

    // ── Preset buttons (9: 4V–12V) ───────────────────────────────────────────
    const int PW = (IW - 8*8) / 9;   // 8 gaps of 8px between 9 buttons
    for (int i = 0; i < 9; i++) {
        _psuPreset[i] = lv_btn_create(_pagePsu);
        lv_obj_set_size(_psuPreset[i], PW, PSU_PH);
        lv_obj_set_pos(_psuPreset[i], PAD + i*(PW+8), PSU_PY);
        styleBtn(_psuPreset[i]);
        lv_obj_set_style_radius(_psuPreset[i], 6, 0);
        char lbl[5]; snprintf(lbl, sizeof(lbl), "%dV", PRESET_V[i]);
        lv_obj_t* pl = mkLbl(_psuPreset[i], lbl, &lv_font_montserrat_16, C_DIM);
        lv_obj_center(pl);
        lv_obj_set_user_data(_psuPreset[i], (void*)(intptr_t)i);
        lv_obj_add_event_cb(_psuPreset[i], [](lv_event_t* e) {
            int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
            sendCmd("psu_sv", (float)PRESET_V[idx]);
        }, LV_EVENT_PRESSED, nullptr);
    }

    // ── Setpoint adjust row ──────────────────────────────────────────────────
    // Layout: [btn0.5-] [btn0.1-] [SET POINT box] [btn0.1+] [btn0.5+]
    const int AB  = PSU_AB;    // 60px square buttons
    const int SVW = IW - 4*AB - 4*GAP;  // set-voltage box width
    const int ABY = PSU_AY + (PSU_AH - AB) / 2;  // vert-centred in row

    struct { const char* num; const char* sym; float d; } bumps[4] = {
        {"0.5","-",-0.5f}, {"0.1","-",-0.1f}, {"0.1","+",0.1f}, {"0.5","+",0.5f}
    };
    int bx[4];
    bx[0] = PAD;
    bx[1] = PAD + AB + GAP;
    bx[2] = PAD + 2*AB + 3*GAP + SVW;
    bx[3] = PAD + 3*AB + 4*GAP + SVW;

    for (int b = 0; b < 4; b++) {
        float* dp = new float(bumps[b].d);
        lv_obj_t* btn = lv_btn_create(_pagePsu);
        lv_obj_set_size(btn, AB, AB);
        lv_obj_set_pos(btn, bx[b], ABY);
        styleBtn(btn);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_add_event_cb(btn, [](lv_event_t* e) {
            float d = *(float*)lv_event_get_user_data(e);
            sendCmd("psu_sv", psu.setV + d);
        }, LV_EVENT_PRESSED, dp);
        // Number on top, symbol below
        lv_obj_t* nl = mkLbl(btn, bumps[b].num, &lv_font_montserrat_16, C_TEXT);
        lv_obj_set_pos(nl, 0, 8);
        lv_obj_set_width(nl, AB);
        lv_obj_set_style_text_align(nl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_t* sl = mkLbl(btn, bumps[b].sym, &lv_font_montserrat_20, C_DIM);
        lv_obj_set_pos(sl, 0, AB - 28);
        lv_obj_set_width(sl, AB);
        lv_obj_set_style_text_align(sl, LV_TEXT_ALIGN_CENTER, 0);
    }

    // Set-voltage display box (tappable → decimal keypad)
    int svx = PAD + 2*AB + 2*GAP;
    lv_obj_t* svBox = lv_obj_create(_pagePsu);
    lv_obj_set_size(svBox, SVW, PSU_AH);
    lv_obj_set_pos(svBox, svx, PSU_AY);
    styleTile(svBox);
    lv_obj_add_flag(svBox, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(svBox, [](lv_event_t*){ dpOpen(); }, LV_EVENT_PRESSED, nullptr);

    lv_obj_t* spLbl = mkLbl(svBox, "SET POINT", &lv_font_montserrat_12, C_DIM);
    lv_obj_align(spLbl, LV_ALIGN_TOP_MID, 0, 8);
    _psuSetVLbl = mkLbl(svBox, "--.- V", &lv_font_montserrat_32, C_TEXT);
    lv_obj_align(_psuSetVLbl, LV_ALIGN_CENTER, 0, 8);

    // ── OUTPUT ON/OFF button ─────────────────────────────────────────────────
    const int onY = PSU_OY + (PSU_OH - PSU_OBH) / 2;
    const int onX = (SCR_W - PSU_OBW) / 2;
    _psuOnBtn = lv_btn_create(_pagePsu);
    lv_obj_set_size(_psuOnBtn, PSU_OBW, PSU_OBH);
    lv_obj_set_pos(_psuOnBtn, onX, onY);
    styleBtn(_psuOnBtn);
    lv_obj_set_style_radius(_psuOnBtn, 8, 0);
    lv_obj_add_event_cb(_psuOnBtn, [](lv_event_t*) {
        uint32_t now = millis();
        if (now - _psuToggleMs < 800) return;
        _psuToggleMs = now;
        sendCmd("psu_on", psu.on ? 0.f : 1.f);
    }, LV_EVENT_CLICKED, nullptr);
    _psuOnLbl = mkLbl(_psuOnBtn, "---", &lv_font_montserrat_20, C_DIM);
    lv_obj_center(_psuOnLbl);
}

// ── Warmer page ───────────────────────────────────────────────────────────────
static void buildWarmerPage() {
    _pageWarm = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_pageWarm, SCR_W, SCR_H);
    lv_obj_set_pos(_pageWarm, 0, 0);
    lv_obj_set_style_bg_opa(_pageWarm, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_pageWarm, 0, 0);
    lv_obj_set_style_pad_all(_pageWarm, 0, 0);
    lv_obj_set_style_shadow_width(_pageWarm, 0, 0);
    lv_obj_clear_flag(_pageWarm, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < 2; i++)
        buildChCard(_pageWarm, PAD + i*(WC_W+PAD), WC_Y, WC_W, WC_H, i, true, _wcw[i]);
}

// ── UI update ─────────────────────────────────────────────────────────────────
// ── Label update helper ───────────────────────────────────────────────────────
// Only calls lv_label_set_text when the text actually changes.
// Unchanged text → no dirty region → no LVGL flush → no PSRAM collision.
// This is the primary defence against jitter caused by constant redraws.
static inline void setLbl(lv_obj_t* lbl, const char* txt) {
    if (lbl && strcmp(lv_label_get_text(lbl), txt) != 0)
        lv_label_set_text(lbl, txt);
}

static void updateChCard(ChCardW& ww, int i) {
    if (!ch[i].valid) return;
    char buf[32];
    ChData& c = ch[i];

    if (c.tempF <= -40.f || c.tempF == 0.f) {
        setLbl(ww.temp, "---");
        lv_obj_set_style_text_color(ww.temp, C_DIM, 0);
    } else {
        snprintf(buf, sizeof(buf), "%.1f\xC2\xB0""F", c.tempF);
        setLbl(ww.temp, buf);
        lv_obj_set_style_text_color(ww.temp, C_TEXT, 0);
    }

    setLbl(ww.status, c.status);
    lv_obj_set_style_text_color(ww.status, statusColor(c.status), 0);

    if (fabsf(c.dTdtFpm) >= 0.1f) {
        snprintf(buf, sizeof(buf), "%+.1f\xC2\xB0""/m", c.dTdtFpm);
        setLbl(ww.dtdt, buf);
    } else {
        setLbl(ww.dtdt, "");
    }

    int bw = (int)(lv_obj_get_width(lv_obj_get_parent(ww.bar)) * c.dutyPct / 100.f);
    lv_obj_set_width(ww.bar, bw);
    lv_obj_set_style_bg_color(ww.bar, dutyBarColor(c.dutyPct), 0);
    snprintf(buf, sizeof(buf), "%d%%", c.dutyPct);
    setLbl(ww.duty, buf);

    snprintf(buf, sizeof(buf), "%.0f\xC2\xB0""F", c.setptF);
    setLbl(ww.setpt, buf);

    bool en = (strcmp(c.status, "HEATING")  == 0 ||
               strcmp(c.status, "HOLDING")  == 0 ||
               strcmp(c.status, "REDUCING") == 0 ||
               strcmp(c.status, "CUTOFF")   == 0 ||
               strcmp(c.status, "SENS ERR") == 0);
    setLbl(ww.enLbl, en ? "ON" : "OFF");
    lv_obj_set_style_bg_color(ww.enBtn,    en ? C_BTN_ON : C_BTN,    0);
    lv_obj_set_style_border_color(ww.enBtn, en ? C_GREEN  : C_BTNBDR, 0);
    lv_obj_set_style_text_color(ww.enLbl,   en ? C_GREEN  : C_DIM,    0);
}

static void updatePsu(lv_obj_t* vout, lv_obj_t* aout, lv_obj_t* watt,
                       lv_obj_t* vin,  lv_obj_t* setv,
                       lv_obj_t* onBtn, lv_obj_t* onLbl) {
    // Don't touch widgets until we've received at least one packet
    if (!psu.received) return;

    char buf[20];

    // Only declare comm-loss after 3 consecutive bad packets to avoid
    // single-packet glitches flashing the display red.
    if (psu.failStreak >= 3) {
        if (vout) setLbl(vout, "---");
        if (aout) setLbl(aout, "---");
        if (watt) setLbl(watt, "---");
        if (vin)  setLbl(vin,  "---");
        if (setv) setLbl(setv, "--.- V");
        if (onLbl) setLbl(onLbl, "NO COMM");
        if (onBtn) {
            lv_obj_set_style_bg_color(onBtn,     C_BTN_ERR, 0);
            lv_obj_set_style_border_color(onBtn, C_RED,     0);
        }
        if (onLbl) lv_obj_set_style_text_color(onLbl, C_RED, 0);
        return;
    }

    // Good data — update readings
    snprintf(buf, sizeof(buf), "%.1f", psu.measV);  if (vout) setLbl(vout, buf);
    snprintf(buf, sizeof(buf), "%.1f", psu.measA);  if (aout) setLbl(aout, buf);
    snprintf(buf, sizeof(buf), "%.1f", psu.measW);  if (watt) setLbl(watt, buf);
    snprintf(buf, sizeof(buf), "%.1f", psu.inputV); if (vin)  setLbl(vin,  buf);
    if (setv) {
        snprintf(buf, sizeof(buf), "%.2f V", psu.setV);
        setLbl(setv, buf);
    }
    if (onBtn && onLbl) {
        const char* txt; lv_color_t bg, bd, tc;
        if      (psu.on && psu.cc)  { txt="ON - CC"; bg=C_BTN_CC;  bd=C_AMBER; tc=C_AMBER; }
        else if (psu.on)            { txt="ON - CV"; bg=C_BTN_ON;  bd=C_GREEN; tc=C_GREEN; }
        else                        { txt="OFF";     bg=C_BTN;     bd=C_BTNBDR;tc=C_DIM;   }
        setLbl(onLbl, txt);
        lv_obj_set_style_bg_color(onBtn,     bg, 0);
        lv_obj_set_style_border_color(onBtn, bd, 0);
        lv_obj_set_style_text_color(onLbl,   tc, 0);
    }
}

// ── Brightness ───────────────────────────────────────────────────────────────
static uint8_t _brightness = 200;   // current backlight level (0-255); loaded from NVS

static void onBrightness(lv_event_t*) {
    int val = lv_slider_get_value(_brtSlider);
    display_set_brightness((uint8_t)val);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", val * 100 / 255);
    lv_label_set_text(_brtPctLbl, buf);
    _brightness = (uint8_t)val;
    _prefs.putUChar("brt", _brightness);
}

// ── IAQ helpers (used by updateUI and buildEnvPage) ──────────────────────────
static const char* iaqLabel(float kohm) {
    if (kohm >= 300.f) return "Excellent";
    if (kohm >= 150.f) return "Good";
    if (kohm >=  50.f) return "Fair";
    if (kohm >=  25.f) return "Poor";
    return "Bad";
}
static lv_color_t iaqColor(float kohm) {
    if (kohm >= 300.f) return C_GREEN;
    if (kohm >= 150.f) return lv_color_hex(0x55ccaa);
    if (kohm >=  50.f) return C_AMBER;
    if (kohm >=  25.f) return C_ACCENT;
    return C_RED;
}

static void updateUI() {
    char buf[40];

    // Header env label (temp + humidity summary)
    if (env.valid) {
        snprintf(buf, sizeof(buf), "%.1f\xC2\xB0  %.1f%%", env.tempF, env.humidity);
        setLbl(_envLbl, buf);
    }

    // Environment page — all 4 cards
    if (env.valid) {
        // Temperature card
        snprintf(buf, sizeof(buf), "%.1f", env.tempF);
        setLbl(_envPageTempLbl, buf);
        lv_arc_set_value(_envPageTempArc, (int)env.tempF);

        // Humidity card
        snprintf(buf, sizeof(buf), "%.1f", env.humidity);
        setLbl(_envPageHumLbl, buf);
        lv_arc_set_value(_envPageHumArc, (int)env.humidity);

        // Pressure card
        snprintf(buf, sizeof(buf), "%.1f", env.pressHpa);
        setLbl(_envPagePresLbl, buf);
        int pressClamped = (int)env.pressHpa;
        if (pressClamped < 950) pressClamped = 950;
        if (pressClamped > 1050) pressClamped = 1050;
        lv_arc_set_value(_envPagePresArc, pressClamped);

        // IAQ card — value, badge, tier bars
        snprintf(buf, sizeof(buf), "%.1f", env.gasKohm);
        setLbl(_envPageGasLbl, buf);
        lv_color_t iqc = iaqColor(env.gasKohm);
        lv_obj_set_style_text_color(_envPageGasLbl, iqc, 0);
        setLbl(_envPageIaqLbl, iaqLabel(env.gasKohm));
        lv_obj_set_style_text_color(_envPageIaqLbl, iqc, 0);
        lv_obj_set_style_border_color(_envPageIaqBadge, iqc, 0);

        // Tier bars — each shows how far through that tier the reading has gone
        const float tierMin[5] = {300.f, 150.f, 50.f, 25.f, 0.f};
        const float tierMax[5] = {500.f, 300.f, 150.f, 50.f, 25.f};
        for (int i = 0; i < 5; i++) {
            float pct = (env.gasKohm - tierMin[i]) / (tierMax[i] - tierMin[i]);
            pct = pct < 0.f ? 0.f : pct > 1.f ? 1.f : pct;
            int tw = lv_obj_get_width(lv_obj_get_parent(_envPageIaqBars[i]));
            lv_obj_set_width(_envPageIaqBars[i], (int)(pct * tw));
        }
    }

    // PSU — dashboard strip + PSU page
    updatePsu(_dPsuV, _dPsuA, _dPsuW, _dPsuVin, _dPsuSetV, _dPsuOnBtn, _dPsuOnLbl);
    updatePsu(_psuVout, _psuAout, _psuWatt, _psuVin, _psuSetVLbl, _psuOnBtn, _psuOnLbl);

    // Preset button highlights
    if (psu.ok) {
        int active = -1;
        for (int i = 0; i < 9; i++)
            if (fabsf(psu.setV - (float)PRESET_V[i]) < 0.05f) { active = i; break; }
        for (int i = 0; i < 9; i++) {
            bool a = (i == active);
            lv_obj_set_style_bg_color(_psuPreset[i],     a ? C_PRESET_ACT : C_BTN,    0);
            lv_obj_set_style_border_color(_psuPreset[i], a ? C_BLUE       : C_BTNBDR, 0);
            lv_obj_t* l = lv_obj_get_child(_psuPreset[i], 0);
            if (l) lv_obj_set_style_text_color(l, a ? C_BLUE : C_DIM, 0);
        }
    }

    // Channel cards — dashboard (compact) and warmer (full)
    for (int i = 0; i < 2; i++) {
        updateChCard(_dcw[i], i);
        updateChCard(_wcw[i], i);
    }
}

// ── Environment page ──────────────────────────────────────────────────────────

static void buildEnvPage() {
    _pageEnv = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_pageEnv, SCR_W, SCR_H);
    lv_obj_set_pos(_pageEnv, 0, 0);
    lv_obj_set_style_bg_opa(_pageEnv, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_pageEnv, 0, 0);
    lv_obj_set_style_pad_all(_pageEnv, 0, 0);
    lv_obj_set_style_shadow_width(_pageEnv, 0, 0);
    lv_obj_clear_flag(_pageEnv, LV_OBJ_FLAG_SCROLLABLE);

    // ── Layout ───────────────────────────────────────────────────────────────
    const int IP    = 14;
    const int SB_W  = 60;
    const int SB_X  = SCR_W - PAD - SB_W;           // 730
    const int TW    = (SB_X - PAD * 3) / 2;          // 350
    const int TH    = (SCR_H - HDR_H - PAD * 3) / 2; // 200
    const int IW    = TW - IP * 2;                    // 322
    const int IH    = TH - IP * 2;                    // 172
    const int R1Y   = HDR_H + PAD;                    // 60
    const int R2Y   = HDR_H + PAD * 2 + TH;           // 270
    const int C1X   = PAD;                            // 10
    const int C2X   = PAD * 2 + TW;                   // 370

    // Gauge split: divider at DIV_X, arc fills right portion
    const int DIV_X  = 148;
    const int ARC_X  = DIV_X + 8;          // 156
    const int ARC_SZ = IW - ARC_X;         // 166 — square arc
    const int ARC_Y  = (IH - ARC_SZ) / 2; // 3   — vertically centred

    // ── Card factory ─────────────────────────────────────────────────────────
    auto mkC = [&](int x, int y) -> lv_obj_t* {
        lv_obj_t* c = lv_obj_create(_pageEnv);
        lv_obj_set_size(c, TW, TH);
        lv_obj_set_pos(c, x, y);
        lv_obj_set_style_bg_color(c, lv_color_hex(0x111111), 0);
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(c, lv_color_hex(0x2a2a2a), 0);
        lv_obj_set_style_border_width(c, 1, 0);
        lv_obj_set_style_radius(c, CARD_R, 0);
        lv_obj_set_style_pad_all(c, IP, 0);
        lv_obj_set_style_shadow_width(c, 0, 0);
        lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
        return c;
    };

    // ── Vertical divider ─────────────────────────────────────────────────────
    auto mkDiv = [&](lv_obj_t* p) {
        lv_obj_t* d = lv_obj_create(p);
        lv_obj_set_size(d, 1, IH);
        lv_obj_set_pos(d, DIV_X, 0);
        lv_obj_set_style_bg_color(d, lv_color_hex(0x282828), 0);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(d, 0, 0);
        lv_obj_set_style_radius(d, 0, 0);
        lv_obj_set_style_pad_all(d, 0, 0);
        lv_obj_set_style_shadow_width(d, 0, 0);
        lv_obj_clear_flag(d, LV_OBJ_FLAG_SCROLLABLE);
    };

    // ── Arc gauge factory ────────────────────────────────────────────────────
    // C-shape arc (135°→45°, 270° sweep, gap at bottom). Same bg_angles as
    // the existing pressure arc. set_value(lo) = empty indicator on startup.
    auto mkGauge = [&](lv_obj_t* p, int lo, int hi,
                       lv_color_t col, lv_color_t bgCol) -> lv_obj_t* {
        lv_obj_t* a = lv_arc_create(p);
        lv_obj_set_size(a, ARC_SZ, ARC_SZ);
        lv_obj_set_pos(a, ARC_X, ARC_Y);
        lv_arc_set_bg_angles(a, 135, 45);
        lv_arc_set_range(a, lo, hi);
        lv_arc_set_value(a, lo);
        lv_arc_set_mode(a, LV_ARC_MODE_NORMAL);
        lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_arc_color(a, bgCol, LV_PART_MAIN);
        lv_obj_set_style_arc_width(a, 12, LV_PART_MAIN);
        lv_obj_set_style_arc_color(a, col, LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(a, 12, LV_PART_INDICATOR);
        lv_obj_set_style_opa(a, LV_OPA_0, LV_PART_KNOB);
        lv_obj_set_style_size(a, 0, LV_PART_KNOB);
        lv_obj_set_style_bg_opa(a, LV_OPA_0, 0);
        lv_obj_set_style_border_width(a, 0, 0);
        lv_obj_set_style_shadow_width(a, 0, 0);
        lv_obj_set_style_pad_all(a, 4, 0);
        // Disable animation so each set_value() updates instantly (no multi-frame
        // dirty writes per telemetry packet — reduces PSRAM write pressure).
        lv_arc_set_change_rate(a, 0);
        return a;
    };

    // ── Unit label centred in arc interior ───────────────────────────────────
    auto mkArcUnit = [&](lv_obj_t* p, const char* txt, lv_color_t col) {
        lv_obj_t* u = mkLbl(p, txt, &lv_font_montserrat_12, col);
        lv_obj_set_style_opa(u, LV_OPA_50, 0);
        lv_obj_set_style_text_align(u, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(u, 32);
        // x: centre of arc interior;  y: ~62% down the arc (above the gap)
        lv_obj_set_pos(u, ARC_X + (ARC_SZ - 32) / 2,
                           ARC_Y + (int)(ARC_SZ * 0.62f));
    };

    // ── Card 1 — Temperature ─────────────────────────────────────────────────
    {
        lv_obj_t* c = mkC(C1X, R1Y);
        lv_obj_t* h = mkLbl(c, "TEMPERATURE", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_style_text_letter_space(h, 2, 0); lv_obj_set_pos(h, 0, 0);
        _envPageTempLbl = mkLbl(c, "---", &lv_font_montserrat_40, C_GREEN);
        lv_obj_set_pos(_envPageTempLbl, 0, 16);
        lv_obj_t* u = mkLbl(c, "\xC2\xB0""F", &lv_font_montserrat_16, C_GREEN);
        lv_obj_set_pos(u, 0, 70);
        lv_obj_t* r = mkLbl(c, "40 - 120\xC2\xB0""F", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_pos(r, 0, 123);
        mkDiv(c);
        _envPageTempArc = mkGauge(c, 40, 120, C_GREEN, lv_color_hex(0x1a2a1a));
        mkArcUnit(c, "\xC2\xB0""F", C_GREEN);
    }

    // ── Card 2 — Humidity ────────────────────────────────────────────────────
    {
        lv_obj_t* c = mkC(C2X, R1Y);
        lv_obj_t* h = mkLbl(c, "HUMIDITY", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_style_text_letter_space(h, 2, 0); lv_obj_set_pos(h, 0, 0);
        _envPageHumLbl = mkLbl(c, "---", &lv_font_montserrat_40, C_BLUE);
        lv_obj_set_pos(_envPageHumLbl, 0, 16);
        lv_obj_t* u = mkLbl(c, "%RH", &lv_font_montserrat_16, C_BLUE);
        lv_obj_set_pos(u, 0, 70);
        lv_obj_t* r = mkLbl(c, "Comfort: 30 - 60%", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_pos(r, 0, 123);
        mkDiv(c);
        _envPageHumArc = mkGauge(c, 0, 100, C_BLUE, lv_color_hex(0x1a1e2a));
        mkArcUnit(c, "%", C_BLUE);
    }

    // ── Card 3 — Pressure ────────────────────────────────────────────────────
    {
        lv_obj_t* c = mkC(C1X, R2Y);
        lv_obj_t* h = mkLbl(c, "PRESSURE", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_style_text_letter_space(h, 2, 0); lv_obj_set_pos(h, 0, 0);
        _envPagePresLbl = mkLbl(c, "---", &lv_font_montserrat_40, C_MUTED);
        lv_obj_set_pos(_envPagePresLbl, 0, 16);
        lv_obj_t* u = mkLbl(c, "hPa", &lv_font_montserrat_16, C_DIM);
        lv_obj_set_pos(u, 0, 70);
        lv_obj_t* r1 = mkLbl(c, "Normal: 1013 hPa", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_pos(r1, 0, 113);
        lv_obj_t* r2 = mkLbl(c, "Range: 950 - 1050", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_pos(r2, 0, 133);
        mkDiv(c);
        _envPagePresArc = mkGauge(c, 950, 1050, C_MUTED, lv_color_hex(0x252525));
        mkArcUnit(c, "hPa", C_MUTED);
    }

    // ── Card 4 — Air Quality ─────────────────────────────────────────────────
    {
        lv_obj_t* c = mkC(C2X, R2Y);
        lv_obj_t* h = mkLbl(c, "AIR QUALITY", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_style_text_letter_space(h, 2, 0); lv_obj_set_pos(h, 0, 0);

        const int LS = DIV_X;   // match other cards' divider position

        _envPageGasLbl = mkLbl(c, "---", &lv_font_montserrat_40, C_AMBER);
        lv_obj_set_pos(_envPageGasLbl, 0, 16);
        lv_obj_t* gu = mkLbl(c, "kOhm", &lv_font_montserrat_14, C_DIM);
        lv_obj_set_pos(gu, 0, 70);

        // IAQ badge
        _envPageIaqBadge = lv_obj_create(c);
        lv_obj_set_size(_envPageIaqBadge, 90, 28);
        lv_obj_set_pos(_envPageIaqBadge, 0, 115);
        lv_obj_set_style_radius(_envPageIaqBadge, 14, 0);
        lv_obj_set_style_bg_color(_envPageIaqBadge, lv_color_hex(0x1a1000), 0);
        lv_obj_set_style_bg_opa(_envPageIaqBadge, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(_envPageIaqBadge, C_AMBER, 0);
        lv_obj_set_style_border_width(_envPageIaqBadge, 1, 0);
        lv_obj_set_style_pad_all(_envPageIaqBadge, 0, 0);
        lv_obj_set_style_shadow_width(_envPageIaqBadge, 0, 0);
        lv_obj_clear_flag(_envPageIaqBadge, LV_OBJ_FLAG_SCROLLABLE);
        _envPageIaqLbl = mkLbl(_envPageIaqBadge, "---", &lv_font_montserrat_14, C_AMBER);
        lv_obj_center(_envPageIaqLbl);

        // Divider
        lv_obj_t* vd = lv_obj_create(c);
        lv_obj_set_size(vd, 1, IH);
        lv_obj_set_pos(vd, LS, 0);
        lv_obj_set_style_bg_color(vd, lv_color_hex(0x282828), 0);
        lv_obj_set_style_bg_opa(vd, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(vd, 0, 0);
        lv_obj_set_style_radius(vd, 0, 0);
        lv_obj_set_style_pad_all(vd, 0, 0);
        lv_obj_set_style_shadow_width(vd, 0, 0);
        lv_obj_clear_flag(vd, LV_OBJ_FLAG_SCROLLABLE);

        // Tier bars (right side)
        const int RSX = LS + 10;
        const int RSW = IW - RSX;
        const int LW  = 52;
        const int THW = 30;
        const int BW  = RSW - LW - THW - 8;
        const int RH  = 26;
        const int TY0 = (IH - 5 * RH) / 2;

        const char* tl[5] = {"Excellent","Good","Fair","Poor","Bad"};
        const char* tt[5] = {">300",">150",">50",">25","<25"};
        lv_color_t  tc[5] = {C_GREEN, lv_color_hex(0x55ccaa), C_AMBER, C_ACCENT, C_RED};

        for (int i = 0; i < 5; i++) {
            int ry = TY0 + i * RH;
            int cy = ry + (RH - 14) / 2;
            lv_obj_t* tl_ = mkLbl(c, tl[i], &lv_font_montserrat_12, C_DIM);
            lv_obj_set_pos(tl_, RSX, cy); lv_obj_set_width(tl_, LW);

            lv_obj_t* tr = lv_obj_create(c);
            lv_obj_set_size(tr, BW, 6);
            lv_obj_set_pos(tr, RSX + LW + 6, ry + (RH - 6) / 2);
            lv_obj_set_style_bg_color(tr, lv_color_hex(0x1e1e1e), 0);
            lv_obj_set_style_bg_opa(tr, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(tr, 0, 0);
            lv_obj_set_style_radius(tr, 3, 0);
            lv_obj_set_style_pad_all(tr, 0, 0);
            lv_obj_set_style_shadow_width(tr, 0, 0);
            lv_obj_clear_flag(tr, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t* tf = lv_obj_create(tr);
            lv_obj_set_size(tf, 0, 6);
            lv_obj_set_pos(tf, 0, 0);
            lv_obj_set_style_bg_color(tf, tc[i], 0);
            lv_obj_set_style_bg_opa(tf, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(tf, 0, 0);
            lv_obj_set_style_radius(tf, 3, 0);
            lv_obj_set_style_pad_all(tf, 0, 0);
            _envPageIaqBars[i] = tf;

            lv_obj_t* th_ = mkLbl(c, tt[i], &lv_font_montserrat_12, C_DIM);
            lv_obj_set_pos(th_, RSX + LW + BW + 10, cy);
            lv_obj_set_width(th_, THW);
        }
    }

    // ── Brightness sidebar ────────────────────────────────────────────────────
    // Vertical slider: LVGL 8 renders a slider vertically when height > width.
    // Min is at the bottom (value 10), max at the top (255); dragging up = brighter.
    //
    // NOTE — Advance board migration (arriving tomorrow):
    //   display_set_brightness() currently calls tft.setBrightness() via Light_PWM
    //   on GPIO2.  On Advance v1.1, replace the body of display_set_brightness()
    //   in display_driver.cpp with:
    //     Wire.beginTransmission(0x30);
    //     Wire.write(0x05 + (uint8_t)((level / 255.0f) * 11));  // 0x05–0x10
    //     Wire.endTransmission();
    {
        const int IP_S = 8;
        const int SB_H = SCR_H - HDR_H - PAD * 2;          // 410
        const int IW_S = SB_W - IP_S * 2;                   // 44
        const int SLH  = (SB_H - IP_S * 2) - 42 - 36;      // 316  track height
        const int SLX  = (IW_S - 8) / 2;                    // 18   centre 8px track
        const int SLY  = 42;                                 // below bulb icon

        lv_obj_t* sb = lv_obj_create(_pageEnv);
        lv_obj_set_size(sb, SB_W, SB_H);
        lv_obj_set_pos(sb, SB_X, HDR_H + PAD);
        lv_obj_set_style_bg_color(sb, lv_color_hex(0x111111), 0);
        lv_obj_set_style_bg_opa(sb, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(sb, lv_color_hex(0x2a2a2a), 0);
        lv_obj_set_style_border_width(sb, 1, 0);
        lv_obj_set_style_radius(sb, CARD_R, 0);
        lv_obj_set_style_pad_all(sb, IP_S, 0);
        lv_obj_set_style_shadow_width(sb, 0, 0);
        lv_obj_clear_flag(sb, LV_OBJ_FLAG_SCROLLABLE);

        // Lightbulb: filled amber circle (bulb) + smaller amber rect (base)
        const int BX = (IW_S - 20) / 2;   // 12 — centre 20px circle in 44px
        lv_obj_t* bulb = lv_obj_create(sb);
        lv_obj_set_size(bulb, 20, 20);
        lv_obj_set_pos(bulb, BX, 2);
        lv_obj_set_style_bg_color(bulb, C_AMBER, 0);
        lv_obj_set_style_bg_opa(bulb, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(bulb, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(bulb, 0, 0);
        lv_obj_set_style_shadow_width(bulb, 0, 0);
        lv_obj_clear_flag(bulb, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t* base = lv_obj_create(sb);
        lv_obj_set_size(base, 12, 6);
        lv_obj_set_pos(base, BX + 4, 20);
        lv_obj_set_style_bg_color(base, C_AMBER, 0);
        lv_obj_set_style_bg_opa(base, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(base, 2, 0);
        lv_obj_set_style_border_width(base, 0, 0);
        lv_obj_set_style_shadow_width(base, 0, 0);
        lv_obj_clear_flag(base, LV_OBJ_FLAG_SCROLLABLE);

        // Vertical slider (height > width → LVGL 8 vertical mode)
        _brtSlider = lv_slider_create(sb);
        lv_obj_set_size(_brtSlider, 8, SLH);
        lv_obj_set_pos(_brtSlider, SLX, SLY);
        lv_slider_set_range(_brtSlider, 10, 255);
        lv_slider_set_value(_brtSlider, _brightness, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(_brtSlider, lv_color_hex(0x1e1e1e), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_brtSlider, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(_brtSlider, 4, LV_PART_MAIN);
        lv_obj_set_style_bg_color(_brtSlider, lv_color_hex(0x666666), LV_PART_INDICATOR);
        lv_obj_set_style_radius(_brtSlider, 4, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(_brtSlider, C_TEXT, LV_PART_KNOB);
        lv_obj_set_style_radius(_brtSlider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
        lv_obj_set_style_pad_all(_brtSlider, 10, LV_PART_KNOB);
        lv_obj_set_style_shadow_width(_brtSlider, 0, 0);
        // LV_EVENT_RELEASED: brightness updates when finger lifts, not continuously
        // during drag. The slider knob still moves visually during drag (LVGL handles
        // that internally as a small dirty region), but the heavy operations
        // (display_set_brightness + label update + NVS write) only fire once.
        lv_obj_add_event_cb(_brtSlider, onBrightness, LV_EVENT_RELEASED, nullptr);

        // Percentage label (centred, updates live in onBrightness)
        char pctBuf[8];
        snprintf(pctBuf, sizeof(pctBuf), "%d%%", _brightness * 100 / 255);
        _brtPctLbl = mkLbl(sb, pctBuf, &lv_font_montserrat_12, C_MUTED);
        lv_obj_set_width(_brtPctLbl, IW_S);
        lv_obj_set_style_text_align(_brtPctLbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(_brtPctLbl, 0, SLY + SLH + 8);

        lv_obj_t* brtTag = mkLbl(sb, "BRT", &lv_font_montserrat_12, C_DIM);
        lv_obj_set_width(brtTag, IW_S);
        lv_obj_set_style_text_align(brtTag, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(brtTag, 0, SLY + SLH + 24);
    }
}

// ── Top-level build ───────────────────────────────────────────────────────────
static void buildUI() {
    lv_obj_t* scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    buildDashboardPage();
    buildPsuPage();
    buildWarmerPage();
    buildEnvPage();
    buildHeader();          // header last → renders on top of pages

    lv_obj_add_flag(_pagePsu,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(_pageWarm, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(_pageEnv,  LV_OBJ_FLAG_HIDDEN);
    showPage(PAGE_DASH);
}

// ── UART receive buffer ───────────────────────────────────────────────────────
static char _rxBuf[320];
static int  _rxPos = 0;

// =============================================================================
void setup() {
    Serial.begin(115200);
    { unsigned long t = millis(); while (!Serial && millis()-t < 1000) delay(5); }
    Serial.println("[CrowPanel] PSU Warmer Display booting...");

    // Load saved brightness before buildUI() so the slider initialises with it
    _prefs.begin("warmer_ui", false);
    _brightness = _prefs.getUChar("brt", 200);   // default ~78%

    FeatherPort.begin(FEATHER_BAUD, SERIAL_8N1, FEATHER_RX_PIN, FEATHER_TX_PIN);

    display_driver_init();
    buildUI();

    // Run 20 render cycles to pre-populate the PSRAM framebuffer before the
    // backlight comes on. Touch is left DISABLED here — it will be armed in
    // loop() after a 500ms lockout so any GT911 initialization ghost state
    // has cleared before LVGL ever polls the controller.
    lv_indev_t* _touch = lv_indev_get_next(NULL);
    if (_touch) lv_indev_enable(_touch, false);
    for (int i = 0; i < 20; i++) { lv_timer_handler(); delay(10); }
    Serial.println("[display] Framebuffer pre-rendered, touch armed after 500ms");
}

void loop() {
    // ── Touch arm (1000ms post-boot lockout) ─────────────────────────────────
    // Touch is left disabled in setup(). At 1000ms we:
    //   1. Drain the GT911 hardware by reading it 8× directly (40ms total).
    //      This consumes any ghost "touched" state the controller holds after
    //      I2C init, before LVGL ever polls it.
    //   2. Call lv_indev_reset() to clear any LVGL indev history.
    //   3. Enable touch.
    // 500ms was not enough — GT911 ghost state can persist beyond that window.
    // 1000ms gives the controller a full second to settle after boot.
    static bool _touchArmed = false;
    if (!_touchArmed && millis() >= 1000) {
        display_drain_touch();               // drain GT911 hardware first
        lv_indev_t* touch = lv_indev_get_next(NULL);
        if (touch) {
            lv_indev_reset(touch, NULL);     // clear LVGL indev history
            lv_indev_enable(touch, true);
        }
        _touchArmed = true;
        Serial.println("[touch] armed at 1000ms");
    }
    // ── Backlight ramp ────────────────────────────────────────────────────────
    // Starts 300ms after boot (gives supply time to stabilise), ramps to the
    // saved brightness level over 2 seconds.
    static bool     _blDone    = false;
    static uint32_t _blStartMs = 300;
    if (!_blDone) {
        uint32_t now = millis();
        if (now >= _blStartMs) {
            uint32_t elapsed = now - _blStartMs;
            if (elapsed >= 2000) {
                display_set_brightness(_brightness);
                _blDone = true;
            } else {
                display_set_brightness((uint8_t)(elapsed * _brightness / 2000));
            }
        }
    }
    // ─────────────────────────────────────────────────────────────────────────

    // ── Drain entire UART buffer before rendering ─────────────────────────────
    // Accumulate all available packets first. Batching into one updateUI() call
    // rather than rendering per-packet prevents multiple renders per vsync interval.
    bool _gotData = false;
    while (FeatherPort.available()) {
        char c = FeatherPort.read();
        if (c == '\n') {
            _rxBuf[_rxPos] = '\0';
            if (_rxPos > 10) {
                parseTelemetry(_rxBuf);
                _gotData = true;
            }
            _rxPos = 0;
        } else if (_rxPos < (int)sizeof(_rxBuf) - 1) {
            _rxBuf[_rxPos++] = c;
        }
    }

    // ── Update LVGL widget states (only when new telemetry arrived) ───────────
    // setLbl() change detection keeps dirty regions small — only changed
    // widgets get re-rendered.
    if (_gotData) updateUI();

    // ── Process touch + render dirty regions ─────────────────────────────────
    display_driver_tick();   // no-op — waitDisplay is in flush callback
    lv_timer_handler();
}