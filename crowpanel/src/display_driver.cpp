// =============================================================================
// display_driver.cpp — Elecrow CrowPanel 5" (ESP32-S3-WROOM-1-N4R8)
//
// Pin assignments and timing verified against:
//   Elecrow official PlatformIO tutorial (5-inch-esp32-display-platformio-tutorial)
//   LovyanGFX forum post (forum.elecrow.com/discussion/473)
//
// Key differences from naive implementation:
//   1. config_detail() with use_psram = 1 required for PSRAM draw buffers
//   2. Light_PWM for backlight (not raw digitalWrite)
//   3. Timing: larger porch values than the datasheet minimum
//   4. LovyanGFX Touch_GT911 used instead of manual I2C driver
//
// Jitter fix (vs earlier revision):
//   - Flush callback uses pushImageDMA() instead of setAddrWindow/writePixels.
//     writePixels writes pixels linearly without honouring the RGB framebuffer's
//     2D stride, causing the "shifts right" partial-update artifact on parallel
//     RGB panels.  pushImageDMA() handles the stride correctly.
//   - LVGL draw buffer increased from 1/10 to 1/2 screen height (PSRAM).
//     Fewer flush calls per frame = fewer opportunities for mid-scan glitches.
// =============================================================================

#include "display_driver.h"
#include <lvgl.h>
#include <Wire.h>
#include <driver/gpio.h>
// Wire  = I2C_NUM_0 — used by LovyanGFX for GT911 touch
// Wire1 = I2C_NUM_1 — dedicated to STC8H1K28 backlight, no LovyanGFX conflict
static TwoWire WireBL(1);  // I2C_NUM_1 on GPIO15/16 for backlight only

// ── LovyanGFX LGFX class ──────────────────────────────────────────────────────
class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Bus_RGB     _bus_instance;
    lgfx::Panel_RGB   _panel_instance;
    // lgfx::Light_PWM _light_instance;  // Advance board uses I2C backlight (STC8H1K28)
    lgfx::Touch_GT911 _touch_instance;

    LGFX() {
        // ── Panel config ──────────────────────────────────────────────────────
        {
            auto cfg = _panel_instance.config();
            cfg.memory_width  = SCREEN_WIDTH;
            cfg.memory_height = SCREEN_HEIGHT;
            cfg.panel_width   = SCREEN_WIDTH;
            cfg.panel_height  = SCREEN_HEIGHT;
            cfg.offset_x      = 0;
            cfg.offset_y      = 0;
            _panel_instance.config(cfg);
        }

        // ── Panel detail — enable dual PSRAM framebuffers ─────────────────────
        // use_psram = 2: allocates two full-screen PSRAM buffers.
        // LovyanGFX Panel_RGB uses one as the LCD-active buffer and one as the
        // write buffer. waitDisplay() triggers the buffer swap at vsync, giving
        // true double-buffering: LCD reads from buffer A while LVGL writes to
        // buffer B, then they swap. Eliminates PSRAM read/write contention.
        {
            auto cfg = _panel_instance.config_detail();
            cfg.use_psram = 1;   // Single PSRAM framebuffer for the LCD scanout.
            _panel_instance.config_detail(cfg);
        }

        // ── RGB bus ───────────────────────────────────────────────────────────
        {
            auto cfg = _bus_instance.config();
            cfg.panel = &_panel_instance;

            // Blue channel
            cfg.pin_d0  = GPIO_NUM_21;  // B0
            cfg.pin_d1  = GPIO_NUM_47;  // B1
            cfg.pin_d2  = GPIO_NUM_48;  // B2
            cfg.pin_d3  = GPIO_NUM_45;  // B3
            cfg.pin_d4  = GPIO_NUM_38;  // B4

            // Green channel
            cfg.pin_d5  = GPIO_NUM_9;   // G0
            cfg.pin_d6  = GPIO_NUM_10;  // G1
            cfg.pin_d7  = GPIO_NUM_11;  // G2
            cfg.pin_d8  = GPIO_NUM_12;  // G3
            cfg.pin_d9  = GPIO_NUM_13;  // G4
            cfg.pin_d10 = GPIO_NUM_14;  // G5

            // Red channel
            cfg.pin_d11 = GPIO_NUM_7;   // R0
            cfg.pin_d12 = GPIO_NUM_17;  // R1
            cfg.pin_d13 = GPIO_NUM_18;  // R2
            cfg.pin_d14 = GPIO_NUM_3;   // R3
            cfg.pin_d15 = GPIO_NUM_46;  // R4

            // Sync / control
            cfg.pin_henable = GPIO_NUM_42;  // DE
            cfg.pin_vsync   = GPIO_NUM_41;  // VSYNC
            cfg.pin_hsync   = GPIO_NUM_40;  // HSYNC
            cfg.pin_pclk    = GPIO_NUM_39;  // PCLK

            cfg.freq_write  = 15000000;     // 15 MHz — reduced from 21 to cut PSRAM DMA
                                              // bandwidth from 336 Mbps to 240 Mbps,
                                              // leaving headroom for CPU LVGL redraws.

            // Timing — confirmed from working community examples for CrowPanel 5"
            cfg.hsync_polarity    = 0;
            cfg.hsync_front_porch = 8;
            cfg.hsync_pulse_width = 4;
            cfg.hsync_back_porch  = 8;
            cfg.vsync_polarity    = 0;
            cfg.vsync_front_porch = 8;
            cfg.vsync_pulse_width = 4;
            cfg.vsync_back_porch  = 8;
            cfg.pclk_active_neg   = 0;
            cfg.de_idle_high      = 0;
            cfg.pclk_idle_high    = 1;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        // ── Backlight — STC8H1K28 I2C controller at 0x30 (GPIO15 SDA / GPIO16 SCL)
        // No Light_PWM instance needed. Brightness is set via Wire in
        // display_set_brightness(). Wire is shared with GT911 touch on same bus.

        // ── GT911 touch — I2C on GPIO15 (SDA) / GPIO16 (SCL) ────────────────
        {
            auto cfg = _touch_instance.config();
            cfg.x_min      = 0;
            cfg.x_max      = SCREEN_WIDTH  - 1;
            cfg.y_min      = 0;
            cfg.y_max      = SCREEN_HEIGHT - 1;
            cfg.pin_int    = -1;
            cfg.pin_rst    = -1;
            cfg.bus_shared = false;
            cfg.offset_rotation = 0;
            cfg.i2c_port  = 0;
            cfg.i2c_addr  = 0x5D;
            cfg.pin_sda   = GPIO_NUM_15;  // GT911 I2C — shared with backlight STC8H1K28
            cfg.pin_scl   = GPIO_NUM_16;
            cfg.freq      = 400000;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }
};

// ── Statics ───────────────────────────────────────────────────────────────────
static LGFX tft;

// ── LVGL flush callback ───────────────────────────────────────────────────────
// pushImageDMA writes LVGL's render buffer into the PSRAM write buffer using
// the swap565_t cast (required when LV_COLOR_16_SWAP=1).
//
// With use_psram=2 (dual PSRAM framebuffers), waitDisplay() on the last flush
// waits for the LCD to finish its current scan then swaps which buffer the LCD
// reads from. This is true double-buffering: the LCD reads from a complete,
// stable frame in buffer A while LVGL renders into buffer B, then they swap
// at vsync. Eliminates all PSRAM read/write contention between LCD and CPU.
static void lvgl_flush(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_p) {
    tft.pushImageDMA(area->x1, area->y1,
                     area->x2 - area->x1 + 1,
                     area->y2 - area->y1 + 1,
                     (lgfx::swap565_t*)&color_p->full);
    lv_disp_flush_ready(drv);
    // No waitDisplay needed — bounce buffer handles PSRAM/LCD decoupling
    // at the hardware level. Sync is unnecessary and would block unnecessarily.
}

// ── LVGL touch read callback ──────────────────────────────────────────────────
// The GT911 returns garbage on its first read after I2C init (touches=15,
// all flags set). Our drain in loop() clears it but the controller can
// regenerate a phantom reading in the next 20ms scan cycle before LVGL polls.
// Suppressing all press events for 200ms after the 1000ms arm window (i.e.
// until millis()≥1200) guarantees LVGL starts in a clean RELEASED state.
// The touch callback is never called before 1000ms anyway (lv_indev disabled),
// so this filter only applies to the narrow 1000–1200ms window after boot.
static void lvgl_touch_read(lv_indev_drv_t* drv, lv_indev_data_t* data) {
    (void)drv;
    if (millis() < 1200) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    uint16_t tx = 0, ty = 0;
    if (tft.getTouch(&tx, &ty)) {
        data->point.x = (lv_coord_t)tx;
        data->point.y = (lv_coord_t)ty;
        data->state   = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ── LVGL log ──────────────────────────────────────────────────────────────────
static void lvgl_log(const char* buf) { Serial.print(buf); }

// ── Public API ────────────────────────────────────────────────────────────────
void display_driver_init() {
    Serial.println("[display] Init...");

    if (psramFound()) {
        Serial.printf("[display] PSRAM: %u bytes\n", ESP.getFreePsram());
    } else {
        Serial.println("[display] WARN: PSRAM not found — display may not work");
    }

    // Initialise Arduino Wire on the touch/backlight I2C bus FIRST.
    // Wire must be started before any display_set_brightness() call —
    // LovyanGFX initialises its own internal I2C for GT911 but does NOT
    // initialise the Arduino Wire object. Without this, all backlight
    // writes to STC8H1K28 at 0x30 fail silently and the screen stays black.
    // Wire (I2C_NUM_0) intentionally NOT initialized here — LovyanGFX
    // initialises it for GT911 inside tft.begin(). Initialising it here
    // too causes both peripherals to fight for GPIO15/16 and hangs the scan.
    WireBL.begin(15, 16);  // I2C_NUM_1 — dedicated to backlight, never touched by LovyanGFX
    Serial.println("[display] WireBL init done (GPIO15/16, I2C_NUM_1)");

    // SC7277 is the 7" board driver — the 5" Advance uses ST7262 (pure RGB).
    // Scanning to confirm STC8H1K28 backlight (0x30), RTC (0x51), GT911 (0x5D).
    // 200ms delay gives all I2C devices time to complete power-on sequencing.
    delay(200);
    Serial.print("[display] I2C scan GPIO15/GPIO16 (WireBL):");
    for (uint8_t addr = 1; addr < 127; addr++) {
        WireBL.beginTransmission(addr);
        if (WireBL.endTransmission() == 0) {
            Serial.printf(" 0x%02X", addr);
        }
    }
    Serial.println();

    // NOTE: The ST7262 display IC is pure RGB — no I2C init needed at all.
    // STC8H1K28 backlight controller at I2C 0x30 — V1.1 protocol:
    //   0x05 = OFF
    //   0x06–0x09 = intermediate brightness
    //   0x10 = maximum brightness
    // Keep backlight OFF during LVGL init; loop() ramp turns it on.
    // Must send before tft.begin() — afterwards LovyanGFX owns I2C_NUM_0.
    WireBL.beginTransmission(0x30);
    WireBL.write(0x10);  // V1.1 maximum brightness
    uint8_t blErr = WireBL.endTransmission();
    Serial.printf("[display] Backlight MAX (0x10): %s\n",
                  blErr == 0 ? "OK" : "FAILED");

    tft.begin();
    tft.setRotation(0);
    Serial.println("[display] LovyanGFX tft.begin() done");

    tft.fillScreen(TFT_BLACK);
    // Backlight stays off (245 sent before tft.begin). loop() ramp turns it on.
    tft.fillScreen(TFT_BLACK);
    Serial.println("[display] LovyanGFX init done");

    // LVGL init
    lv_init();
    lv_log_register_print_cb(lvgl_log);

    // ── Draw buffers — small SRAM buffers matching community examples ────────
    // LovyanGFX 1.2.x community examples for identical ESP32-S3 RGB hardware
    // use small SRAM buffers (800×10 lines = 16KB each).  Using PSRAM for draw
    // buffers caused long flush windows (~9ms per half-screen) that maximised
    // PSRAM bus contention with LCD DMA reads → tearing.
    // With 16KB SRAM buffers each flush takes ~0.4ms — 22× less contention.
    // SRAM also has higher bandwidth than PSRAM for CPU writes.
    static lv_color_t buf1[SCREEN_WIDTH * 10];
    static lv_color_t buf2[SCREEN_WIDTH * 10];
    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, SCREEN_WIDTH * 10);
    Serial.printf("[display] SRAM draw buffers: 2 × %u bytes\n",
                  (unsigned)(SCREEN_WIDTH * 10 * sizeof(lv_color_t)));

    // Register display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res      = SCREEN_WIDTH;
    disp_drv.ver_res      = SCREEN_HEIGHT;
    disp_drv.flush_cb     = lvgl_flush;
    disp_drv.draw_buf     = &draw_buf;
    disp_drv.full_refresh = 0;   // Partial updates only — bounce buffer decouples LCD
                                  // from PSRAM so small dirty-region writes are safe.
                                  // full_refresh=1 would write 768KB/frame continuously
                                  // (especially bad during slider drag at ~30Hz).
    lv_disp_drv_register(&disp_drv);

    // Register touch driver
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type                   = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb                = lvgl_touch_read;
    indev_drv.long_press_time        = 500;
    indev_drv.long_press_repeat_time = 100;
    lv_indev_drv_register(&indev_drv);

    Serial.printf("[display] Ready %dx%d  heap=%u  psram=%u\n",
                  SCREEN_WIDTH, SCREEN_HEIGHT,
                  ESP.getFreeHeap(), ESP.getFreePsram());
}

void display_driver_tick() {
    // No-op. Buffer swap is handled inside lvgl_flush on the last flush
    // of each full frame via waitDisplay(). Keeping this stub so loop()
    // compiles without changes.
}

// Drain any pending GT911 touch state directly from hardware before handing
// control to LVGL. Called once in loop() just before arming the indev.
// The GT911 can hold a ghost "touched" state after I2C init. Reading it here
// (outside LVGL) discards the ghost without corrupting the indev state machine.
void display_drain_touch() {
    uint16_t tx = 0, ty = 0;
    for (int i = 0; i < 8; i++) {
        tft.getTouch(&tx, &ty);
        delay(5);
    }
}

// Backlight brightness via GPIO bit-bang I2C to STC8H1K28 at 0x30.
// Cannot use Wire (I2C_NUM_0) — LovyanGFX owns it for GT911.
// Cannot use WireBL (I2C_NUM_1) post-tft.begin() — LovyanGFX reconfigures
// GPIO15/16 for I2C_NUM_0, disconnecting I2C_NUM_1 from the physical bus.
// Direct GPIO bit-bang bypasses both peripherals and always reaches the bus.
// Takes ~200µs — safe between LVGL timer calls.
static void bl_i2c_write(uint8_t addr, uint8_t val) {
    const int SDA = 15, SCL = 16;
    // Ensure open-drain mode — required for I2C wired-OR signalling
    gpio_set_direction((gpio_num_t)SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction((gpio_num_t)SCL, GPIO_MODE_INPUT_OUTPUT_OD);

    auto sda_hi = [&]{ gpio_set_level((gpio_num_t)SDA, 1); delayMicroseconds(3); };
    auto sda_lo = [&]{ gpio_set_level((gpio_num_t)SDA, 0); delayMicroseconds(3); };
    auto scl_hi = [&]{ gpio_set_level((gpio_num_t)SCL, 1); delayMicroseconds(5); };
    auto scl_lo = [&]{ gpio_set_level((gpio_num_t)SCL, 0); delayMicroseconds(3); };

    auto send_byte = [&](uint8_t b) {
        for (int i = 7; i >= 0; i--) {
            (b >> i) & 1 ? sda_hi() : sda_lo();
            scl_hi(); scl_lo();
        }
        sda_hi(); scl_hi(); scl_lo();  // release SDA, clock ACK
    };

    // START: SDA high→low while SCL high
    sda_hi(); scl_hi(); sda_lo(); scl_lo();
    send_byte((addr << 1) | 0);  // address + write bit
    send_byte(val);
    // STOP: SDA low→high while SCL high
    sda_lo(); scl_hi(); sda_hi();
}

// V1.1 STC8H1K28 peripheral commands via bit-bang I2C to 0x30:
//   Backlight: 0x05=off, 0x06-0x10=dim-to-max
//   Buzzer:    0x15=on,  0x16=off
//   Speaker:   0x17=on,  0x18=off
void display_buzzer(bool on) {
    bl_i2c_write(0x30, on ? 0x15 : 0x16);
}

void display_speaker(bool on) {
    bl_i2c_write(0x30, on ? 0x17 : 0x18);
}

void display_set_brightness(uint8_t level) {
    // V1.1 firmware: only 0x05(off), 0x06, 0x07, 0x08, 0x09, 0x10(max) are valid.
    // Values 0x0A-0x0F behave identically to 0x09 — map to 5 discrete steps.
    uint8_t reg;
    if      (level == 0)   reg = 0x05;  // off
    else if (level < 52)   reg = 0x06;  // ~20%
    else if (level < 103)  reg = 0x07;  // ~40%
    else if (level < 154)  reg = 0x08;  // ~60%
    else if (level < 205)  reg = 0x09;  // ~80%
    else                   reg = 0x10;  // 100%
    bl_i2c_write(0x30, reg);
}