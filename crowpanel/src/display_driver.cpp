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

// ── LovyanGFX LGFX class ──────────────────────────────────────────────────────
class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Bus_RGB     _bus_instance;
    lgfx::Panel_RGB   _panel_instance;
    lgfx::Light_PWM   _light_instance;
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
            cfg.pin_d0  = GPIO_NUM_8;   // B0
            cfg.pin_d1  = GPIO_NUM_3;   // B1
            cfg.pin_d2  = GPIO_NUM_46;  // B2
            cfg.pin_d3  = GPIO_NUM_9;   // B3
            cfg.pin_d4  = GPIO_NUM_1;   // B4

            // Green channel
            cfg.pin_d5  = GPIO_NUM_5;   // G0
            cfg.pin_d6  = GPIO_NUM_6;   // G1
            cfg.pin_d7  = GPIO_NUM_7;   // G2
            cfg.pin_d8  = GPIO_NUM_15;  // G3
            cfg.pin_d9  = GPIO_NUM_16;  // G4
            cfg.pin_d10 = GPIO_NUM_4;   // G5

            // Red channel
            cfg.pin_d11 = GPIO_NUM_45;  // R0
            cfg.pin_d12 = GPIO_NUM_48;  // R1
            cfg.pin_d13 = GPIO_NUM_47;  // R2
            cfg.pin_d14 = GPIO_NUM_21;  // R3
            cfg.pin_d15 = GPIO_NUM_14;  // R4

            // Sync / control
            cfg.pin_henable = GPIO_NUM_40;  // DE
            cfg.pin_vsync   = GPIO_NUM_41;  // VSYNC
            cfg.pin_hsync   = GPIO_NUM_39;  // HSYNC
            cfg.pin_pclk    = GPIO_NUM_0;   // PCLK

            cfg.freq_write  = 14000000;     // 14 MHz — stable for this panel

            // Timing — confirmed from working community examples for CrowPanel 5"
            cfg.hsync_polarity    = 0;
            cfg.hsync_front_porch = 8;
            cfg.hsync_pulse_width = 4;
            cfg.hsync_back_porch  = 43;
            cfg.vsync_polarity    = 0;
            cfg.vsync_front_porch = 8;
            cfg.vsync_pulse_width = 4;
            cfg.vsync_back_porch  = 12;
            cfg.pclk_active_neg   = 1;
            cfg.de_idle_high      = 0;
            cfg.pclk_idle_high    = 0;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        // ── Backlight — GPIO2 via PWM ─────────────────────────────────────────
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl      = GPIO_NUM_2;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        // ── GT911 touch — I2C on IO19 (SDA) / IO20 (SCL) ─────────────────────
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
            cfg.pin_sda   = GPIO_NUM_19;
            cfg.pin_scl   = GPIO_NUM_20;
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

    // LovyanGFX init
    tft.begin();
    tft.setRotation(0);
    tft.setBrightness(0);   // keep dark during LVGL init
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

void display_set_brightness(uint8_t level) {
    tft.setBrightness(level);
}