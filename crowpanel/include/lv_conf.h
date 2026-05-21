#if 1

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* =====================================================================
   COLOUR & DISPLAY
   ===================================================================== */
#define LV_COLOR_DEPTH      16
#define LV_COLOR_16_SWAP    1   // byte-swap RGB565 output; matches lgfx::swap565_t* in flush callback
                                 // required for correct PSRAM write-back on ESP32-S3 parallel RGB panels
#define LV_HOR_RES_MAX      800
#define LV_VER_RES_MAX      480

/* =====================================================================
   MEMORY  — use PSRAM for LVGL heap
   ===================================================================== */
#define LV_MEM_CUSTOM       1
#define LV_MEM_CUSTOM_INCLUDE  "esp32-hal-psram.h"
#define LV_MEM_CUSTOM_ALLOC    ps_malloc
#define LV_MEM_CUSTOM_REALLOC  ps_realloc
#define LV_MEM_CUSTOM_FREE     free

/* =====================================================================
   DISPLAY REFRESH
   ===================================================================== */
#define LV_DISP_DEF_REFR_PERIOD  10
#define LV_ANTIALIAS             1

/* =====================================================================
   TICK — use Arduino millis() as LVGL time source
   LV_TICK_CUSTOM 1 means LVGL calls the expression below instead of
   lv_tick_inc(). No ISR or manual tick call needed.
   Guards prevent redefinition warnings when PlatformIO passes these
   via command-line build flags as well.
   ===================================================================== */
#ifndef LV_TICK_CUSTOM
#define LV_TICK_CUSTOM              1
#endif
#ifndef LV_TICK_CUSTOM_INCLUDE
#define LV_TICK_CUSTOM_INCLUDE      <Arduino.h>
#endif
#ifndef LV_TICK_CUSTOM_SYS_TIME_EXPR
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

/* =====================================================================
   INPUT DEVICE
   ===================================================================== */
#define LV_INDEV_DEF_READ_PERIOD  16   // 60Hz touch polling

/* =====================================================================
   FONTS
   ===================================================================== */
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_20  1
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_MONTSERRAT_48  1

#define LV_FONT_DEFAULT  &lv_font_montserrat_16

/* =====================================================================
   WIDGETS
   ===================================================================== */
#ifndef LV_USE_ARC
#define LV_USE_ARC        1
#endif
#ifndef LV_USE_BTNMATRIX
#define LV_USE_BTNMATRIX  1
#endif
#ifndef LV_USE_IMG
#define LV_USE_IMG        1
#endif
#ifndef LV_USE_TEXTAREA
#define LV_USE_TEXTAREA   1
#endif

#define LV_USE_BAR        1
#define LV_USE_BTN        1
#define LV_USE_LABEL      1
#define LV_USE_SLIDER     1
#define LV_USE_SWITCH     1
#define LV_USE_TABVIEW    1
#define LV_USE_CALENDAR   0
#define LV_USE_CANVAS     0
#define LV_USE_CHECKBOX   0
#define LV_USE_CHART      0
#define LV_USE_DROPDOWN   0
#define LV_USE_LINE       0
#define LV_USE_METER      0
#define LV_USE_ROLLER     0
#define LV_USE_SPINBOX    0
#define LV_USE_TABLE      0
#define LV_USE_WIN        0

/* =====================================================================
   ANIMATIONS & EFFECTS
   ===================================================================== */
#define LV_USE_ANIMATION    1
#define LV_USE_SHADOW       0
#define LV_USE_BLEND_MODES  0
#define LV_USE_OPA_SCALE    0

/* =====================================================================
   LOGGING
   ===================================================================== */
#define LV_USE_LOG      1
#define LV_LOG_LEVEL    LV_LOG_LEVEL_ERROR
#define LV_LOG_PRINTF   0

/* =====================================================================
   MISC
   ===================================================================== */
#define LV_USE_PERF_MONITOR  0
#define LV_USE_MEM_MONITOR   0
#define LV_SPRINTF_CUSTOM    0
#define LV_USE_USER_DATA     1

#endif  /* LV_CONF_H */
#endif  /* enable */