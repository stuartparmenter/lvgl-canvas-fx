// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "fx_base.h"
#include <cstdint>

extern "C" {
#include <lvgl.h>
}

namespace esphome {
namespace lvgl_canvas_fx {

class FxAurora : public FxBase {
 public:
  void on_bind(lv_obj_t *canvas) override;
  void on_resize(const Rect &r) override;
  void step(float dt) override;

  // Optional tuning knobs
  void set_speed(float s) { speed_ = (s <= 0.f) ? 0.01f : s; }
  void set_scale(uint8_t s) { scale_ = s; }          // 8..64 typical
  void set_intensity(uint8_t i) { intensity_ = i; }  // 64..255

 private:
  // ---- One-time builders ----
  void build_sin_lut_();
  void build_palette_();
  void build_noise_lut_();

  // ---- Helpers ----
  static inline uint8_t hash8_(int x, int y);
  inline uint8_t value_noise8_fast_(int u_q8_8, int v_q8_8, uint8_t scale);

#if LV_COLOR_DEPTH == 16
  // Optimized batch processing
  inline void process_4_pixels_dsp_(uint8_t *out, int xx, int syb, int v, uint8_t ang_t1, int u_scale,
                                    int intensity_scaled);

  inline void process_2_pixels_fast_(uint8_t *out, int xx, int syb, int v, uint8_t ang_t1, int u_scale,
                                     int intensity_scaled);
#endif

  // ---- State ----
  bool ready_{false};
  float t_{0.f};
  float speed_{0.06f};
  uint8_t scale_{24};
  uint8_t intensity_{192};
  uint8_t pal_shift_{0};

  // ---- Format info (cached at bind) ----
  int bpp_{0};
  bool has_alpha_{false};

  // ---- LUTs (built at bind) ----
  int16_t sin_q15_[256]{};       // [-32767..+32767]
  uint8_t noise_lut_[64][64]{};  // Pre-computed noise grid
  uint8_t smooth_lut_[256]{};    // Smooth interpolation curve

// Platform-specific pre-computed palettes
#if LV_COLOR_DEPTH == 16
  uint8_t pal16_byte0_[256]{};
  uint8_t pal16_byte1_[256]{};
#elif LV_COLOR_DEPTH == 24
  uint8_t pal_rgb_[256][3]{};  // Pre-packed RGB888
#elif LV_COLOR_DEPTH == 32
  uint32_t pal_native_32_[256]{};  // Pre-packed ARGB8888
#else
  lv_color_t pal_[256]{};  // Fallback to LVGL colors
#endif
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
