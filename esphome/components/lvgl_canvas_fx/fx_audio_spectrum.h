// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "fx_base.h"

extern "C" {
#include <lvgl.h>
}

#include <vector>
#include <algorithm>
#include <math.h>
#include <stdint.h>

#include "esp_dsp.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome {
namespace lvgl_canvas_fx {

class FxAudioSpectrum : public FxBase {
 public:
  // ---- Lifecycle ----
  void on_bind(lv_obj_t *canvas) override;
  void on_resize(const Rect &r) override;
  void on_data(const void *data, size_t bytes) override;  // called from mic thread (no LVGL here)
  void step(float dt) override;                           // main thread: draw bars/peaks

  // ---- Optional tunables (call anytime on the main thread) ----
  void configure(int fft_n, int bars, float fs, float peak_decay, float smooth, float gain_db, float noise_db,
                 float fmin_hz, float fmax_frac);

  void set_colors(lv_color_t bar, lv_color_t peak) {
    bar_color_ = bar;
    peak_color_ = peak;
  }
  void set_gap_px(int gap) { gap_px_ = std::max(0, gap); }
  void set_round_to_mult8(bool v) { round_to_mult8_ = v; }

 private:
  // ---- Internal helpers ----
  void ensure_init_();
  void teardown_();
  void ensure_bar_geom_();  // recompute per area_

  // ---- Config (same semantics as page_eq.h) ----
  int fft_n_ = 512;
  int num_bars_ = 16;
  float fs_ = 16000.0f;
  float peak_decay_ = 0.02f;
  float smooth_ = 0.25f;
  float gain_db_ = 6.0f;
  float noise_db_ = -50.0f;
  float fmin_ = 250.0f;
  float fmax_frac_ = 0.45f;

  // Silence gating + low-shelf bias
  float noise_gate_db_ = -62.0f;
  float idle_decay_ = 0.04f;
  float bar0_scale_ = 0.50f;
  float bar1_scale_ = 0.70f;

  // Colors + geometry
  lv_color_t bar_color_ = lv_color_hex(0x00FFD0);
  lv_color_t peak_color_ = lv_color_hex(0xFF4000);
  int gap_px_ = 1;
  bool round_to_mult8_ = true;

  // ---- DSP state ----
  bool inited_ = false;
  float *twiddle_ = nullptr;

  std::vector<float> ring_;
  size_t rpos_ = 0;

  std::vector<float> window_;
  std::vector<float> fft_in_;  // interleaved complex [re, im]

  std::vector<float> bar_;   // [0..1]
  std::vector<float> peak_;  // [0..1]

  // Producer→consumer snapshot (thread-safe)
  std::vector<float> bar_copy_;
  std::vector<float> peak_copy_;
  SemaphoreHandle_t data_mutex_ = nullptr;
  bool data_ready_ = false;

  // HPF DC blocker
  float hp_prev_x_ = 0.0f, hp_prev_y_ = 0.0f;
  static constexpr float HP_A = 0.990f;

  // Diagnostics used by gating (always computed)
  float last_rms_ = 0.0f;
  float last_rms_db_ = -120.0f;
  float last_max_abs_ = 0.0f;

  // Draw state
  lv_draw_rect_dsc_t dsc_bar_{};
  lv_draw_rect_dsc_t dsc_peak_{};
  bool dsc_init_{false};

  // Cached per-frame geometry
  int bar_w_px_{1};
  int usable_w_{0};
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
