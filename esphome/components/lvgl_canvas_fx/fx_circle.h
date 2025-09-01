// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "fx_base.h"

extern "C" {
  #include <lvgl.h>
}

namespace esphome {
namespace lvgl_canvas_fx {

// Set to 0 if you want to disable raw row-by-row clears
#ifndef FX_CIRCLE_FAST_CLEAR
#define FX_CIRCLE_FAST_CLEAR 1
#endif

class FxCircle : public FxBase {
 public:
  void on_bind(lv_obj_t* canvas) override;
  void on_resize(const Rect &r) override;
  void step(float dt) override;

 private:
#if FX_CIRCLE_FAST_CLEAR
  void fast_clear_rect_(int x, int y, int w, int h);
#endif

  lv_draw_rect_dsc_t dsc_bg_{};
  lv_draw_rect_dsc_t dsc_fg_{};
  bool dsc_init_{false};

  int prev_cx_{-1};
  int prev_cy_{-1};
  int prev_r_{-1};

  float t_{0.0f};
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
