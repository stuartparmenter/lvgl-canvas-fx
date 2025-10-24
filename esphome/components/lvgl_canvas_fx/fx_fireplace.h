// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "fx_base.h"
#include <vector>
#include "esphome/core/helpers.h"

extern "C" {
#include <lvgl.h>
}

namespace esphome {
namespace lvgl_canvas_fx {

class FxFireplace : public FxBase {
 public:
  void on_bind(lv_obj_t *canvas) override { FxBase::on_bind(canvas); }
  void on_resize(const Rect &r) override;
  void step(float dt) override;

 private:
  static constexpr int FEED_MIN = 34;
  static constexpr int FEED_MAX = 36;
  static constexpr int COOL_GRAD = 0;
  static constexpr int COOL_RAND_MAX = 1;
  static constexpr int STEPS_PER_FRAME = 2;
  static constexpr int EMBER_RATE = 3;
  static constexpr int LOG_HEIGHT = 2;

  static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

  void build_palette_();  // 0..36 inclusive
  void sim_step_();       // update heat field once
  void draw_frame_();     // write to canvas buffer

  int W_{0}, H_{0};
  using HeatVec = std::vector<uint8_t, esphome::RAMAllocator<uint8_t>>;
  HeatVec heat_;              // size = W_*H_, values 0..36
  lv_color_t palette_[37]{};  // precomputed
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
