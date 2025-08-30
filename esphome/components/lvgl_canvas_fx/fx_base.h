// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
extern "C" {
    #include <lvgl.h>
}

namespace esphome {
namespace lvgl_canvas_fx {

class FxBase {
 public:
  struct Rect { int x{0}, y{0}, w{0}, h{0}; };
  virtual ~FxBase() = default;

  // Called once when bound to a canvas (canvas already has a buffer)
  virtual void on_bind(lv_obj_t* canvas) { canvas_ = canvas; }

  // Called whenever the tracked sub-rect or canvas size changes
  virtual void on_resize(const Rect &r) { area_ = r; }

  // Called every tick (dt in seconds)
  virtual void step(float dt) = 0;

 protected:
  lv_obj_t* canvas_{nullptr};
  Rect area_{};
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
