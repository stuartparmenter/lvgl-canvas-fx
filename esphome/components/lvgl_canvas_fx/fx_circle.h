// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "fx_base.h"

namespace esphome {
namespace lvgl_canvas_fx {

class FxCircle : public FxBase {
 public:
  void step(float dt) override;

 private:
  float t_{0.0f};
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
