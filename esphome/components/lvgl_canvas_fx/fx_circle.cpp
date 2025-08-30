// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "fx_circle.h"
#include "fx_registry.h"
#include <math.h>

extern "C" {
  #include <lvgl.h>
}

namespace esphome {
namespace lvgl_canvas_fx {

void FxCircle::step(float dt) {
  if (!canvas_) return;

  t_ += dt;

  // clear sub-rect
  lv_draw_rect_dsc_t bg; lv_draw_rect_dsc_init(&bg);
  bg.bg_color = lv_color_black();
  bg.bg_opa   = LV_OPA_COVER;
  bg.border_opa = LV_OPA_TRANSP;
  lv_canvas_draw_rect(canvas_, area_.x, area_.y, area_.w, area_.h, &bg);

  // radius anim
  const int maxr = std::max(1, std::min(area_.w, area_.h) / 2 - 1);
  constexpr float TAU = 6.283185307179586f;  // 2π
  const float phase = sinf(t_ * TAU * 0.5f); // 0.5 Hz
  const int r = 4 + (int)roundf(0.5f * (1.0f + phase) * (maxr - 4));

  // center
  const int cx = area_.x + area_.w / 2;
  const int cy = area_.y + area_.h / 2;

  // filled circle via rounded rect with LV_RADIUS_CIRCLE
  lv_draw_rect_dsc_t d;
  lv_draw_rect_dsc_init(&d);
  d.bg_color = lv_palette_main(LV_PALETTE_BLUE);
  d.bg_opa   = LV_OPA_COVER;
  d.border_opa = LV_OPA_TRANSP;
  d.radius = LV_RADIUS_CIRCLE;                 // ← key line
  const int side = r * 2;
  lv_canvas_draw_rect(canvas_, cx - r, cy - r, side, side, &d);

  // (optional) outline:
  // lv_draw_arc_dsc_t a; lv_draw_arc_dsc_init(&a);
  // a.color = lv_color_white(); a.width = 2;
  // lv_canvas_draw_arc(canvas_, cx, cy, r, 0, 360, &a);
}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
