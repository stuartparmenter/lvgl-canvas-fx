// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "fx_fireplace.h"
#include <algorithm>
#include <cmath>
#include <cstring> // memset

extern "C" {
  #include <lvgl.h>
}

namespace esphome {
namespace lvgl_canvas_fx {

void FxFireplace::on_resize(const Rect &r) {
  FxBase::on_resize(r);
  W_ = std::max(0, r.w);
  H_ = std::max(0, r.h);

  if (W_ <= 0 || H_ <= 0) {
    heat_.clear();
    return;
  }
  heat_.assign(static_cast<size_t>(W_) * static_cast<size_t>(H_), 0);
  build_palette_();
}

void FxFireplace::build_palette_() {
  // Color gradient from black → red → orange → yellow → white
  static const uint8_t K[][3] = {
    {  0,   0,   0},
    { 35,   0,   0},
    {180,   0,   0},
    {255,  80,   0},
    {255, 200,   0},
    {255, 255, 255}
  };
  auto lerp8 = [](uint8_t a, uint8_t b, float t)->uint8_t {
    float v = (1.0f - t) * a + t * b;
    if (v < 0) v = 0; if (v > 255) v = 255;
    return (uint8_t) std::lround(v);
  };

  const int steps = 36;
  for (int i = 0; i <= steps; ++i) {
    float u = (float)i / (float)steps;
    const int segs = 5;
    float s = u * segs;
    int si = (int)s; if (si >= segs) si = segs - 1;
    float tt = s - si;
    uint8_t r = lerp8(K[si][0], K[si+1][0], tt);
    uint8_t g = lerp8(K[si][1], K[si+1][1], tt);
    uint8_t b = lerp8(K[si][2], K[si+1][2], tt);
    palette_[i] = lv_color_make(r, g, b);
  }
}

void FxFireplace::sim_step_() {
  if (W_ <= 0 || H_ <= 0) return;

  // --- HOT FULL-WIDTH BOTTOM FEED ---
  // Every frame we inject heat at the bottom row.
  // Mostly uniform, but add a slight bias near the center so the flame column is taller there.
  for (int x = 0; x < W_; ++x) {
    int base = FEED_MIN + lv_rand(0, FEED_MAX - FEED_MIN); // 34..36
    int dx = x - (W_ / 2);
    if ((dx * dx) < (W_ * W_) / 18) base += 1; // hotter in the middle
    if (base > 36) base = 36;
    heat_[(H_-1)*W_ + x] = (uint8_t)base;
  }

  // --- RARE EMBERS ABOVE THE HEARTH ---
  // Occasionally pop a hot pixel higher up to look like sparks.
  if (lv_rand(0, EMBER_RATE) == 0) {
    int ex = (W_ > 2) ? lv_rand(1, W_-2) : 0;
    int ey_min = H_/2;
    int ey_max = (H_ > 10) ? (H_ - 10) : (H_ - 1);
    if (ey_max < ey_min) ey_min = ey_max;
    int ey = (ey_max >= 0) ? lv_rand(ey_min, ey_max) : 0;
    heat_[ey*W_ + ex] = 36;
  }

  // --- UPWARD PROPAGATION ---
  // Each cell is copied from the row below with a small sideways jitter,
  // so the flame flickers and leans left/right.
  // Occasional big sideways offsets create the taller “tongues.”
  for (int y = 0; y < H_-1; ++y) {
    uint8_t* row   = &heat_[y*W_];
    uint8_t* below = &heat_[(y+1)*W_];
    for (int x = 0; x < W_; ++x) {
      int ofs = lv_rand(0, 2) - 1;       // -1..+1 normal jitter
      if (lv_rand(0, 12) == 0) {         // ~8% chance of a bigger jump
        ofs += lv_rand(-2, 2);
      }
      int sx = clampi(x + ofs, 0, W_-1);

      // Minimal cooling → flames climb higher before dying
      int cool = COOL_GRAD + lv_rand(0, COOL_RAND_MAX);
      int v = (int)below[sx] - cool;
      if (v < 0) v = 0;
      row[x] = (uint8_t)v;
    }

    // --- QUICK HORIZONTAL SMOOTH ---
    // 1:2:1 filter to soften speckles and make flame surfaces smoother.
    uint8_t left = row[0];
    for (int x = 1; x < W_-1; ++x) {
      uint8_t cur = row[x];
      uint8_t nxt = row[x+1];
      row[x] = (uint8_t)((left + (cur<<1) + nxt) >> 2);
      left = cur;
    }
  }
}

void FxFireplace::draw_frame_() {
  if (!canvas_ || W_ <= 0 || H_ <= 0) return;

  // Access canvas buffer
  const lv_img_dsc_t* img = (const lv_img_dsc_t*) lv_canvas_get_img(canvas_);
  if (!img || !img->data) return;
  auto* buf = (lv_color_t*) img->data;

  // Map each heat cell (0..36) to a palette color
  for (int y = 0; y < H_; ++y) {
    int iy = y * W_;
    for (int x = 0; x < W_; ++x) {
      int idx = (int)heat_[iy + x];
      if (idx < 0) idx = 0; if (idx > 36) idx = 36;
      buf[iy + x] = palette_[idx];
    }
  }

  // --- “LOG” LINE AT THE BOTTOM ---
  // Always draw a 2-pixel strip of brownish pixels at the base
  // so the flames look like they’re rising from wood.
  int log_h = LOG_HEIGHT;
  if (log_h > H_) log_h = H_;
  for (int yb = H_ - log_h; yb < H_; ++yb) {
    if (yb < 0) continue;
    int iyb = yb * W_;
    for (int x = 0; x < W_; ++x) {
      uint8_t d = ((x ^ yb) & 3) ? 10 : 0;
      uint8_t r = 78 + d, g = 40 + (d>>1), b = 10;
      buf[iyb + x] = lv_color_make(r, g, b);
    }
  }

  lv_obj_invalidate(canvas_);
}

void FxFireplace::step(float /*dt*/) {
  if (!canvas_ || W_ <= 0 || H_ <= 0) return;

  // Run multiple simulation steps per frame to keep motion smooth & lively
  for (int i = 0; i < STEPS_PER_FRAME; ++i) {
    sim_step_();
  }
  draw_frame_();
}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
