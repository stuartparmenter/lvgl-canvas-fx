// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "lvgl_canvas_fx.h"
#include "register_builtin_effects.h"

#include "esp_timer.h"
#include "esp_random.h"
#include <algorithm>
#include <cmath>

namespace {
bool g_fx_registered = false;
inline void ensure_fx_registered_once() {
  if (!g_fx_registered) {
    esphome::lvgl_canvas_fx::register_builtin_effects();
    g_fx_registered = true;
  }
}
}  // namespace

namespace esphome {
namespace lvgl_canvas_fx {

void LvglCanvasFx::set_fps(float fps) {
  if (fps < 1.0f) fps = 1.0f;
  if (fps > 240.0f) fps = 240.0f;
  period_ms_ = (uint32_t) lroundf(1000.0f / fps);
  if (running_) this->set_update_interval(period_ms_);
}

void LvglCanvasFx::setup_binding(lv_obj_t* canvas_obj,
                                 const std::string &effect_key,
                                 int x, int y, int w, int h,
                                 bool start_paused) {
  canvas_ = canvas_obj;
  effect_key_ = effect_key;
  area_.x = x; area_.y = y; area_.w = w; area_.h = h;
  running_ = !start_paused;
  rebind_ = running_;
}

void LvglCanvasFx::setup() {
  ensure_fx_registered_once();

  const uint32_t active_period = running_ ? period_ms_ : paused_period_ms_;
  const uint32_t jitter_ms = (active_period > 0) ? (esp_random() % active_period) : 0;

  this->set_timeout(jitter_ms, [this, active_period]() {
    last_us_ = esp_timer_get_time();
    this->set_update_interval(active_period);
  });
}

bool LvglCanvasFx::read_canvas_size_(uint16_t &w, uint16_t &h) {
  if (!canvas_ || !lv_obj_is_valid(canvas_)) return false;
  lv_obj_update_layout(canvas_);
  w = (uint16_t) lv_obj_get_width(canvas_);
  h = (uint16_t) lv_obj_get_height(canvas_);
  return (w > 0 && h > 0);
}

void LvglCanvasFx::on_canvas_size_change_() {
  if (!fx_) return;
  uint16_t cw{0}, ch{0};
  if (!this->read_canvas_size_(cw, ch)) return;
  const int w = (area_.w > 0) ? area_.w : (int)cw;
  const int h = (area_.h > 0) ? area_.h : (int)ch;
  fx_->on_resize(FxBase::Rect{area_.x, area_.y, w, h});
}

bool LvglCanvasFx::ensure_bound_() {
  if (!canvas_ || !lv_obj_is_valid(canvas_)) return false;

  const lv_img_dsc_t *img = (const lv_img_dsc_t*) lv_canvas_get_img(canvas_);
  if (!img || !img->data) {
    ESP_LOGW("lvgl_canvas_fx", "Canvas image not ready yet; will retry");
    return false;
  }

  if (!fx_) {
    fx_ = FxRegistry::make(effect_key_);     // assign unique_ptr directly
    if (!fx_) {
      ESP_LOGE("lvgl_canvas_fx", "Effect '%s' not found", effect_key_.c_str());
      return false;
    }
    fx_->on_bind(canvas_);
  }
  this->on_canvas_size_change_();
  return true;
}

void LvglCanvasFx::update() {
  if (!running_) return;

  const uint64_t now = esp_timer_get_time();
  const float raw_dt = (last_us_ > 0) ? float(now - last_us_) / 1e6f
                                    : float(this->get_update_interval()) / 1000.0f;
  const float dt = std::min(std::max(raw_dt, 0.0f), 0.1f);  // max 100ms
  last_us_ = now;

  uint16_t cw{0}, ch{0};
  const bool have_size = this->read_canvas_size_(cw, ch);

  if (rebind_) {
    rebind_ = false;
    if (!this->ensure_bound_()) return;
    last_w_ = cw; last_h_ = ch;
  } else if (have_size && (cw != last_w_ || ch != last_h_)) {
    last_w_ = cw; last_h_ = ch;
    this->on_canvas_size_change_();
  }

  if (!fx_) return;
  fx_->step(dt);
}

void LvglCanvasFx::dump_config() {
  uint16_t cw=0,ch=0; read_canvas_size_(cw,ch);
  ESP_LOGCONFIG("lvgl_canvas_fx",
    "LvglCanvasFx(%p): effect='%s' area=[%d,%d %dx%d] canvas=%ux%u period=%ums paused=%ums running=%s",
    this, effect_key_.c_str(), area_.x, area_.y, area_.w, area_.h,
    cw, ch, period_ms_, paused_period_ms_, running_ ? "true" : "false");
}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
