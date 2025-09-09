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
#if LVGL_CANVAS_FX_METRICS
    // Initialize metrics window on first schedule
    m_.window_start_us = last_us_;
    m_.last_tick_us    = last_us_;
#endif
  });
}

// ---------- Data ingress ----------
void LvglCanvasFx::submit_data(const void* data, size_t bytes) {
  if (fx_) fx_->on_data(data, bytes);
}

void LvglCanvasFx::set_effect(const std::string &key) {
  if (key == effect_key_) return;
  effect_key_ = key;
  fx_.reset();         // drop old instance
  rebind_ = true;      // ensure ensure_bound_() runs next update
  ESP_LOGI("lvgl_canvas_fx", "Effect changed to '%s'; will rebind", effect_key_.c_str());
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
    if (!this->ensure_bound_()) return;
    rebind_ = false;
    last_w_ = cw; last_h_ = ch;
  } else if (have_size && (cw != last_w_ || ch != last_h_)) {
    last_w_ = cw; last_h_ = ch;
    this->on_canvas_size_change_();
  }

  if (!fx_) return;
  // Measure the effect step() duration
#if LVGL_CANVAS_FX_METRICS
  const uint64_t t0 = esp_timer_get_time();
#endif

  fx_->step(dt);

#if LVGL_CANVAS_FX_METRICS
  const uint64_t t1 = esp_timer_get_time();
  const uint32_t step_us = static_cast<uint32_t>(t1 - t0);

  // Loop interval (tick-to-tick), using end-of-previous update
  const uint32_t loop_us = static_cast<uint32_t>(t1 - m_.last_tick_us);
  m_.last_tick_us = t1;

  // Aggregate window stats
  m_.frames++;
  m_.step_us_sum += step_us;
  m_.loop_us_sum += loop_us;
  m_.step_us_max  = std::max(m_.step_us_max, step_us);
  m_.loop_us_max  = std::max(m_.loop_us_max, loop_us);
  if (step_us / 1000.0f > static_cast<float>(period_ms_)) m_.overruns++;

  // Periodic log/roll
  if (t1 - m_.window_start_us >= (uint64_t)METRICS_PERIOD_MS * 1000ULL) {
    metrics_log_and_roll_(t1);
  }
#endif
}

void LvglCanvasFx::dump_config() {
  uint16_t cw=0,ch=0; read_canvas_size_(cw,ch);
  ESP_LOGCONFIG("lvgl_canvas_fx",
    "LvglCanvasFx(%p): effect='%s' area=[%d,%d %dx%d] canvas=%ux%u period=%ums paused=%ums running=%s",
    this, effect_key_.c_str(), area_.x, area_.y, area_.w, area_.h,
    cw, ch, period_ms_, paused_period_ms_, running_ ? "true" : "false");
#if LVGL_CANVAS_FX_METRICS
  ESP_LOGCONFIG("lvgl_canvas_fx", "Metrics: enabled (period=%ums)", METRICS_PERIOD_MS);
#else
  ESP_LOGCONFIG("lvgl_canvas_fx", "Metrics: disabled (LVGL_CANVAS_FX_METRICS=0)");
#endif
}

#if LVGL_CANVAS_FX_METRICS
void LvglCanvasFx::metrics_log_and_roll_(uint64_t now_us) {
  if (m_.frames == 0) {
    m_.window_start_us = now_us;
    return;
  }
  const double win_s        = (now_us - m_.window_start_us) / 1e6;
  const double target_fps   = (period_ms_ > 0) ? (1000.0 / period_ms_) : 0.0;
  const double effective_fps= m_.frames / win_s;
  const double avg_step_ms  = (m_.step_us_sum / 1000.0) / m_.frames;
  const double avg_loop_ms  = (m_.loop_us_sum / 1000.0) / m_.frames;

  ESP_LOGD("lvgl_canvas_fx",
           "[metrics] eff=%.2ffps tgt=%.2ffps frames=%u "
           "step(avg/max)=%.3f/%.3f ms loop(avg/max)=%.3f/%.3f ms overruns=%u",
           effective_fps, target_fps, m_.frames,
           avg_step_ms, m_.step_us_max / 1000.0,
           avg_loop_ms, m_.loop_us_max / 1000.0,
           m_.overruns);

  // Roll the window
  m_.window_start_us = now_us;
  m_.frames = 0;
  m_.step_us_sum = 0;  m_.step_us_max = 0;
  m_.loop_us_sum = 0;  m_.loop_us_max = 0;
  m_.overruns = 0;
}
#endif

}  // namespace lvgl_canvas_fx
}  // namespace esphome
