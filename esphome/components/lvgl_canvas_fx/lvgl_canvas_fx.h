// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/automation.h"   // ← must be included before subclassing Action<>

#include <memory>
#include <string>

#include "fx_base.h"
#include "fx_registry.h"

extern "C" {
  #include <lvgl.h>
}

#ifndef LVGL_CANVAS_FX_METRICS
#define LVGL_CANVAS_FX_METRICS 1
#endif

namespace esphome {
namespace lvgl_canvas_fx {

class LvglCanvasFx : public PollingComponent {
 public:
  // Lifecycle
  void setup() override;
  void update() override;
  void dump_config() override;

  // Codegen wiring: on 2025.8.2 pass the raw lv_obj_t* (not LvCompound*)
  void setup_binding(lv_obj_t* canvas_obj,
                     const std::string &effect_key,
                     int x, int y, int w, int h,
                     bool start_paused);

  // Runtime control
  void pause()  { running_ = false; this->set_update_interval(paused_period_ms_); }
  void resume() { running_ = true;  this->set_update_interval(period_ms_); rebind_ = true; }
  void toggle() { running_ ? pause() : resume(); }
  bool is_running() const { return running_; }

  // Per-instance timing
  void set_initial_period(uint32_t ms) { period_ms_ = ms; if (running_) this->set_update_interval(period_ms_); }
  void set_fps(float fps);
  void set_paused_period_ms(uint32_t ms) { paused_period_ms_ = ms; if (!running_) this->set_update_interval(paused_period_ms_); }

 protected:
  struct Area { int x{0}, y{0}, w{0}, h{0}; };

  bool ensure_bound_();
  bool read_canvas_size_(uint16_t &w, uint16_t &h);
  void on_canvas_size_change_();

  // Bound canvas & effect
  lv_obj_t* canvas_{nullptr};           // single declaration
  std::string effect_key_;
  std::unique_ptr<FxBase> fx_;

  // Sub-rectangle (w/h == 0 => follow canvas size)
  Area area_{};

  // State
  bool running_{true};
  bool rebind_{false};
  uint16_t last_w_{0}, last_h_{0};

  // Timing
  uint32_t period_ms_{33};
  uint32_t paused_period_ms_{500};
  uint64_t last_us_{0};

#if LVGL_CANVAS_FX_METRICS
  // ---- Metrics window (printed every ~5s) ----
  struct {
    uint64_t window_start_us{0};
    uint64_t last_tick_us{0};      // end-of-update timestamp
    uint32_t frames{0};
    uint64_t step_us_sum{0};
    uint32_t step_us_max{0};
    uint64_t loop_us_sum{0};
    uint32_t loop_us_max{0};
    uint32_t overruns{0};
  } m_{};

  static constexpr uint32_t METRICS_PERIOD_MS = 5000;
  void metrics_log_and_roll_(uint64_t now_us);
#endif
};

// -------- Automation actions (per-instance) --------
class PauseAction : public Action<> {
 public:
  void set_target(LvglCanvasFx* t){ t_ = t; }
  void play() override { if (t_) t_->pause(); }
 private:
  LvglCanvasFx* t_{nullptr};
};

class ResumeAction : public Action<> {
 public:
  void set_target(LvglCanvasFx* t){ t_ = t; }
  void play() override { if (t_) t_->resume(); }
 private:
  LvglCanvasFx* t_{nullptr};
};

class ToggleAction : public Action<> {
 public:
  void set_target(LvglCanvasFx* t){ t_ = t; }
  void play() override { if (t_) t_->toggle(); }
 private:
  LvglCanvasFx* t_{nullptr};
};

class SetFpsAction : public Action<> {
 public:
  void set_target(LvglCanvasFx* t){ t_ = t; }
  void set_fps(float f){ fps_ = f; }
  void play() override { if (t_) t_->set_fps(fps_); }
 private:
  LvglCanvasFx* t_{nullptr};
  float fps_{30.0f};
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
