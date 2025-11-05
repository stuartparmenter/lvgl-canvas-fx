// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/automation.h"  // ← must be included before subclassing Action<>
#include "esphome/core/version.h"

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

class LvglCanvasFx : public Component {
 public:
  // Lifecycle
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BEFORE_CONNECTION; }

  // Codegen wiring: on 2025.8.2 pass the raw lv_obj_t* (not LvCompound*)
  void setup_binding(lv_obj_t *canvas_obj, const std::string &effect_key, int x, int y, int w, int h,
                     bool start_paused);

  // Runtime control
  void pause();
  void resume();
  void toggle() { running_ ? pause() : resume(); }
  bool is_running() const { return running_; }

  // ---- Data ingress ----
  // Submit arbitrary bytes to the active effect (if any).
  void submit_data(const void *data, size_t bytes);

  // Per-instance timing
  void set_initial_period(uint32_t ms) { period_ms_ = ms; }
  void set_fps(float fps);
  void set_effect(const std::string &key);

 protected:
  struct Area {
    int x{0}, y{0}, w{0}, h{0};
  };

  bool ensure_bound_();
  bool read_canvas_size_(uint16_t &w, uint16_t &h);
  void on_canvas_size_change_();
  void tick_(float dt);  // Execute one frame update

  // Bound canvas & effect
  lv_obj_t *canvas_{nullptr};  // single declaration
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
  uint64_t last_us_{0};

#if LVGL_CANVAS_FX_METRICS
  // ---- Metrics window (printed every ~5s) ----
  struct {
    uint64_t window_start_us{0};
    uint64_t last_tick_us{0};  // end-of-update timestamp
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
  void set_target(LvglCanvasFx *t) { t_ = t; }
  void play() override {
    if (t_)
      t_->pause();
  }

 private:
  LvglCanvasFx *t_{nullptr};
};

class ResumeAction : public Action<> {
 public:
  void set_target(LvglCanvasFx *t) { t_ = t; }
  void play() override {
    if (t_)
      t_->resume();
  }

 private:
  LvglCanvasFx *t_{nullptr};
};

class ToggleAction : public Action<> {
 public:
  void set_target(LvglCanvasFx *t) { t_ = t; }
  void play() override {
    if (t_)
      t_->toggle();
  }

 private:
  LvglCanvasFx *t_{nullptr};
};

class SetFpsAction : public Action<> {
 public:
  void set_target(LvglCanvasFx *t) { t_ = t; }
  void set_fps(float f) { fps_ = f; }
  void play() override {
    if (t_)
      t_->set_fps(fps_);
  }

 private:
  LvglCanvasFx *t_{nullptr};
  float fps_{30.0f};
};

template<typename... Ts> class SetEffectAction : public Action<Ts...> {
 public:
  void set_target(LvglCanvasFx *t) { t_ = t; }

  // One source of truth: templatable string named "effect"
  TEMPLATABLE_VALUE(std::string, effect)

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
  void play(const Ts&... x) override {
#else
  void play(Ts... x) override {
#endif
    if (!t_)
      return;
    const std::string key = this->effect_.value(x...);
    if (!key.empty())
      t_->set_effect(key);
  }

 private:
  LvglCanvasFx *t_{nullptr};
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
