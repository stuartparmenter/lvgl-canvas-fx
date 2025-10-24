// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include "fx_base.h"

extern "C" {
#include <lvgl.h>
}
#include <chipmunk/chipmunk.h>

#include <vector>
#include <algorithm>
#include <cstdint>

namespace esphome {
namespace lvgl_canvas_fx {

class FxFireworksPhysics : public FxBase {
 public:
  FxFireworksPhysics();
  ~FxFireworksPhysics() override;

  void on_bind(lv_obj_t *canvas) override;
  void on_resize(const Rect &r) override;
  void step(float dt) override;

  // Optional runtime knobs (safe to ignore if you prefer the auto-tuning)
  void set_fade_every_n(uint8_t n) { fade_every_n_ = (n == 0) ? 1 : n; }
  // density_target in [0..1]; 1.0 = original density at 64x64, smaller lowers shard counts on large canvases
  void set_density_target(float s) {
    density_target_ = s;
    recompute_params_();
  }

 private:
  // ---------- Tunables (baseline) ----------
  static constexpr uint8_t TRAIL_FADE_OPA = 32;  // lower = longer trails
  static constexpr uint32_t TICK_MS = 33;        // ~30 FPS
  static constexpr int CULL_MARGIN = 2;
  static constexpr uint32_t HARD_TTL_MS = 3200;
  static constexpr uint32_t SOFT_TTL_PAD_MS = 800;
  static constexpr uint32_t MIN_SPAWN_GAP_MS = 300;

  // ---------- Small helpers ----------
  static inline uint32_t now_ms() { return lv_tick_get(); }
  static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

  // ---------- Data structures ----------
  struct BodyRef {
    cpBody *body{nullptr};
    bool is_rocket{false};
    bool alive{true};
    lv_color_t color{lv_color_white()};
    uint8_t px{1};
    uint32_t born_ms{0};
    uint32_t life_ms{1000};
    uint32_t fuse_ms{0};    // rocket-only
    bool can_split{false};  // crossette-only
    bool split_done{false};
    uint32_t split_ms{0};
  };

  // ---------- State ----------
  uint16_t W_{0}, H_{0};
  cpSpace *space_{nullptr};
  std::vector<BodyRef *> items_;
  uint8_t inflight_{0};
  uint32_t last_spawn_ms_{0};
  uint8_t fade_opa_{TRAIL_FADE_OPA};

  // tick accumulator (to keep fixed physics step)
  float acc_ms_{0.0f};

  // Canvas view cache (fast path)
  enum class BufFmt : uint8_t { FMT_NONE, FMT_565, FMT_32 };
  const lv_img_dsc_t *img_{nullptr};
  uint8_t *buf_{nullptr};
  int stride_bytes_{0};
  BufFmt fmt_{BufFmt::FMT_NONE};
  inline bool canvas_ready_() const { return canvas_ && img_ && buf_ && stride_bytes_ > 0 && fmt_ != BufFmt::FMT_NONE; }

  // Size-aware parameters (recomputed on resize)
  float grav_px_s2_{180.0f};  // px/s^2
  float launch_vy_min_{-220.0f};
  float launch_vy_max_{-180.0f};
  float launch_vx_abs_{15.0f};
  uint32_t burst_t_min_ms_{600};
  uint32_t burst_t_max_ms_{950};
  float frag_speed_min_{55.0f};
  float frag_speed_max_{85.0f};

  // Density auto-scaling
  // density_scale_ is applied to: number of shards per burst, and effective MAX_INFLIGHT
  float density_scale_{1.0f};   // computed from area vs 64x64
  float density_target_{1.0f};  // user override (multiplier, clamp [0.25..1.0])

  // Fading cadence
  uint8_t fade_every_n_{1};
  uint8_t fade_tick_{0};

  // ---------- Impl pieces ----------
  void create_space_();
  void clear_items_();
  static void free_body_(cpBody *b);

  static lv_color_t pick_warm_();
  static lv_color_t pick_accent_();
  static lv_color_t tweak_(lv_color_t c);

  static float frand_(float a, float b);

  // Fast canvas API
  void refresh_canvas_view_();
  void clear_black_fast_();               // memset clear (both 565 & 32-bit)
  void fade_to_black_fast_(uint8_t opa);  // fast fade (565 bit-math / 32-bit mix)
  inline void put_px_(int x, int y, lv_color_t c);

  // Drawing & frame
  void draw_px_rect_(int x, int y, uint8_t px, lv_color_t color);
  void render_();
  void cull_and_free_();

  // Param scaling by size
  void recompute_params_();

  // Spawning / bursting
  BodyRef *spawn_particle_(float x, float y, float vx, float vy, lv_color_t base, uint8_t px = 1,
                           uint32_t life_ms = 1000, bool can_split = false, uint32_t split_ms = 0);
  BodyRef *spawn_core_(float cx, float cy, uint32_t life_ms = 1500, uint8_t px = 2);

  enum class BurstKind : uint8_t { PEONY, CHRYS, RING, STAR5, STAR6, PALM, WILLOW, CROSSETTE };
  static BurstKind pick_burst_kind_();
  void burst_peony_(int cx, int cy, bool chrys = false);
  void burst_ring_(int cx, int cy);
  void burst_starN_(int cx, int cy, int N);
  void burst_palm_(int cx, int cy);
  void burst_willow_(int cx, int cy);
  void burst_crossette_(int cx, int cy);
  void burst_dispatch_(int cx, int cy, BurstKind k);

  // Per-tick “pre-step” mutations
  void handle_crossette_splits_(uint32_t tnow);
  void handle_rocket_bursts_(uint32_t tnow);
  void maybe_spawn_(uint32_t tnow);
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
