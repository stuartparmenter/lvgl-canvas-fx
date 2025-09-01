// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
#include "fx_fireworks_physics.h"

extern "C" {
  #include <lvgl.h>
}
#include "esp_random.h"
#include <cmath>

namespace esphome {
namespace lvgl_canvas_fx {

// -------------------- ctor/dtor --------------------
FxFireworksPhysics::FxFireworksPhysics() {}
FxFireworksPhysics::~FxFireworksPhysics() {
  clear_items_();
  if (space_) { cpSpaceFree(space_); space_ = nullptr; }
}

// -------------------- helpers --------------------
float FxFireworksPhysics::frand_(float a, float b) {
  return a + (b - a) * (float)(esp_random() & 0xFFFF) / 65535.0f;
}
lv_color_t FxFireworksPhysics::pick_warm_() {
  static const lv_color_t WARM[] = {
    lv_color_hex(0xFFFFFF), lv_color_hex(0xFFF4CC), lv_color_hex(0xFFD27F),
    lv_color_hex(0xFFB84D), lv_color_hex(0xFF9E2D)
  };
  return WARM[esp_random() % (sizeof(WARM) / sizeof(WARM[0]))];
}
lv_color_t FxFireworksPhysics::pick_accent_() {
  static const lv_color_t ACCENT[] = {
    lv_color_hex(0x00E6E6), lv_color_hex(0x8A2BE2)
  };
  return ACCENT[esp_random() % (sizeof(ACCENT) / sizeof(ACCENT[0]))];
}
lv_color_t FxFireworksPhysics::tweak_(lv_color_t c) {
  uint8_t w = 16 + (esp_random() % 48);
  return lv_color_mix(lv_color_white(), c, w);
}

void FxFireworksPhysics::free_body_(cpBody* b) {
  if (!b) return;
  if (cpSpace* s = cpBodyGetSpace(b)) cpSpaceRemoveBody(s, b);
  cpBodyFree(b);
}

// -------------------- space / items --------------------
void FxFireworksPhysics::create_space_() {
  if (space_) return;
  space_ = cpSpaceNew();
  cpSpaceSetIterations(space_, 12);
  cpSpaceSetDamping(space_, (cpFloat)0.993);
  cpSpaceSetSleepTimeThreshold(space_, (cpFloat)0.30);
  cpSpaceSetGravity(space_, cpv((cpFloat)0, (cpFloat)grav_px_s2_));
}

void FxFireworksPhysics::clear_items_() {
  for (auto* br : items_) { if (br) { free_body_(br->body); delete br; } }
  items_.clear();
  inflight_ = 0;
}

// -------------------- size-aware parameterization --------------------
void FxFireworksPhysics::recompute_params_() {
  if (H_ == 0) return;

  // Density scaling: compare area to 64x64 baseline, clamp to [0.25..1.0]
  const float area = (float)W_ * (float)H_;
  const float base_area = 64.0f * 64.0f;
  const float area_scale = base_area / std::max(area, base_area); // <= 1
  density_scale_ = std::max(0.25f, std::min(1.0f, area_scale * density_target_));

  // Auto fade cadence for big canvases
  // ~<= 64k px => every frame, ~>128k px => every other, ~>256k px => every 3rd
  if (area <= 64.0f * 64.0f)      fade_every_n_ = 1;
  else if (area <= 128.0f * 128.0f) fade_every_n_ = 2;
  else                              fade_every_n_ = 3;

  // Gravity proportional to height → consistent look across sizes.
  // Baseline ~180 px/s^2 at 64px tall ⇒ ≈ 2.8125 * H
  const float g = 2.8125f * (float)H_;
  grav_px_s2_ = g;

  // Target apex ~62% up from launch y ⇒ v0 = sqrt(2 g Δh)
  const float delta_h = 0.62f * (float)H_;
  const float v0      = sqrtf(2.0f * g * delta_h);     // upward magnitude

  // Launch scatter around v0
  launch_vy_min_ = -1.10f * v0;
  launch_vy_max_ = -0.90f * v0;

  // Tiny horizontal wobble scales with height
  launch_vx_abs_ = 0.06f * (float)H_;

  // Burst timing around apex t = v0/g (ms)
  const float t_apex_s = v0 / g;
  const uint32_t t_ms  = (uint32_t)(t_apex_s * 1000.0f);
  burst_t_min_ms_      = (uint32_t)(0.90f * t_ms);
  burst_t_max_ms_      = (uint32_t)(1.10f * t_ms);

  // Fragment speeds scale with launch speed → consistent diameter
  frag_speed_min_      = 0.55f * v0;
  frag_speed_max_      = 0.95f * v0;
}

// -------------------- fast canvas view --------------------
void FxFireworksPhysics::refresh_canvas_view_() {
  img_ = (const lv_img_dsc_t*) lv_canvas_get_img(canvas_);
  buf_ = nullptr; stride_bytes_ = 0; fmt_ = BufFmt::FMT_NONE;
  if (!img_ || !img_->data || W_ <= 0 || H_ <= 0) return;

  if (img_->header.cf == LV_IMG_CF_TRUE_COLOR) {
#if LV_COLOR_DEPTH == 16
    fmt_ = BufFmt::FMT_565;
    buf_ = (uint8_t*) img_->data;
    stride_bytes_ = W_ * (int)sizeof(lv_color_t);
#elif LV_COLOR_DEPTH == 32
    fmt_ = BufFmt::FMT_32;
    buf_ = (uint8_t*) img_->data;
    stride_bytes_ = W_ * (int)sizeof(lv_color_t);
#else
    fmt_ = BufFmt::FMT_NONE;
#endif
  }
}

inline void FxFireworksPhysics::put_px_(int x, int y, lv_color_t c) {
  if (!canvas_ready_()) return;
  if ((unsigned)x >= (unsigned)W_ || (unsigned)y >= (unsigned)H_) return;
  auto* row = reinterpret_cast<lv_color_t*>(buf_ + y * stride_bytes_);
  row[x] = c;  // honors LV_COLOR_*_SWAP implicitly
}

void FxFireworksPhysics::clear_black_fast_() {
  if (!canvas_ready_()) return;
  // For TRUE_COLOR (565/32bpp), black is all zeros in lv_color_t memory
  memset(buf_, 0x00, (size_t)stride_bytes_ * H_);
}

void FxFireworksPhysics::fade_to_black_fast_(uint8_t opa) {
  if (!canvas_ready_()) return;
  if (opa == LV_OPA_TRANSP) return;
  if (opa >= LV_OPA_COVER) { clear_black_fast_(); return; }

  const uint8_t a = (uint8_t)(255u - (uint32_t)opa);

#if LV_COLOR_DEPTH == 16 && !(defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP)
  // Fast path only when 565 is not byte-swapped.
  uint16_t* p = reinterpret_cast<uint16_t*>(buf_);
  const size_t n = (size_t)W_ * H_;
  for (size_t i = 0; i < n; i++) {
    uint16_t c = p[i];
    uint32_t r = (c >> 11) & 0x1F;
    uint32_t g = (c >>  5) & 0x3F;
    uint32_t b =  c        & 0x1F;
    r = (r * a + 127) >> 8;
    g = (g * a + 127) >> 8;
    b = (b * a + 127) >> 8;
    p[i] = (uint16_t)((r << 11) | (g << 5) | b);
  }
#else
  // Swap-safe generic path (works for 565+swap and 32-bit).
  auto* row = reinterpret_cast<lv_color_t*>(buf_);
  const size_t n = (size_t)W_ * H_;
  const lv_color_t black = lv_color_black();
  for (size_t i = 0; i < n; i++) row[i] = lv_color_mix(black, row[i], a);
#endif
}

// -------------------- spawning / bursting --------------------
FxFireworksPhysics::BodyRef* FxFireworksPhysics::spawn_particle_(
    float x, float y, float vx, float vy, lv_color_t base,
    uint8_t px, uint32_t life_ms, bool can_split, uint32_t split_ms) {
  if (!space_) return nullptr;
  BodyRef* br = new BodyRef();
  br->is_rocket = false;
  br->born_ms   = now_ms();
  br->life_ms   = life_ms;
  br->px        = px;
  br->color     = tweak_(base);
  br->can_split = can_split;
  br->split_ms  = split_ms;

  br->body = cpBodyNew((cpFloat)1.0, (cpFloat)INFINITY);  // no rotation
  cpBodySetPosition(br->body, cpv((cpFloat)x,(cpFloat)y));
  cpBodySetVelocity(br->body, cpv((cpFloat)vx,(cpFloat)vy));
  cpSpaceAddBody(space_, br->body);
  items_.push_back(br);
  return br;
}

FxFireworksPhysics::BodyRef* FxFireworksPhysics::spawn_core_(float cx, float cy, uint32_t life_ms, uint8_t px) {
  if (!space_) return nullptr;
  const float vx = frand_(-launch_vx_abs_, launch_vx_abs_);
  const float vy = frand_(launch_vy_min_, launch_vy_max_);

  BodyRef* core = new BodyRef();
  core->is_rocket = true;
  core->born_ms   = now_ms();
  core->fuse_ms   = (uint32_t)(frand_(0.85f, 1.15f) * (float)((burst_t_min_ms_ + burst_t_max_ms_) / 2));
  core->life_ms   = life_ms;
  core->px        = px;
  core->color     = lv_color_white();

  core->body = cpBodyNew((cpFloat)1.0, (cpFloat)INFINITY);
  cpBodySetPosition(core->body, cpv((cpFloat)cx,(cpFloat)cy));
  cpBodySetVelocity(core->body, cpv((cpFloat)vx,(cpFloat)vy));
  cpSpaceAddBody(space_, core->body);
  items_.push_back(core);
  inflight_++;
  return core;
}

FxFireworksPhysics::BurstKind FxFireworksPhysics::pick_burst_kind_() {
  uint8_t r = esp_random() % 100;
  if (r < 28) return BurstKind::PEONY;
  if (r < 45) return BurstKind::RING;
  if (r < 58) return BurstKind::STAR5;
  if (r < 68) return BurstKind::PALM;
  if (r < 79) return BurstKind::WILLOW;
  if (r < 88) return BurstKind::CHRYS;
  if (r < 96) return BurstKind::STAR6;
  return BurstKind::CROSSETTE;
}

// N.B. counts are scaled by density_scale_
void FxFireworksPhysics::burst_peony_(int cx, int cy, bool chrys) {
  bool accent = (frand_(0,1) < 1.0f/6.0f);
  lv_color_t base = accent ? pick_accent_() : pick_warm_();
  int   n   = 20 + (int)frand_(0,18);
  n = std::max(6, (int)std::round(n * density_scale_));
  const float s0  = frand_(frag_speed_min_*0.8f, frag_speed_max_*0.85f);
  const int   life= chrys ? (1100 + (int)frand_(0,600)) : (800 + (int)frand_(0,400));
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float jitter = frand_(-0.6f, 0.6f) * 0.02f;
    float sp = s0 * frand_(0.90f, 1.10f);
    float vx = cosf(a + jitter) * sp;
    float vy = sinf(a + jitter) * sp;
    uint8_t px = (frand_(0,1) < 0.25f) ? 2 : 1;
    spawn_particle_((float)cx, (float)cy, vx, vy, base, px, (uint32_t)life);
  }
}
void FxFireworksPhysics::burst_ring_(int cx, int cy) {
  bool accent = (frand_(0,1) < 0.33f);
  lv_color_t base = accent ? pick_accent_() : pick_warm_();
  int   n   = 26 + (int)frand_(0,16);
  n = std::max(8, (int)std::round(n * density_scale_));
  const float s   = frand_(frag_speed_min_*0.9f, frag_speed_max_*0.9f);
  const int   life= 850 + (int)frand_(0,450);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    spawn_particle_((float)cx, (float)cy, cosf(a)*s, sinf(a)*s, base, 1, (uint32_t)life);
  }
}
void FxFireworksPhysics::burst_starN_(int cx, int cy, int N) {
  lv_color_t base = pick_warm_();
  const float s   = frand_(frag_speed_min_*0.95f, frag_speed_max_);
  const int   life= 900 + (int)frand_(0,400);
  for (int i=0;i<N;i++) {
    float a = (2.0f * (float)M_PI * i) / N;
    int rays = 3 + (int)frand_(0,1);
    rays = std::max(2, (int)std::round(rays * density_scale_));
    for (int k=0;k<rays;k++) {
      float t = 0.55f + 0.18f * k; // 55%, 73%, 91%
      spawn_particle_((float)cx, (float)cy, cosf(a)*s*t, sinf(a)*s*t, base, 1, (uint32_t)life);
    }
  }
  burst_ring_(cx, cy);
}
void FxFireworksPhysics::burst_palm_(int cx, int cy) {
  lv_color_t base = pick_warm_();
  int fronds = 8 + (int)frand_(0,6);
  fronds = std::max(4, (int)std::round(fronds * density_scale_));
  const int life   = 1000 + (int)frand_(0,500);
  for (int i=0;i<fronds;i++) {
    float spread = (35.0f * (float)M_PI/180.0f);
    float a = -((float)M_PI/2) + frand_(-spread, spread);
    float s = frand_(frag_speed_min_*1.0f, frag_speed_max_*1.05f);
    spawn_particle_((float)cx, (float)cy, cosf(a)*s, sinf(a)*s, base, 2, (uint32_t)life);
  }
}
void FxFireworksPhysics::burst_willow_(int cx, int cy) {
  lv_color_t base = pick_warm_();
  int   n   = 18 + (int)frand_(0,10);
  n = std::max(6, (int)std::round(n * density_scale_));
  const float s   = frand_(frag_speed_min_*0.65f, frag_speed_min_*0.85f);
  const int   life= 1600 + (int)frand_(0,800);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    float vx = cosf(a) * s, vy = sinf(a) * s + 2.0f; // gentle droop
    spawn_particle_((float)cx, (float)cy, vx, vy, base, 1, (uint32_t)life);
  }
}
void FxFireworksPhysics::burst_crossette_(int cx, int cy) {
  lv_color_t base = pick_warm_();
  int   n   = 10 + (int)frand_(0,6);
  n = std::max(4, (int)std::round(n * density_scale_));
  const float s   = frand_(frag_speed_min_*0.9f, frag_speed_min_*1.2f);
  const int   life= 1100 + (int)frand_(0,300);
  for (int i=0;i<n;i++) {
    float a = (2.0f * (float)M_PI * i) / n;
    const uint32_t split_ms = 240 + (uint32_t)frand_(0,220);
    spawn_particle_((float)cx, (float)cy, cosf(a)*s, sinf(a)*s, base, 1, (uint32_t)life,
                    /*can_split=*/true, split_ms);
  }
}
void FxFireworksPhysics::burst_dispatch_(int cx, int cy, BurstKind k) {
  switch (k) {
    case BurstKind::PEONY:     burst_peony_(cx, cy, false); break;
    case BurstKind::CHRYS:     burst_peony_(cx, cy, true);  break;
    case BurstKind::RING:      burst_ring_(cx, cy);         break;
    case BurstKind::STAR5:     burst_starN_(cx, cy, 5);     break;
    case BurstKind::STAR6:     burst_starN_(cx, cy, 6);     break;
    case BurstKind::PALM:      burst_palm_(cx, cy);         break;
    case BurstKind::WILLOW:    burst_willow_(cx, cy);       break;
    case BurstKind::CROSSETTE: burst_crossette_(cx, cy);    break;
  }
}

// -------------------- per-tick pre-step mutations --------------------
void FxFireworksPhysics::handle_crossette_splits_(uint32_t tnow) {
  std::vector<BodyRef*> to_split; to_split.reserve(8);
  for (auto* it : items_) {
    if (!it || it->is_rocket || !it->body || it->split_done || !it->can_split) continue;
    if (tnow - it->born_ms >= it->split_ms) to_split.push_back(it);
  }
  for (auto* it : to_split) {
    const cpVect p = cpBodyGetPosition(it->body);
    const float sv = frand_(frag_speed_min_*0.55f, frag_speed_min_*0.75f);
    spawn_particle_((float)p.x, (float)p.y,  sv,  0, it->color, 1, 600);
    spawn_particle_((float)p.x, (float)p.y, -sv,  0, it->color, 1, 600);
    spawn_particle_((float)p.x, (float)p.y,  0,  sv, it->color, 1, 600);
    spawn_particle_((float)p.x, (float)p.y,  0, -sv, it->color, 1, 600);
    it->split_done = true;
    it->alive = false; // retire original shard
  }
}

void FxFireworksPhysics::handle_rocket_bursts_(uint32_t tnow) {
  std::vector<BodyRef*> to_burst; to_burst.reserve(4);
  for (auto* it : items_) {
    if (!it || !it->is_rocket || !it->body) continue;
    if (tnow - it->born_ms >= it->fuse_ms) to_burst.push_back(it);
  }
  for (auto* it : to_burst) {
    const cpVect p = cpBodyGetPosition(it->body);
    burst_dispatch_((int)p.x, (int)p.y, pick_burst_kind_());
    it->alive = false;
    if (inflight_) inflight_--;
  }
}

void FxFireworksPhysics::maybe_spawn_(uint32_t tnow) {
  // Effective MAX_INFLIGHT scaled by density (>=1 becomes 1, never zero)
  const uint8_t max_inflight = std::max<uint8_t>(1, (uint8_t)std::round(3.0f * density_scale_));
  if (inflight_ >= max_inflight) return;

  const uint32_t scaled_gap = std::max<uint32_t>(
      MIN_SPAWN_GAP_MS,
      (burst_t_min_ms_ + burst_t_max_ms_) / 3  // ~0.33 * time-to-apex
  );
  if (tnow - last_spawn_ms_ < scaled_gap) return;

  const float pad = 8.0f;
  const float cx = frand_(pad, std::max(pad, (float)W_ - pad));
  const float cy = (float)H_ - 2.0f; // near ground
  spawn_core_(cx, cy, 1300 + (uint32_t)frand_(0, 500), 2);
  last_spawn_ms_ = tnow;
}

// -------------------- draw --------------------
void FxFireworksPhysics::draw_px_rect_(int x, int y, uint8_t px, lv_color_t color) {
  // Fast path: direct writes
  if (canvas_ready_()) {
    const int W = static_cast<int>(W_);
    const int H = static_cast<int>(H_);
    if (x >= W || y >= H || x + (int)px <= 0 || y + (int)px <= 0) return;
    const int sx = (x < 0) ? 0 : x;
    const int sy = (y < 0) ? 0 : y;
    const int ex = std::min(W, x + (int)px);
    const int ey = std::min(H, y + (int)px);
    for (int yy = sy; yy < ey; ++yy) {
      for (int xx = sx; xx < ex; ++xx) put_px_(xx, yy, color);
    }
    return;
  }

  // Fallback: original LVGL draw (should rarely hit)
  if (!canvas_) return;
  const int W = lv_obj_get_width(canvas_), H = lv_obj_get_height(canvas_);
  if (x >= W || y >= H || x + (int)px <= 0 || y + (int)px <= 0) return;

  const int sx = (x < 0) ? 0 : x;
  const int sy = (y < 0) ? 0 : y;
  const int ex = std::min(W, x + (int)px);
  const int ey = std::min(H, y + (int)px);

  lv_draw_rect_dsc_t d; lv_draw_rect_dsc_init(&d);
  d.bg_color   = color;
  d.bg_opa     = LV_OPA_COVER;
  d.border_opa = LV_OPA_TRANSP;
  lv_canvas_draw_rect(canvas_, sx, sy, ex - sx, ey - sy, &d);
}

// -------------------- frame --------------------
void FxFireworksPhysics::render_() {
  if (!canvas_ || W_ == 0 || H_ == 0) return;

  // Trails / motion blur (skip according to fade_every_n_)
  if ((fade_tick_++ % fade_every_n_) == 0) {
    fade_to_black_fast_(fade_opa_);
  }

  for (auto* br : items_) {
    if (!br || !br->body) continue;
    const cpVect p = cpBodyGetPosition(br->body);
    draw_px_rect_((int)p.x, (int)p.y, br->px, br->color);
  }

  cull_and_free_();
  lv_obj_invalidate(canvas_);
}

void FxFireworksPhysics::cull_and_free_() {
  const uint32_t t = now_ms();
  items_.erase(std::remove_if(items_.begin(), items_.end(), [&](BodyRef* br){
    if (!br) return true;
    const bool too_old = (t - br->born_ms > (br->life_ms + SOFT_TTL_PAD_MS)) ||
                         (t - br->born_ms > HARD_TTL_MS);
    const cpVect p = br->body ? cpBodyGetPosition(br->body) : cpv(-9999, -9999);
    const bool oob = (p.x < -CULL_MARGIN) || (p.y < -CULL_MARGIN) ||
                     (p.x > W_ + CULL_MARGIN) || (p.y > H_ + CULL_MARGIN);
    if (too_old || oob || !br->alive) {
      if (br->is_rocket && inflight_ > 0) inflight_--;
      free_body_(br->body);
      delete br;
      return true;
    }
    return false;
  }), items_.end());
}

// -------------------- FxBase hooks --------------------
void FxFireworksPhysics::on_bind(lv_obj_t* canvas) {
  FxBase::on_bind(canvas);
  if (canvas_) lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);
  create_space_();
  refresh_canvas_view_();
}

void FxFireworksPhysics::on_resize(const Rect &r) {
  FxBase::on_resize(r);
  W_ = std::max(0, r.w);
  H_ = std::max(0, r.h);

  clear_items_();
  if (!canvas_ || W_ <= 0 || H_ <= 0) return;

  refresh_canvas_view_();
  if (!canvas_ready_()) return;
  lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);
  create_space_();
  recompute_params_();
}

void FxFireworksPhysics::step(float dt) {
  if (!canvas_ || !space_ || W_ <= 0 || H_ <= 0) return;

  // Fixed physics tick (~30 FPS), independent of component polling jitter
  acc_ms_ += dt * 1000.0f;
  while (acc_ms_ >= (float)TICK_MS) {
    const uint32_t tnow = now_ms();

    // 1) pre-step mutations
    handle_crossette_splits_(tnow);
    handle_rocket_bursts_(tnow);
    maybe_spawn_(tnow);

    // 2) step physics
    cpSpaceSetGravity(space_, cpv((cpFloat)0, (cpFloat)grav_px_s2_));
    cpSpaceStep(space_, (cpFloat)TICK_MS / (cpFloat)1000.0);

    // 3) render & cull
    render_();

    acc_ms_ -= (float)TICK_MS;
  }
}

} // namespace lvgl_canvas_fx
} // namespace esphome
