// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "fx_aurora.h"
#include <algorithm>
#include <cstring>
#include <cmath>
#include "esphome/core/log.h"
#include "esp_dsp.h"

extern "C" {
  #include <lvgl.h>
}

static const char *const TAG = "fx_aurora";

namespace esphome {
namespace lvgl_canvas_fx {

// -------------------- Builders --------------------

void FxAurora::build_sin_lut_() {
  for (int i = 0; i < 256; ++i) {
    float s = std::sinf(6.283185307179586f * (float)i / 256.0f);
    int v = (int) std::lroundf(s * 32767.0f);
    if (v >  32767) v =  32767;
    if (v < -32767) v = -32767;
    sin_q15_[i] = (int16_t) v;
  }
}

void FxAurora::build_palette_() {
  // 256-step gradient: teal -> blue -> purple
  for (int i = 0; i < 256; ++i) {
    float t = i / 255.0f;
    uint8_t r, g, b;
    if (t < 0.5f) {
      float u = t * 2.0f;
      r = (uint8_t)(0.0f   + u * 40.0f);
      g = (uint8_t)(200.0f - u * 110.0f);
      b = (uint8_t)(160.0f + u * 95.0f);
    } else {
      float u = (t - 0.5f) * 2.0f;
      r = (uint8_t)(40.0f  + u * 140.0f);
      g = (uint8_t)(90.0f  - u * 30.0f);
      b = (uint8_t)(255.0f - u * 55.0f);
    }

    #if LV_COLOR_DEPTH == 16
      // Use LVGL's channel macros to derive RGB,
      // then pack to 565 and choose bytes based on LV_COLOR_16_SWAP.
      lv_color_t c = lv_color_make(r, g, b);
      uint8_t R = LV_COLOR_GET_R(c);
      uint8_t G = LV_COLOR_GET_G(c);
      uint8_t B = LV_COLOR_GET_B(c);

      uint16_t r5 = (R >> 3) & 0x1F;
      uint16_t g6 = (G >> 2) & 0x3F;
      uint16_t b5 = (B >> 3) & 0x1F;
      uint16_t rgb565 = (uint16_t)((r5 << 11) | (g6 << 5) | b5);

      #if LV_COLOR_16_SWAP
        // LVGL stores high byte first in memory when swap=1
        pal16_byte0_[i] = (uint8_t)(rgb565 >> 8);
        pal16_byte1_[i] = (uint8_t)(rgb565 & 0xFF);
      #else
        // Standard little-endian memory order
        pal16_byte0_[i] = (uint8_t)(rgb565 & 0xFF);
        pal16_byte1_[i] = (uint8_t)(rgb565 >> 8);
      #endif

    #elif LV_COLOR_DEPTH == 24
      pal_rgb_[i][0] = r;
      pal_rgb_[i][1] = g;
      pal_rgb_[i][2] = b;

    #elif LV_COLOR_DEPTH == 32
      pal_native_32_[i] = (0xFFu << 24) | (uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b;
    #endif
  }
}

void FxAurora::build_noise_lut_() {
  for (int y = 0; y < 64; ++y) {
    for (int x = 0; x < 64; ++x) {
      noise_lut_[y][x] = hash8_(x, y);
    }
  }
  for (int i = 0; i < 256; ++i) {
    int t = i;
    int t2 = (t * t) >> 8;
    int t3 = (t2 * t) >> 8;
    smooth_lut_[i] = (uint8_t)((3 * t2 - 2 * t3) & 0xFF);
  }
}

// -------------------- Bind/Resize --------------------

void FxAurora::on_bind(lv_obj_t* canvas) {
  FxBase::on_bind(canvas);
  const lv_img_dsc_t *img = (const lv_img_dsc_t*) lv_canvas_get_img(canvas_);
  
  switch (img->header.cf) {
    case LV_IMG_CF_TRUE_COLOR:
    case LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED:
      #if LV_COLOR_DEPTH == 16
        bpp_ = 2;
      #elif LV_COLOR_DEPTH == 24  
        bpp_ = 3;
      #elif LV_COLOR_DEPTH == 32
        bpp_ = 4;
      #endif
      has_alpha_ = false;
      break;
    case LV_IMG_CF_TRUE_COLOR_ALPHA:
      #if LV_COLOR_DEPTH == 16
        bpp_ = 3;  // 2 bytes color + 1 byte alpha
      #elif LV_COLOR_DEPTH == 24
        bpp_ = 4;
      #elif LV_COLOR_DEPTH == 32
        bpp_ = 4;
      #endif
      has_alpha_ = true;
      break;
    default:
      ESP_LOGW(TAG, "Unsupported canvas format: %d", (int)img->header.cf);
      return;
  }

  ESP_LOGD(TAG, "canvas cf=%d bpp=%d alpha=%d", (int)img->header.cf, bpp_, has_alpha_);

  build_sin_lut_();
  build_palette_();
  build_noise_lut_();

  lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);
  ready_ = true;
}

void FxAurora::on_resize(const Rect &r) {
  FxBase::on_resize(r);
  lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);
}

// -------------------- Helpers --------------------

inline uint8_t FxAurora::hash8_(int x, int y) {
  uint32_t h = (uint32_t)(x) * 0x27d4eb2du ^ (uint32_t)(y) * 0x9e3779b1u;
  h ^= (h >> 16);
  h *= 0x7feb352du;
  h ^= (h >> 15);
  return (uint8_t) h;
}

inline uint8_t FxAurora::value_noise8_fast_(int u_q8_8, int v_q8_8, uint8_t scale) {
  int s = std::max<int>(1, scale);
  int xi = (u_q8_8 >> 8) / s;
  int yi = (v_q8_8 >> 8) / s;

  uint8_t a = noise_lut_[yi & 63][xi & 63];
  uint8_t b = noise_lut_[yi & 63][(xi + 1) & 63];
  uint8_t c = noise_lut_[(yi + 1) & 63][xi & 63];
  uint8_t d = noise_lut_[(yi + 1) & 63][(xi + 1) & 63];

  int cell_u = ((u_q8_8 >> 8) % s) * 256 / s;
  int cell_v = ((v_q8_8 >> 8) % s) * 256 / s;

  int su = smooth_lut_[cell_u];
  int sv = smooth_lut_[cell_v];

  int ab = a + (((b - a) * su) >> 8);
  int cd = c + (((d - c) * su) >> 8);
  return (uint8_t)(ab + (((cd - ab) * sv) >> 8));
}

#if LV_COLOR_DEPTH == 16
// Batch process 4 pixels using ESP-DSP (write BYTES)
inline void FxAurora::process_4_pixels_dsp_(
    uint8_t* out, 
    int xx,
    int syb, int v,
    uint8_t ang_t1,
    int u_scale,
    int intensity_scaled) {

  float values[4];
  float scale_factor = intensity_scaled / 256.0f;

  for (int i = 0; i < 4; ++i) {
    uint8_t ax = (uint8_t)(((xx + i) * 5 + ang_t1) & 0xFF);
    int16_t sx = sin_q15_[ax];
    int sxb = ((sx + 32768) >> 8);
    int u = (xx + i) * u_scale;
    uint8_t n = value_noise8_fast_(u, v, scale_);
    values[i] = (sxb * 3 + syb * 2 + n * 3) * scale_factor;
  }

  dsps_mulc_f32_ansi(values, values, 4, 1.0f, 1, 1);

  for (int i = 0; i < 4; ++i) {
    int val = (int)values[i];
    uint8_t idx = (uint8_t)((val + pal_shift_) & 0xFF);
    uint8_t* p = out + (i * 2);
    p[0] = pal16_byte0_[idx];
    p[1] = pal16_byte1_[idx];
  }
}

// Process 2 pixels (write BYTES)
inline void FxAurora::process_2_pixels_fast_(
    uint8_t* out,
    int xx,
    int syb, int v,
    uint8_t ang_t1,
    int u_scale,
    int intensity_scaled) {
  for (int i = 0; i < 2; ++i) {
    uint8_t ax = (uint8_t)(((xx + i) * 5 + ang_t1) & 0xFF);
    int16_t sx = sin_q15_[ax];
    int sxb = ((sx + 32768) >> 8);
    int u = (xx + i) * u_scale;
    uint8_t n = value_noise8_fast_(u, v, scale_);
    int val = ((sxb * 3) + (syb * 2) + (n * 3)) * intensity_scaled >> 8;
    uint8_t idx = (uint8_t)((val + pal_shift_) & 0xFF);
    uint8_t* p = out + (i * 2);
    p[0] = pal16_byte0_[idx];
    p[1] = pal16_byte1_[idx];
  }
}
#endif

// -------------------- Frame --------------------

void FxAurora::step(float dt) {
  if (!canvas_ || !ready_) return;

  const int cw = lv_obj_get_width(canvas_);
  const int ch = lv_obj_get_height(canvas_);
  if (cw <= 0 || ch <= 0) return;

  const int x0 = area_.x;
  const int y0 = area_.y;
  const int ww = (area_.w > 0) ? area_.w : cw;
  const int hh = (area_.h > 0) ? area_.h : ch;

  const lv_img_dsc_t *img = (const lv_img_dsc_t*) lv_canvas_get_img(canvas_);
  if (!img || !img->data) return;
  uint8_t *buf = const_cast<uint8_t*>(static_cast<const uint8_t*>(img->data));
  
  const int stride = cw * bpp_;
  
  if (x0 + ww > cw || y0 + hh > ch) {
    ESP_LOGW(TAG, "Render area exceeds canvas bounds");
    return;
  }

  // Advance time & palette drift
  t_ += dt * speed_;
  int pal_shift_delta = (int)(dt * 32.0f);
  pal_shift_ = (uint8_t)((pal_shift_ + pal_shift_delta) & 0xFF);

  const uint8_t ang_t0 = (uint8_t)((int)(t_ * 64.0f) & 0xFF);
  const uint8_t ang_t1 = (uint8_t)((int)(t_ * 113.0f) & 0xFF);

  const int u_scale = (ww > 1) ? (65536 / ww) : 65536;
  const int v_scale = (hh > 1) ? (65536 / hh) : 65536;
  const int intensity_scaled = (intensity_ * 32) >> 8;

  for (int yy = 0; yy < hh; ++yy) {
    uint8_t *row = buf + (y0 + yy) * stride + (x0 * bpp_);
    const uint8_t ay = (uint8_t)((yy * 3 + ang_t0) & 0xFF);
    const int16_t sy = sin_q15_[ay];
    const int syb = ((sy + 32768) >> 8);
    const int v = yy * v_scale;

    int xx = 0;

    #if LV_COLOR_DEPTH == 16
    if (!has_alpha_) {
      uint8_t* row_bytes = row;  // 2 bytes per pixel, write explicitly
      for (; xx <= ww - 4; xx += 4) {
        process_4_pixels_dsp_(row_bytes + (xx * 2),
                              xx, syb, v, ang_t1, u_scale, intensity_scaled);
      }
      for (; xx <= ww - 2; xx += 2) {
        process_2_pixels_fast_(row_bytes + (xx * 2),
                               xx, syb, v, ang_t1, u_scale, intensity_scaled);
      }
    }
    #endif

    // Remaining pixels
    for (; xx < ww; ++xx) {
      const uint8_t ax = (uint8_t)((xx * 5 + ang_t1) & 0xFF);
      const int16_t sx = sin_q15_[ax];
      const int sxb = ((sx + 32768) >> 8);
      const int u = xx * u_scale;
      const uint8_t n = value_noise8_fast_(u, v, scale_);
      int val = ((sxb * 3) + (syb * 2) + (n * 3)) * intensity_scaled >> 8;
      const uint8_t idx = (uint8_t)((val + pal_shift_) & 0xFF);

      uint8_t *pixel_ptr = row + xx * bpp_;

      #if LV_COLOR_DEPTH == 16
        if (!has_alpha_) {
          pixel_ptr[0] = pal16_byte0_[idx];
          pixel_ptr[1] = pal16_byte1_[idx];
        } else {
          pixel_ptr[0] = pal16_byte0_[idx];
          pixel_ptr[1] = pal16_byte1_[idx];
          pixel_ptr[2] = 0xFF;
        }

      #elif LV_COLOR_DEPTH == 24
        if (!has_alpha_) {
          memcpy(pixel_ptr, pal_rgb_[idx], 3);
        } else {
          memcpy(pixel_ptr, pal_rgb_[idx], 3);
          pixel_ptr[3] = 0xFF;
        }

      #elif LV_COLOR_DEPTH == 32
        *((uint32_t*)pixel_ptr) = pal_native_32_[idx];
      #endif
    }
  }

  // Invalidate the updated region
  lv_area_t a;
  lv_area_set(&a,
              (lv_coord_t)x0, (lv_coord_t)y0,
              (lv_coord_t)(x0 + ww - 1), (lv_coord_t)(y0 + hh - 1));
  lv_obj_invalidate_area(canvas_, &a);
}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
