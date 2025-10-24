// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "fx_audio_spectrum.h"

extern "C" {
#include <lvgl.h>
}

namespace esphome {
namespace lvgl_canvas_fx {

void FxAudioSpectrum::configure(int fft_n, int bars, float fs, float peak_decay, float smooth, float gain_db,
                                float noise_db, float fmin_hz, float fmax_frac) {
  fft_n_ = fft_n;
  num_bars_ = bars;
  fs_ = fs;
  peak_decay_ = peak_decay;
  smooth_ = smooth;
  gain_db_ = gain_db;
  noise_db_ = noise_db;
  fmin_ = fmin_hz;
  fmax_frac_ = fmax_frac;

  teardown_();
  ensure_init_();
}

void FxAudioSpectrum::on_bind(lv_obj_t *canvas) {
  FxBase::on_bind(canvas);

  lv_draw_rect_dsc_init(&dsc_bar_);
  dsc_bar_.bg_opa = LV_OPA_COVER;
  dsc_bar_.border_opa = LV_OPA_TRANSP;
  dsc_bar_.bg_color = bar_color_;

  lv_draw_rect_dsc_init(&dsc_peak_);
  dsc_peak_.bg_opa = LV_OPA_COVER;
  dsc_peak_.border_opa = LV_OPA_TRANSP;
  dsc_peak_.bg_color = peak_color_;

  dsc_init_ = true;

  lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);

  ensure_init_();
  ensure_bar_geom_();
}

void FxAudioSpectrum::on_resize(const Rect &r) {
  FxBase::on_resize(r);
  ensure_bar_geom_();

  if (canvas_)
    lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);
}

void FxAudioSpectrum::ensure_bar_geom_() {
  if (area_.w <= 0)
    return;
  usable_w_ = area_.w;
  int gap = std::max(0, gap_px_);

  // If you want dynamic bars from width, enable this block:
  if (round_to_mult8_) {
    int candidate = (usable_w_ + gap) / (7 + gap);  // assume ~7px bar as baseline
    candidate = std::max(8, std::min(96, candidate));
    candidate = (candidate + 7) & ~7;
    if (candidate != num_bars_) {
      num_bars_ = candidate;
      // resize bar vectors but keep FFT params
      bar_.assign(num_bars_, 0.0f);
      peak_.assign(num_bars_, 0.0f);
      bar_copy_.assign(num_bars_, 0.0f);
      peak_copy_.assign(num_bars_, 0.0f);
    }
  }

  bar_w_px_ = (usable_w_ - gap * (num_bars_ - 1)) / std::max(1, num_bars_);
  if (bar_w_px_ < 1)
    bar_w_px_ = 1;
}

void FxAudioSpectrum::ensure_init_() {
  if (inited_)
    return;

  twiddle_ = (float *) heap_caps_malloc(fft_n_ * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!twiddle_) {
    twiddle_ = (float *) heap_caps_malloc(fft_n_ * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!twiddle_)
    return;

  if (dsps_fft2r_init_fc32(twiddle_, fft_n_) != ESP_OK) {
    heap_caps_free(twiddle_);
    twiddle_ = nullptr;
    return;
  }

  ring_.assign(fft_n_, 0.0f);
  rpos_ = 0;

  window_.resize(fft_n_);
  for (int n = 0; n < fft_n_; n++) {
    window_[n] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * n / (fft_n_ - 1)));
  }

  fft_in_.assign(fft_n_ * 2, 0.0f);

  bar_.assign(num_bars_, 0.0f);
  peak_.assign(num_bars_, 0.0f);
  bar_copy_.assign(num_bars_, 0.0f);
  peak_copy_.assign(num_bars_, 0.0f);

  hp_prev_x_ = hp_prev_y_ = 0.0f;

  if (data_mutex_ == nullptr)
    data_mutex_ = xSemaphoreCreateMutex();

  inited_ = true;
}

void FxAudioSpectrum::teardown_() {
  inited_ = false;

  ring_.clear();
  window_.clear();
  fft_in_.clear();
  bar_.clear();
  peak_.clear();
  bar_copy_.clear();
  peak_copy_.clear();

  if (twiddle_) {
    heap_caps_free(twiddle_);
    twiddle_ = nullptr;
  }
}

void FxAudioSpectrum::on_data(const void *data, size_t bytes) {
  // Mic thread: heavy DSP is OK here, but NO LVGL calls.
  ensure_init_();
  if (!inited_ || !data || bytes < 4)
    return;

  const int32_t *raw = static_cast<const int32_t *>(data);
  int nsamp = (int) (bytes / 4);
  if (nsamp <= 0)
    return;

  // Accumulate RMS + max; convert 24-bit LJ to float; DC high-pass
  double acc2 = 0.0;
  float max_abs = 0.0f;
  for (int i = 0; i < nsamp; i++) {
    float s = (float) (raw[i] / 256) * (1.0f / 8388608.0f);
    float y = s - hp_prev_x_ + HP_A * hp_prev_y_;
    hp_prev_x_ = s;
    hp_prev_y_ = y;
    s = y;

    acc2 += (double) s * (double) s;
    float a = fabsf(s);
    if (a > max_abs)
      max_abs = a;
  }
  last_rms_ = sqrt(acc2 / (double) nsamp);
  last_rms_db_ = 20.0f * log10f(last_rms_ + 1e-12f);
  last_max_abs_ = max_abs;

  // Ensure buffers sized
  if ((int) ring_.size() != fft_n_) {
    ring_.assign(fft_n_, 0.0f);
    rpos_ = 0;
  }
  if ((int) fft_in_.size() != fft_n_ * 2)
    fft_in_.assign(fft_n_ * 2, 0.0f);

  // Append recent time-domain samples (use latest HPF sample)
  for (int i = 0; i < nsamp; i++) {
    ring_[rpos_] = hp_prev_y_;
    rpos_ = (rpos_ + 1) % fft_n_;
  }

  // Copy most recent frame with Hann window; imag = 0
  for (int n = 0; n < fft_n_; n++) {
    int idx = (int) ((rpos_ + n) % fft_n_);
    float w = (window_.empty() ? 1.0f : window_[n]);
    fft_in_[2 * n + 0] = ring_[idx] * w;
    fft_in_[2 * n + 1] = 0.0f;
  }

  // FFT → real/imag bins
  dsps_fft2r_fc32(fft_in_.data(), fft_n_);
  dsps_bit_rev_fc32(fft_in_.data(), fft_n_);
  dsps_cplx2reC_fc32(fft_in_.data(), fft_n_);

  // Bars
  if ((int) bar_.size() != num_bars_)
    bar_.assign(num_bars_, 0.0f);
  if ((int) peak_.size() != num_bars_)
    peak_.assign(num_bars_, 0.0f);

  const int n_bins = fft_n_ / 2;
  const float fmax = std::min(7500.0f, fmax_frac_ * fs_);

  auto bin_of_hz = [&](float f) {
    float b = f * fft_n_ / fs_;
    if (b < 1.0f)
      b = 1.0f;
    if (b > (float) (n_bins - 1))
      b = (float) (n_bins - 1);
    return (int) b;
  };

  if (last_rms_db_ < noise_gate_db_) {
    // Silence gate: decay both bar + peak
    for (int i = 0; i < num_bars_; i++) {
      bar_[i] = std::max(0.0f, bar_[i] - idle_decay_);
      peak_[i] = std::max(0.0f, peak_[i] - idle_decay_);
    }
  } else {
    for (int i = 0; i < num_bars_; i++) {
      float t0 = (float) i / (float) num_bars_;
      float t1 = (float) (i + 1) / (float) num_bars_;
      float f0 = fmin_ * powf(fmax / fmin_, t0);
      float f1 = fmin_ * powf(fmax / fmin_, t1);

      int b0 = bin_of_hz(f0);
      int b1 = bin_of_hz(f1);
      if (b1 <= b0)
        b1 = b0 + 1;

      float acc = 0.0f;
      for (int b = b0; b < b1; b++) {
        float re = fft_in_[2 * b + 0];
        float im = fft_in_[2 * b + 1];
        acc += re * re + im * im;
      }
      acc /= (float) (b1 - b0);

      float db = 10.0f * log10f(acc + 1e-12f) + gain_db_;
      float norm = (db - noise_db_) / (0.0f - noise_db_);

      if (norm < 0.12f) {
        float t = norm / 0.12f;
        norm = 0.12f * (t * t);
      }
      if (i == 0)
        norm *= bar0_scale_;
      if (i == 1)
        norm *= bar1_scale_;
      norm = std::clamp(norm, 0.0f, 1.0f);

      // Smooth + peak
      bar_[i] = bar_[i] * (1.0f - smooth_) + norm * smooth_;
      if (peak_[i] < bar_[i])
        peak_[i] = bar_[i];
      else
        peak_[i] = std::max(0.0f, peak_[i] - peak_decay_);
    }
  }

  // Publish snapshot for renderer
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    bar_copy_ = bar_;
    peak_copy_ = peak_;
    data_ready_ = true;
    xSemaphoreGive(data_mutex_);
  }
}

void FxAudioSpectrum::step(float /*dt*/) {
  if (!canvas_ || area_.w <= 0 || area_.h <= 0)
    return;
  if (!dsc_init_)
    on_bind(canvas_);

  // Take snapshot (if available)
  std::vector<float> bar_local, peak_local;
  bool ready = false;
  if (data_mutex_ != nullptr && xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(3)) == pdTRUE) {
    if (data_ready_) {
      bar_local = bar_copy_;
      peak_local = peak_copy_;
      ready = true;
    }
    xSemaphoreGive(data_mutex_);
  }
  if (!ready || (int) bar_local.size() != num_bars_)
    return;

  // Clear the effect sub-rect
  lv_draw_rect_dsc_t bg;
  lv_draw_rect_dsc_init(&bg);
  bg.bg_color = lv_color_black();
  bg.bg_opa = LV_OPA_COVER;
  bg.border_opa = LV_OPA_TRANSP;
  lv_canvas_draw_rect(canvas_, area_.x, area_.y, area_.w, area_.h, &bg);

  // Draw bars + peaks
  const int H = area_.h;
  const int gap = gap_px_;
  int x = area_.x;

  // Ensure geometry is up to date with current width
  ensure_bar_geom_();

  for (int i = 0; i < num_bars_; i++) {
    int h = (int) lrintf(bar_local[i] * H);
    if (h < 1)
      h = 1;
    int y0 = area_.y + (H - h);

    // bar
    dsc_bar_.bg_color = bar_color_;
    lv_canvas_draw_rect(canvas_, x, y0, bar_w_px_, h, &dsc_bar_);

    // peak (1px high)
    int y_peak = area_.y + H - (int) lrintf(peak_local[i] * H) - 1;
    if (y_peak < area_.y)
      y_peak = area_.y;
    dsc_peak_.bg_color = peak_color_;
    lv_canvas_draw_rect(canvas_, x, y_peak, bar_w_px_, 1, &dsc_peak_);

    x += bar_w_px_ + gap;
  }
}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
