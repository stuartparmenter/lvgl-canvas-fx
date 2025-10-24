// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "register_builtin_effects.h"
#include "fx_registry.h"

// include each effect's header
#include "fx_circle.h"
#include "fx_fireplace.h"
#include "fx_fireworks_physics.h"
#include "fx_aurora.h"
#include "fx_audio_spectrum.h"

namespace esphome {
namespace lvgl_canvas_fx {

void register_builtin_effects() {
  // One line per effect:
  FxRegistry::register_factory("circle", [] { return std::make_unique<FxCircle>(); });

  FxRegistry::register_factory("fireplace", [] { return std::make_unique<FxFireplace>(); });

  FxRegistry::register_factory("fireworks", [] { return std::make_unique<FxFireworksPhysics>(); });

  FxRegistry::register_factory("aurora", [] {
    auto fx = std::make_unique<FxAurora>();
    // Optional: tune defaults here if you want different look/speed
    // fx->set_speed(0.05f);
    // fx->set_scale(24);
    // fx->set_intensity(192);
    return fx;
  });

  FxRegistry::register_factory("audio_spectrum", [] {
    auto fx = std::make_unique<FxAudioSpectrum>();

    // Music friendly settings
    // 512, 8, 16000.0, 0.022f, 0.25f, 9.0f, -55.0f, 100.0f, 0.47f
    fx->configure(512, 8, 16000.0f, 0.022f, 0.25f, 9.0f, -55.0f, 100.0f, 0.47f);

    fx->set_colors(lv_color_hex(0x00FFD0), lv_color_hex(0xFF4000));
    fx->set_gap_px(1);
    fx->set_round_to_mult8(true);

    return fx;
  });
}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
