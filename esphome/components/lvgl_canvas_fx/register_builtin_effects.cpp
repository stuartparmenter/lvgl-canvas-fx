// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "register_builtin_effects.h"
#include "fx_registry.h"

// include each effect's header
#include "fx_circle.h"
#include "fx_fireplace.h"
#include "fx_fireworks_physics.h"
#include "fx_aurora.h"

namespace esphome {
namespace lvgl_canvas_fx {

void register_builtin_effects() {
  // One line per effect:
  FxRegistry::register_factory("circle", []{
    return std::make_unique<FxCircle>();
  });

  FxRegistry::register_factory("fireplace", []{
    return std::make_unique<FxFireplace>();
  });

  FxRegistry::register_factory("fireworks", []{
    return std::make_unique<FxFireworksPhysics>();
  });

  FxRegistry::register_factory("aurora", []{
    auto fx = std::make_unique<FxAurora>();
    // Optional: tune defaults here if you want different look/speed
    // fx->set_speed(0.05f);
    // fx->set_scale(24);
    // fx->set_intensity(192);
    return fx;
  });

}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
