// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "register_builtin_effects.h"
#include "fx_registry.h"

// include each effect's header
#include "fx_circle.h"
#include "fx_fireplace.h"
#include "fx_fireworks_physics.h"

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

}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
