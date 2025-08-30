// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "register_builtin_effects.h"
#include "fx_registry.h"

// include each effect's header
#include "fx_circle.h"
// #include "plasma/fx_plasma.h"
// #include "noise/fx_noise.h"

namespace esphome {
namespace lvgl_canvas_fx {

void register_builtin_effects() {
  // One line per effect:
  FxRegistry::register_factory("circle", []{
    return std::make_unique<FxCircle>();
  });

  // Add more:
  // FxRegistry::register_factory("plasma", []{ return std::make_unique<FxPlasma>(); });
  // FxRegistry::register_factory("noise",  []{ return std::make_unique<FxNoise>();  });
}

}  // namespace lvgl_canvas_fx
}  // namespace esphome
