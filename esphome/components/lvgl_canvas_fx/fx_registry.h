// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include <functional>
#include <map>
#include <memory>
#include <string>
#include "fx_base.h"

namespace esphome {
namespace lvgl_canvas_fx {

class FxRegistry {
 public:
  using Factory = std::function<std::unique_ptr<FxBase>()>;

  static void register_factory(const std::string &key, Factory f) {
    get_map_()[key] = std::move(f);
  }

  static std::unique_ptr<FxBase> make(const std::string &key) {
    auto &m = get_map_();
    auto it = m.find(key);
    if (it == m.end()) return nullptr;
    return (it->second)();
  }

 private:
  static std::map<std::string, Factory>& get_map_() {
    static std::map<std::string, Factory> m;
    return m;
  }
};

}  // namespace lvgl_canvas_fx
}  // namespace esphome
