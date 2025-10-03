# lvgl-canvas-fx

An ESPHome external component that renders fun, high‑performance visual effects on an **LVGL Canvas**. It’s designed for ESP32‑class devices (ESPHome 2025.x, LVGL v8) and works with any LVGL‑backed display (HUB75 matrices, TFTs, etc.).

> **Status:** early but functional. Effects today: `circle`, `fireworks`, `fireplace`, `aurora`, `audio_spectrum`. Contributions welcome!

---

## Features
- **Drop‑in ESPHome component**: simple YAML config; no app code required.
- **Multiple effects** with easy runtime switching via `lvgl_canvas_fx.set_effect`.
- **Tunable FPS**: With an ESP32-S3, currently on 64×64 you can reach ~60 FPS, on 128×64 closer to 30 FPS, and on 800×480 around 7 FPS. Depends a good bit on your display.
- **Pausing/resuming** to save cycles when a page is hidden.
- **Physics‑based fireworks** powered by Chipmunk2D.

## Requirements
- **ESPHome:** 2025.8.x or newer
- **LVGL:** v8 (ESPHome's built‑in LVGL integration)

## Quick start (full example)
The example below wires up a Canvas, the effect engine, and a template `select` so you can switch effects at runtime. It also pauses the effect when the page is not visible.

```yaml
substitutions:
  DISPLAY_W: "128"
  DISPLAY_H: "64"

external_components:
  - source: github://stuartparmenter/lvgl-canvas-fx@main
    components: [lvgl_canvas_fx]

select:
  - platform: template
    id: fx_effect_select
    name: "FX Effect"
    optimistic: true
    options:
      - circle
      - fireworks
      - fireplace
    initial_option: fireworks
    set_action:
      - lvgl_canvas_fx.set_effect:
          id: cfx
          effect: !lambda 'return x;'

lvgl_canvas_fx:
  - id: cfx
    effect: fireworks  # or circle or fireplace
    canvas: fx_canvas
    fps: 24
    start_paused: true

lvgl:
  pages:
    - id: fx_page
      widgets:
        - canvas:
            id: fx_canvas
            width: ${DISPLAY_W}
            height: ${DISPLAY_H}
      on_load:
        then:
          # Keep the selected option authoritative: sync effect to current select value
          - lvgl_canvas_fx.set_effect:
              id: cfx
              effect: !lambda 'return id(fx_effect_select).state;'
          - lvgl_canvas_fx.resume:
              id: cfx
      on_unload:
        then:
          - lvgl_canvas_fx.pause:
              id: cfx
```

### Minimal example (no select)
```yaml
lvgl_canvas_fx:
  - id: cfx
    effect: fireplace
    canvas: fx_canvas
    fps: 24

lvgl:
  pages:
    - id: fx_page
      widgets:
        - canvas:
            id: fx_canvas
            width: ${DISPLAY_W}
            height: ${DISPLAY_H}
```

## Options
Under the `lvgl_canvas_fx:` array you can configure:

| Option          | Type     | Default  | Notes |
|-----------------|----------|----------|------|
| `id`            | ID       | —        | Required. Instance id.
| `canvas`        | string   | —        | Required. The LVGL Canvas widget id to render into.
| `effect`        | string   | `circle` | One of: `circle`, `fireworks`, `fireplace`.
| `fps`           | int      | `30`     | Target frame rate. Lower to save CPU.
| `start_paused`  | bool     | `false`  | Start paused and `resume` when visible.

### Services (actions)
- `lvgl_canvas_fx.set_effect: { id, effect }`
- `lvgl_canvas_fx.pause: { id }`
- `lvgl_canvas_fx.resume: { id }`

## Tips & troubleshooting
- **Page lifecycle:** Use `start_paused: true` and call `resume` on `on_load` and `pause` on `on_unload` to avoid burning cycles off‑screen.
- **Canvas buffer:** The ESPHome LVGL Canvas widget creates its buffer—no need to create/free one in the component.
- **Performance:** Rendering cost depends heavily on canvas resolution. At small sizes (64×64) 60 FPS is achievable; at 128×64 you may see ~30 FPS; at 800×480 expect ~7 FPS.
- **Memory usage:** Some effects are memory‑heavy. Small resolutions may run without PSRAM, but higher resolutions require PSRAM enabled.

## Roadmap
- More effects (PRs welcome!)
- Color themes & parameters per effect
- Metrics counters & debug overlay

## Development
Directory layout mirrors ESPHome external component conventions:
```
/esphome/components/lvgl_canvas_fx/
  lvgl_canvas_fx.py         # component schema & codegen
  lvgl_canvas_fx.cpp/.h     # runtime
  fx_*.cpp/.h               # individual effects
  register_builtin_effects.cpp
```

### Building locally
Point ESPHome at your local checkout:
```yaml
external_components:
  - source:
      type: local
      path: "/path/to/lvgl-canvas-fx/esphome/components"
    components: [lvgl_canvas_fx]
```

## License
MIT © Stuart Parmenter
