# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

# esphome/components/lvgl_canvas_fx/__init__.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import lvgl
from esphome import automation
from esphome.core import CORE
from pathlib import Path

DEPENDENCIES = ["lvgl"]

ns = cg.esphome_ns.namespace("lvgl_canvas_fx")

LvglCanvasFx = ns.class_("LvglCanvasFx", cg.PollingComponent)
PauseAction   = ns.class_("PauseAction", automation.Action)
ResumeAction  = ns.class_("ResumeAction", automation.Action)
ToggleAction  = ns.class_("ToggleAction", automation.Action)
SetFpsAction  = ns.class_("SetFpsAction", automation.Action)

CONF_FX = "lvgl_canvas_fx"
CONF_EFFECT = "effect"
CONF_FPS = "fps"
CONF_CANVAS = "canvas"
CONF_X = "x"
CONF_Y = "y"
CONF_W = "width"
CONF_H = "height"
CONF_START_PAUSED = "start_paused"
CONF_PAUSED_PERIOD_MS = "paused_period_ms"

ITEM_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LvglCanvasFx),
    cv.Required(CONF_EFFECT): cv.string,
    cv.Required(CONF_CANVAS): cv.use_id(lvgl.Widget),
    cv.Optional(CONF_FPS, default=30.0): cv.float_range(min=1.0, max=240.0),
    cv.Optional(CONF_X, default=0): cv.int_range(min=0),
    cv.Optional(CONF_Y, default=0): cv.int_range(min=0),
    cv.Optional(CONF_W, default=0): cv.int_range(min=0),
    cv.Optional(CONF_H, default=0): cv.int_range(min=0),
    cv.Optional(CONF_START_PAUSED, default=False): cv.boolean,
    cv.Optional(CONF_PAUSED_PERIOD_MS): cv.positive_int,
})

CONFIG_SCHEMA = cv.All(cv.ensure_list(ITEM_SCHEMA))

@automation.register_action("lvgl_canvas_fx.pause", PauseAction, cv.Schema({ cv.Required(CONF_ID): cv.use_id(LvglCanvasFx) }))
async def pause_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id); tgt = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_target(tgt)); return var

@automation.register_action("lvgl_canvas_fx.resume", ResumeAction, cv.Schema({ cv.Required(CONF_ID): cv.use_id(LvglCanvasFx) }))
async def resume_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id); tgt = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_target(tgt)); return var

@automation.register_action("lvgl_canvas_fx.toggle", ToggleAction, cv.Schema({ cv.Required(CONF_ID): cv.use_id(LvglCanvasFx) }))
async def toggle_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id); tgt = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_target(tgt)); return var

@automation.register_action("lvgl_canvas_fx.set_fps", SetFpsAction, cv.Schema({
    cv.Required(CONF_ID): cv.use_id(LvglCanvasFx),
    cv.Required(CONF_FPS): cv.float_range(min=1.0, max=240.0),
}))
async def setfps_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id); tgt = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_target(tgt)); cg.add(var.set_fps(config[CONF_FPS])); return var

async def to_code(config):
    # Generate the component instances
    for item in config:
        var = cg.new_Pvariable(item[CONF_ID])
        await cg.register_component(var, {})

        period_ms = int(round(1000.0 / item[CONF_FPS]))
        cg.add(var.set_initial_period(period_ms))
        if CONF_PAUSED_PERIOD_MS in item:
            cg.add(var.set_paused_period_ms(item[CONF_PAUSED_PERIOD_MS]))

        canvas_widget = await cg.get_variable(item[CONF_CANVAS])

        cg.add(var.setup_binding(
            canvas_widget,
            item[CONF_EFFECT],
            item[CONF_X], item[CONF_Y],
            item[CONF_W], item[CONF_H],
            item[CONF_START_PAUSED]
        ))