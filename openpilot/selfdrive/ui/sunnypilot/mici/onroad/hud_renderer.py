"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.torque_bar import TorqueBarSP
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.system.ui.lib.application import gui_app

EXCLAMATION_POINT_SPACING = 10
WHEEL_BOTTOM_OFFSET = 35

class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self._torque_bar_op = self._torque_bar
    self._torque_bar_sp = TorqueBarSP()
    self.blind_spot_indicators = BlindSpotIndicators()

    self._txt_wheel_op = self._txt_wheel
    self._txt_wheel_critical_op = self._txt_wheel_critical
    self._txt_wheel_sp: rl.Texture = gui_app.texture('icons_mici/wheel.png', 80, 80)
    self._txt_wheel_critical_sp: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 80, 80)

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

  def _render(self, rect: rl.Rectangle) -> None:
    self._torque_bar = self._torque_bar_sp if ui_state.alt_torque_bar else self._torque_bar_op
    super()._render(rect)
    self.blind_spot_indicators.render(rect)

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    if not ui_state.alt_torque_bar:
      self._txt_wheel = self._txt_wheel_op
      self._txt_wheel_critical = self._txt_wheel_critical
      super()._draw_steering_wheel(rect)
      return

    self._txt_wheel = self._txt_wheel_sp
    self._txt_wheel_critical = self._txt_wheel_critical
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    # pos - center
    pos_x = int(rect.x + rect.width / 2 + 8)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + WHEEL_BOTTOM_OFFSET + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    # color and draw
    color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      # Draw exclamation point icon
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 1.5
      rl.draw_texture_ex(self._txt_exclamation_point, rl.Vector2(exclamation_pos_x, exclamation_pos_y), 0.0, 1.0, rl.WHITE)

  def _has_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.detected
