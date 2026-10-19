"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Laneful visuals: cyan lane-center target, thin white "after" path (model path bent by laneful's nudge),
and a top-center readout of the pull. The debug numbers come from modeld's
LanefulOutput, so the overlay and the controller cannot disagree.
"""
import numpy as np
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.system.ui.lib.text_measure import measure_text_cached

CENTER_COLOR = (0, 200, 255)
AFTER_COLOR = (255, 255, 255)
MIN_DISPLAY_LINE_PROB = 0.25
MAX_DISPLAY_ERROR = 2.0  # m, full-scale readout
DEFAULT_LANE_WIDTH = 3.7  # m


class LanefulRenderer:
  def __init__(self):
    self._center_points = np.empty((0, 2), dtype=np.float32)
    self._after_points = np.empty((0, 2), dtype=np.float32)
    self._trust = 0.0
    self._lookahead = 0.0
    self._font = gui_app.font(FontWeight.BOLD)
    # smoothed copies for the readout so the numbers don't flicker
    self._pull_display = FirstOrderFilter(0.0, 0.5, DT_MDL)
    self._trust_display = FirstOrderFilter(0.0, 0.5, DT_MDL)
    # fade the whole overlay so enabling/disabling laneful does not pop
    self._alpha = FirstOrderFilter(0.0, 0.3, DT_MDL)
    self._readout_rect: rl.Rectangle | None = None
    self._settings_page = None

  def update(self, renderer, max_idx: int) -> None:
    """Called from the model renderer's update with lane lines and path already in raw points."""
    sm = ui_state.sm
    if not sm.seen['modelDataV2SP']:
      self._alpha.update(0.0)
      self._clear()
      return

    dbg = sm['modelDataV2SP'].laneful
    self._alpha.update(1.0 if dbg.active else 0.0)
    if self._alpha.x < 0.01:
      self._clear()
      return

    self._trust = float(dbg.trust)
    self._lookahead = max(float(dbg.lookahead), 1.0)
    self._pull_display.update(float(dbg.lateralError))
    self._trust_display.update(self._trust)

    left = renderer._lane_lines[1].raw_points
    right = renderer._lane_lines[2].raw_points
    if len(left) == 0 or len(right) != len(left):
      self._center_points = np.empty((0, 2), dtype=np.float32)
      self._after_points = np.empty((0, 2), dtype=np.float32)
      return

    probs = renderer._lane_line_probs
    l_prob = float(probs[1]) if len(probs) > 1 else 0.0
    r_prob = float(probs[2]) if len(probs) > 2 else 0.0
    # Lane-line y follows the model's lateral frame, which may be positive to
    # the left or to the right.  Take the direction from the lines themselves.
    span = right[:, 1] - left[:, 1]
    direction = -1.0 if len(span) and float(np.median(span)) < 0.0 else 1.0
    if l_prob >= MIN_DISPLAY_LINE_PROB and r_prob >= MIN_DISPLAY_LINE_PROB and len(span):
      width = float(np.clip(abs(np.median(span)), 2.8, 4.5))
    else:
      width = DEFAULT_LANE_WIDTH

    center = left.copy()
    if l_prob >= MIN_DISPLAY_LINE_PROB and r_prob >= MIN_DISPLAY_LINE_PROB:
      center[:, 1] = 0.5 * (left[:, 1] + right[:, 1])
    elif l_prob >= MIN_DISPLAY_LINE_PROB:
      center[:, 1] = left[:, 1] + direction * width / 2.0
    elif r_prob >= MIN_DISPLAY_LINE_PROB:
      center[:, 1] = right[:, 1] - direction * width / 2.0
    else:
      self._center_points = np.empty((0, 2), dtype=np.float32)
      self._after_points = np.empty((0, 2), dtype=np.float32)
      return
    self._center_points = renderer._map_line_to_polygon(center, 0.1, 0.0, max_idx)

    # The after path is the predictor's preview error, not a second lane-center
    # estimate.  Bend the model path toward the lane center with the same sign
    # and magnitude that modeld is applying.
    path = renderer._path.raw_points
    if len(path) == 0:
      self._after_points = np.empty((0, 2), dtype=np.float32)
      return
    error = float(dbg.lateralError)
    after = path.copy()
    after[:, 1] += error * np.clip(after[:, 0] / self._lookahead, 0.0, 1.0) ** 2
    after_idx = renderer._get_path_length_idx(after[:, 0], min(self._lookahead, after[-1, 0]))
    self._after_points = renderer._map_line_to_polygon(after, 0.15, renderer._path_offset_z, after_idx)

  def _clear(self) -> None:
    self._center_points = np.empty((0, 2), dtype=np.float32)
    self._after_points = np.empty((0, 2), dtype=np.float32)
    self._trust = 0.0
    self._lookahead = 0.0
    self._pull_display.update(0.0)
    self._trust_display.update(0.0)

  def _color(self, rgb: tuple[int, int, int], alpha: float) -> rl.Color:
    return rl.Color(*rgb, int(np.clip(alpha * self._alpha.x, 0.0, 255.0)))

  def draw(self, rect: rl.Rectangle) -> None:
    if self._center_points.size and self._trust > 0.05:
      draw_polygon(rect, self._center_points, self._color(CENTER_COLOR, np.clip(self._trust, 0.2, 1.0) * 255))
    self._readout_rect = None
    if self._after_points.size:
      draw_polygon(rect, self._after_points, self._color(AFTER_COLOR, 200))
      self._draw_readout(rect)

  def handle_tap(self, pos) -> bool:
    """Tapping the readout opens the laneful settings page. Returns True if the tap was consumed."""
    r = self._readout_rect
    if r is None or not (r.x - 20 <= pos.x <= r.x + r.width + 20 and r.y - 20 <= pos.y <= r.y + r.height + 20):
      return False
    if self._settings_page is None:
      from openpilot.selfdrive.ui.sunnypilot.mici.layouts.laneful import LanefulLayoutMici
      self._settings_page = LanefulLayoutMici()
    gui_app.push_widget(self._settings_page)
    return True

  def _draw_readout(self, rect: rl.Rectangle) -> None:
    """Top-center pill: pull toward lane center in cm (capped at 50), arrow = side,
    bar grows from center toward the pull (full at the cap), brighter with more lane line trust."""
    pull, trust = self._pull_display.x, self._trust_display.x
    w, h = 190, 76
    x, y = rect.x + (rect.width - w) / 2, rect.y + 10
    cx, cy = x + w / 2, y + 30
    self._readout_rect = rl.Rectangle(x, y, w, h)
    rl.draw_rectangle_rounded(self._readout_rect, 0.5, 12, self._color((0, 0, 0), 170))

    text = f"{abs(pull) * 100:.0f}"
    text_w = measure_text_cached(self._font, text, 44, 0).x
    rl.draw_text_ex(self._font, text, rl.Vector2(cx - text_w / 2, cy - 24), 44, 0, self._color((255, 255, 255), 255))

    active = self._color(CENTER_COLOR, np.clip(abs(pull) / 0.1, 0.0, 1.0) * 255)
    idle = self._color((255, 255, 255), 40)
    lx, rx = x + 18, x + w - 18
    rl.draw_triangle(rl.Vector2(lx, cy), rl.Vector2(lx + 16, cy + 12), rl.Vector2(lx + 16, cy - 12), active if pull < 0 else idle)
    rl.draw_triangle(rl.Vector2(rx - 16, cy - 12), rl.Vector2(rx - 16, cy + 12), rl.Vector2(rx, cy), active if pull > 0 else idle)

    bar = rl.Rectangle(x + 24, y + h - 16, w - 48, 6)
    rl.draw_rectangle_rounded(bar, 1.0, 6, self._color((255, 255, 255), 40))
    rl.draw_rectangle(int(cx - 1), int(bar.y - 2), 2, int(bar.height + 4), self._color((255, 255, 255), 120))
    fill = bar.width / 2 * float(np.clip(abs(pull) / MAX_DISPLAY_ERROR, 0.0, 1.0))
    if fill > 1:
      fill_x = cx if pull > 0 else cx - fill
      rl.draw_rectangle_rounded(rl.Rectangle(fill_x, bar.y, fill, bar.height), 1.0, 6,
                                self._color(CENTER_COLOR, np.clip(0.3 + 0.7 * trust, 0.0, 1.0) * 255))
