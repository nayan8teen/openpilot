"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Laneful visuals: the cyan lane-center target and the thin white "after" path (the
model path bent by laneful's nudge). No readout: the overlay is geometry only, so
the car's own path and laneful's correction are the whole story.
"""
import numpy as np
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.shader_polygon import draw_polygon

CENTER_COLOR = (0, 200, 255)
# Unmarked road: the target comes from a road edge and an assumed lane width, so
# it is drawn in a different shade than a measured lane center.
ROAD_HALF_COLOR = (255, 170, 0)
AFTER_COLOR = (255, 255, 255)
MIN_DISPLAY_LINE_PROB = 0.25
DEFAULT_LANE_WIDTH = 3.7  # m


class LanefulRenderer:
  def __init__(self):
    self._center_points = np.empty((0, 2), dtype=np.float32)
    self._after_points = np.empty((0, 2), dtype=np.float32)
    self._trust = 0.0
    self._lookahead = 0.0
    self._road_half = False
    # fade the whole overlay so enabling/disabling laneful does not pop
    self._alpha = FirstOrderFilter(0.0, 0.3, DT_MDL)

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
    self._road_half = bool(dbg.roadHalf)

    left = renderer._lane_lines[1].raw_points
    right = renderer._lane_lines[2].raw_points
    probs = renderer._lane_line_probs
    l_prob = float(probs[1]) if len(probs) > 1 else 0.0
    r_prob = float(probs[2]) if len(probs) > 2 else 0.0

    if l_prob < MIN_DISPLAY_LINE_PROB and r_prob < MIN_DISPLAY_LINE_PROB:
      # Unmarked road: modeld is centering in one half of the road, so draw that
      # half's center instead of a lane center that has no paint to sit on.
      self._center_points = self._road_half_points(renderer, max_idx)
      if self._center_points.size == 0:
        self._after_points = np.empty((0, 2), dtype=np.float32)
        return
    else:
      if len(left) == 0 or len(right) != len(left):
        self._center_points = np.empty((0, 2), dtype=np.float32)
        self._after_points = np.empty((0, 2), dtype=np.float32)
        return

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

  def _road_half_points(self, renderer, max_idx: int) -> np.ndarray:
    """Center of the half of an unmarked road the car is driving in.

    The car sits at y = 0, so whichever edge is nearer to it is the one it is
    driving along.  Half a lane width inward from that edge is the target the
    controller is steering to.
    """
    empty = np.empty((0, 2), dtype=np.float32)
    if not self._road_half:
      return empty
    left_edge = renderer._lane_lines[0].raw_points
    right_edge = renderer._lane_lines[3].raw_points
    if len(left_edge) == 0 or len(right_edge) != len(left_edge):
      return empty

    span = right_edge[:, 1] - left_edge[:, 1]
    if len(span) == 0 or float(np.median(span)) == 0.0:
      return empty
    # +1 when the right edge sits at the larger y in the model's frame.
    direction = 1.0 if float(np.median(span)) > 0.0 else -1.0
    near_right = abs(float(right_edge[0, 1])) <= abs(float(left_edge[0, 1]))
    edge = right_edge if near_right else left_edge
    # Inward from the chosen edge, toward the rest of the road.
    sign = direction if near_right else -direction

    center = edge.copy()
    center[:, 1] = edge[:, 1] - sign * DEFAULT_LANE_WIDTH / 2.0
    return renderer._map_line_to_polygon(center, 0.1, 0.0, max_idx)

  def _clear(self) -> None:
    self._center_points = np.empty((0, 2), dtype=np.float32)
    self._after_points = np.empty((0, 2), dtype=np.float32)
    self._trust = 0.0
    self._lookahead = 0.0
    self._road_half = False

  def _color(self, rgb: tuple[int, int, int], alpha: float) -> rl.Color:
    return rl.Color(*rgb, int(np.clip(alpha * self._alpha.x, 0.0, 255.0)))

  def draw(self, rect: rl.Rectangle) -> None:
    """Geometry only: the target line, then the model's path bent toward it."""
    if self._center_points.size and self._trust > 0.05:
      color = ROAD_HALF_COLOR if self._road_half else CENTER_COLOR
      draw_polygon(rect, self._center_points, self._color(color, np.clip(self._trust, 0.2, 1.0) * 255))
    if self._after_points.size:
      draw_polygon(rect, self._after_points, self._color(AFTER_COLOR, 200))
