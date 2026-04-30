"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import numpy as np
import pyray as rl
from collections import OrderedDict
from functools import wraps
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar, TORQUE_ANGLE_SPAN

# Wheel radius and center offset (from bottom)
WHEEL_RADIUS = 40
WHEEL_Y_OFFSET = 19


def get_mask_angle_vec(r, d, rw, left=True):
  # Vectorized mask angle calculation
  cos_theta = (r * r + d * d - rw * rw) / (2 * r * d)
  theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
  return -90.0 - np.degrees(theta) if left else -90.0 + np.degrees(theta)


def quantized_lru_cache_sp(maxsize=256):
  def decorator(func):
    cache = OrderedDict()

    @wraps(func)
    def wrapper(cx, cy, r_mid, thickness, a0, a1, **kwargs):
      # Quantize inputs for caching: balanced for smoothness vs cache effectiveness
      def q_val(v):
        if isinstance(v, (int, float)):
          return round(v * 20) / 20  # 0.05° precision
        return v  # strings or callables

      q_kwargs = tuple(sorted((k, q_val(v)) for k, v in kwargs.items()))
      key = (round(cx), round(cy), round(r_mid), round(thickness),
             q_val(a0), q_val(a1), q_kwargs)

      if key in cache:
        cache.move_to_end(key)
        return cache[key]

      result = func(cx, cy, r_mid, thickness, a0, a1, **kwargs)
      if len(cache) >= maxsize:
        cache.popitem(last=False)
      cache[key] = result
      return result
    return wrapper
  return decorator


@quantized_lru_cache_sp(maxsize=256)
def arc_bar_pts_sp(cx: float, cy: float,
                   r_mid: float, thickness: float,
                   a0: any, a1: any,
                   *, max_points: int = 60, cap_segs: int = 6,
                   cap_radius_a0: float = 7, cap_radius_a1: float = 7,
                   px_per_seg: float = 4.0,
                   d_val: float = 0, rw_val: float = 0) -> np.ndarray:
  """Return Nx2 np.float32 points for a single closed polygon (thick arc with custom caps)."""

  half = thickness * 0.5
  r_outer = r_mid + half
  r_inner = r_mid - half

  def get_angle_vec(val, r_arr):
    if val == "mask_left":
      return get_mask_angle_vec(r_arr, d_val, rw_val, left=True)
    if val == "mask_right":
      return get_mask_angle_vec(r_arr, d_val, rw_val, left=False)
    if callable(val):
      return np.array([val(r) for r in r_arr], dtype=np.float32)
    return np.full_like(r_arr, val, dtype=np.float32)

  def get_cap(is_start: bool, cap_radius: float, angle_val: any):
    if callable(angle_val) or isinstance(angle_val, str):
      # Generate points along the radius to form the circular mask
      rs = np.linspace(r_outer, r_inner, cap_segs + 1, dtype=np.float32) if not is_start else np.linspace(r_inner, r_outer, cap_segs + 1, dtype=np.float32)
      angs = get_angle_vec(angle_val, rs)
      rads = np.radians(angs)
      return np.c_[cx + rs * np.cos(rads), cy + rs * np.sin(rads)]

    # Standard rounded cap - vectorized
    rad = math.radians(angle_val)
    nx, ny = math.cos(rad), math.sin(rad)
    tx, ty = -ny, nx
    mx, my = cx + nx * r_mid, cy + ny * r_mid

    if cap_radius <= 0:
      p_outer = np.array([mx + nx * half, my + ny * half], dtype=np.float32)
      p_inner = np.array([mx - nx * half, my - ny * half], dtype=np.float32)
      return np.vstack((p_outer, p_inner)) if not is_start else np.vstack((p_inner, p_outer))

    cap_radius = min(cap_radius, half)
    ex, ey = mx + nx * (half - cap_radius), my + ny * (half - cap_radius)
    ex2, ey2 = mx + nx * (-half + cap_radius), my + ny * (-half + cap_radius)

    if not is_start:
      alpha = np.deg2rad(np.linspace(90, 0, cap_segs + 2, dtype=np.float32))[1:-1]
      alpha2 = np.deg2rad(np.linspace(0, -90, cap_segs + 1, dtype=np.float32))[:-1]
    else:
      alpha = np.deg2rad(np.linspace(180, 90, cap_segs + 2, dtype=np.float32))[1:-1]
      alpha2 = np.deg2rad(np.linspace(-90, -180, cap_segs + 1, dtype=np.float32))[:-1]

    c_cos, c_sin = np.cos(alpha), np.sin(alpha)
    cap_end = np.c_[ex + c_cos * cap_radius * tx + c_sin * cap_radius * nx,
                    ey + c_cos * cap_radius * ty + c_sin * cap_radius * ny]

    c_cos2, c_sin2 = np.cos(alpha2), np.sin(alpha2)
    cap_end_bot = np.c_[ex2 + c_cos2 * cap_radius * tx + c_sin2 * cap_radius * nx,
                        ey2 + c_cos2 * cap_radius * ty + c_sin2 * cap_radius * ny]

    return np.vstack((cap_end, cap_end_bot)) if not is_start else np.vstack((cap_end_bot, cap_end))

  # Use centerline angle for span calculation
  a0_mid = get_angle_vec(a0, np.array([r_mid], dtype=np.float32))[0]
  a1_mid = get_angle_vec(a1, np.array([r_mid], dtype=np.float32))[0]
  if a1_mid < a0_mid:
    a0, a1 = a1, a0
    a0_mid, a1_mid = a1_mid, a0_mid
    cap_radius_a0, cap_radius_a1 = cap_radius_a1, cap_radius_a0

  span = max(1e-3, a1_mid - a0_mid)
  arc_len = r_mid * math.radians(span)
  # Limit point count to optimize triangulate() and GPU upload
  arc_segs = max(4, min(int(arc_len / px_per_seg), (max_points - 15) // 2))

  # arcs
  a0_vec = get_angle_vec(a0, np.array([r_outer, r_inner], dtype=np.float32))
  a1_vec = get_angle_vec(a1, np.array([r_outer, r_inner], dtype=np.float32))

  ang_o = np.deg2rad(np.linspace(a0_vec[0], a1_vec[0], arc_segs + 1, dtype=np.float32))
  outer = np.c_[cx + np.cos(ang_o) * r_outer,
                cy + np.sin(ang_o) * r_outer]

  ang_i = np.deg2rad(np.linspace(a1_vec[1], a0_vec[1], arc_segs + 1, dtype=np.float32))
  inner = np.c_[cx + np.cos(ang_i) * r_inner,
                cy + np.sin(ang_i) * r_inner]

  return np.vstack((outer, get_cap(False, cap_radius_a1, a1),
                    inner, get_cap(True, cap_radius_a0, a0),
                    outer[:1])).astype(np.float32)


class TorqueBarSP(TorqueBar):
  def _render(self, rect: rl.Rectangle) -> None:
    # torque line height and alpha animation
    torque_line_height = np.interp(abs(self._torque_filter.x), [0.5, 1], [14 * self._scale, 56 * self._scale])
    if not self._demo:
      self._torque_line_alpha_filter.update(ui_state.status not in (UIStatus.DISENGAGED, UIStatus.LONG_ONLY))
    else:
      self._torque_line_alpha_filter.update(1.0)

    torque_line_bg_alpha = np.interp(abs(self._torque_filter.x), [0.5, 1.0], [0.25, 0.5])
    torque_line_bg_color = rl.Color(255, 255, 255, int(255 * torque_line_bg_alpha * self._torque_line_alpha_filter.x))
    if ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY) and not self._demo:
      torque_line_bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * self._torque_line_alpha_filter.x))

    # Fixed curvature centered on the wheel
    mid_r = 1200 * self._scale
    # To grow upwards while keeping the bottom static:
    # 1. Start with the desired bottom Y position (WHEEL_Y_OFFSET - 14)
    # 2. As height increases, the mid-radius (cy) must move DOWN to push the top UP.
    bottom_y_offset = (WHEEL_Y_OFFSET - 14) * self._scale
    cy = rect.y + rect.height + mid_r - bottom_y_offset - torque_line_height
    cx = rect.x + rect.width / 2 + 8

    # Distance from arc center to wheel center
    d_val = cy - (rect.y + rect.height - WHEEL_Y_OFFSET * self._scale)
    # Shrink mask radius by 1px to ensure overlap and prevent jarred edges
    rw_val = (WHEEL_RADIUS - 1) * self._scale

    # Mask angle functions for circular mask matching wheel radius
    m_left_f = "mask_left"
    m_right_f = "mask_right"

    m_left_mid = get_mask_angle_vec(mid_r, d_val, rw_val, left=True)
    m_right_mid = get_mask_angle_vec(mid_r, d_val, rw_val, left=False)

    top_angle = -90
    torque_bg_angle_span = self._torque_line_alpha_filter.x * TORQUE_ANGLE_SPAN
    torque_start_angle = top_angle - torque_bg_angle_span / 2
    torque_end_angle = top_angle + torque_bg_angle_span / 2

    # draw bg torque indicator line - masked behind the wheel
    bg_pts_left = None
    if torque_start_angle < m_left_mid:
      bg_pts_left = arc_bar_pts_sp(cx, cy, mid_r, torque_line_height, torque_start_angle, m_left_f,
                                   cap_radius_a0=7 * self._scale, d_val=d_val, rw_val=rw_val)
      draw_polygon(rect, bg_pts_left, color=torque_line_bg_color)

    bg_pts_right = None
    if torque_end_angle > m_right_mid:
      bg_pts_right = arc_bar_pts_sp(cx, cy, mid_r, torque_line_height, m_right_f, torque_end_angle,
                                    cap_radius_a1=7 * self._scale, d_val=d_val, rw_val=rw_val)
      draw_polygon(rect, bg_pts_right, color=torque_line_bg_color)

    # draw active torque indicator line
    if abs(self._torque_filter.x) > 1e-3:
      is_left = self._torque_filter.x < 0
      if is_left:
        a0_active = m_left_f
        a1_active = m_left_mid + (torque_bg_angle_span / 2 - (m_right_mid - top_angle)) * self._torque_filter.x
      else:
        a0_active = m_right_f
        a1_active = m_right_mid + (torque_bg_angle_span / 2 - (m_right_mid - top_angle)) * self._torque_filter.x

      sl_pts = arc_bar_pts_sp(cx, cy, mid_r, torque_line_height, a0_active, a1_active,
                              cap_radius_a0=7 * self._scale, cap_radius_a1=7 * self._scale, d_val=d_val, rw_val=rw_val)

      # draw beautiful gradient
      start_grad_pt = cx / rect.width
      # find end_grad_pt based on visible bg bars
      if is_left and bg_pts_left is not None:
        end_grad_pt = (cx * (1 - 0.65) + (min(bg_pts_left[:, 0]) * 0.65)) / rect.width
      elif not is_left and bg_pts_right is not None:
        end_grad_pt = (cx * (1 - 0.65) + (max(bg_pts_right[:, 0]) * 0.65)) / rect.width
      else:
        end_grad_pt = start_grad_pt

      # fade to orange as we approach max torque
      start_color = blend_colors(
        rl.Color(255, 255, 255, int(255 * 0.9 * self._torque_line_alpha_filter.x)),
        rl.Color(255, 200, 0, int(255 * self._torque_line_alpha_filter.x)),  # yellow
        max(0, abs(self._torque_filter.x) - 0.75) * 4,
      )
      end_color = blend_colors(
        rl.Color(255, 255, 255, int(255 * 0.9 * self._torque_line_alpha_filter.x)),
        rl.Color(255, 115, 0, int(255 * self._torque_line_alpha_filter.x)),  # orange
        max(0, abs(self._torque_filter.x) - 0.75) * 4,
      )

      if ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY) and not self._demo:
        start_color = end_color = rl.Color(255, 255, 255, int(255 * 0.35 * self._torque_line_alpha_filter.x))

      gradient = Gradient(
        start=(start_grad_pt, 0),
        end=(end_grad_pt, 0),
        colors=[start_color, end_color],
        stops=[0.0, 1.0],
      )

      draw_polygon(rect, sl_pts, gradient=gradient)
