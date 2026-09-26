"""Lane centering policy shared by stock modeld and sunnypilot modeld_v2.

Adapted from gm1500/openpilot 5cbde503 ("Add full lane centering policy with HUD
selector"), ported to a class so each model process owns its own state, with the
toggle provided through sunnylink instead of an on-road HUD selector.

The lane policy treats E2E as the road-shape feed-forward command and, once a
clean two-line lane has armed, adds a bounded curvature correction that anchors
the path to the lane midpoint. It is deliberately not an E2E/lane-curvature
blend: only the lane-center offset and heading at a longer spatial anchor feed
the correction, which avoids the lagging filtered lane-curvature target that
overshoots on a straight road.
"""
import time

import numpy as np

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants

LANE_POLICY_ENABLED_PARAM = "LanePolicyEnabled"

# Two-line confidence uses entry/exit hysteresis. It prevents a clean lane from
# dropping to E2E merely because one model frame is slightly less certain.
LANE_LOCK_ARM_LINE_PROB = 0.92
LANE_LOCK_RETAIN_LINE_PROB = 0.70
LANE_LOCK_MIN_LANE_WIDTH = 2.6                 # m
LANE_LOCK_MAX_LANE_WIDTH = 5.2                 # m
LANE_LOCK_MIN_WIDTH_EDGE = 0.10                # m inside accepted range
LANE_LOCK_MAX_WIDTH_CHANGE = 0.75              # m across the fitted horizon
LANE_LOCK_ARM_TIME = 0.75                      # s of clean two-line confidence
LANE_LOCK_WIDTH_TIME = 9.95                    # s, matches the old lane planner
LANE_LOCK_ONE_LINE_HOLD_TIME = 1.00            # s using the learned lane width
LANE_LOCK_FIT_START = 8.0                      # m
LANE_LOCK_FIT_END = 55.0                       # m
LANE_LOCK_MIN_LOOKAHEAD = 25.0                 # m
LANE_LOCK_MAX_LOOKAHEAD = 45.0                 # m
LANE_LOCK_HEADING_GAIN = 0.55
LANE_LOCK_MAX_CENTER_CORRECTION = 0.00045      # 1/m
LANE_LOCK_TURN_CURVATURE = 0.00015             # 1/m
LANE_LOCK_TURN_RELEASE_TIME = 0.35             # s
# Build lane authority deliberately, but release it faster when the lane
# midpoint says the previous correction is no longer needed. This prevents a
# curve-exit or lane-change correction from lingering past the centerline.
LANE_LOCK_CORRECTION_ENGAGE_STEP = 0.00008     # 1/m per model frame
LANE_LOCK_CORRECTION_RELEASE_STEP = 0.00020    # 1/m per model frame
LANE_LOCK_CORRECTION_DEADBAND = 0.000012       # 1/m
LANE_LOCK_MAX_LANE_CHANGE_PROB = 0.10
# Below this speed the policy hands back exact E2E: parking aisles and creep
# traffic are not lanes worth centering in, and a correction built while
# stationary must not tug the wheel on launch. Arming progress and the learned
# width are kept so centering resumes smoothly above the threshold.
LANE_LOCK_MIN_SPEED = 3.0                      # m/s
LANE_LOCK_LOG_INTERVAL = 1.0                   # seconds


def get_lane_policy_enabled(params: Params) -> bool:
  """Read the lane-centering toggle, correctly decoding Params' b\"0\"/b\"1\" value.

  Defaults to enabled when unset so the policy matches the param default and
  older params databases that predate the key.
  """
  value = params.get(LANE_POLICY_ENABLED_PARAM)
  if value is None:
    return True
  if isinstance(value, (bytes, bytearray)):
    return value == b"1"
  return bool(value)


def get_inner_lane_line_probs(model_output: dict[str, np.ndarray]) -> tuple[float, float]:
  """Return inner left/right raw-model probabilities from the 8-wide layout."""
  lane_line_probs = np.asarray(model_output['lane_lines_prob'])
  if lane_line_probs.shape != (1, 8):
    raise ValueError(f"expected lane_lines_prob shape (1, 8), got {lane_line_probs.shape}")
  return float(lane_line_probs[0, 3]), float(lane_line_probs[0, 5])


def get_lane_width_measurement(left_y: np.ndarray, right_y: np.ndarray,
                               fit: np.ndarray) -> tuple[float, bool]:
  """Return a robust width measurement and whether the pair is geometrically usable."""
  lane_width = right_y - left_y
  width_p10, width_median, width_p90 = np.percentile(lane_width[fit], (10.0, 50.0, 90.0))
  width_change = float(width_p90 - width_p10)
  width_edge_distance = min(width_p10 - LANE_LOCK_MIN_LANE_WIDTH,
                            LANE_LOCK_MAX_LANE_WIDTH - width_p90)
  valid = (LANE_LOCK_MIN_LANE_WIDTH <= width_median <= LANE_LOCK_MAX_LANE_WIDTH and
           width_edge_distance >= LANE_LOCK_MIN_WIDTH_EDGE and
           width_change <= LANE_LOCK_MAX_WIDTH_CHANGE)
  return float(width_median), bool(valid)


class LanePolicy:
  """Progressively anchor E2E to a stable lane midpoint without output blending."""

  def __init__(self, x_idxs: np.ndarray | None = None, dt: float = DT_MDL):
    self.x = np.asarray(ModelConstants.X_IDXS if x_idxs is None else x_idxs, dtype=np.float64)
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    """Discard all lane-policy state so the caller immediately receives raw E2E."""
    self._lane_curvature = 0.0
    self._full_active = False
    self._arm_time = 0.0
    self._width = 3.7
    self._width_valid = False
    self._line_loss_time = 0.0
    self._center_correction = 0.0
    self._has_center_correction = False
    self._one_line_hold = False
    self._last_turn_sign = 0
    self._turn_release_time = 0.0
    self._last_logged_mode: str | None = None
    self._last_log_time = 0.0
    self._error_logged = False

  @property
  def is_active(self) -> bool:
    return self._full_active

  @property
  def is_blending(self) -> bool:
    """READY while the two-line timer arms, HOLD while a line is reconstructed."""
    return self._one_line_hold or (not self._full_active and self._arm_time > 0.0)

  def _log_mode(self, mode: str) -> None:
    """Log state transitions at a bounded rate for rlog validation."""
    now = time.monotonic()
    if mode != self._last_logged_mode and now - self._last_log_time >= LANE_LOCK_LOG_INTERVAL:
      cloudlog.info(f"ui-lp-full-center: {mode}")
      self._last_logged_mode = mode
      self._last_log_time = now

  def update(self, model_output: dict[str, np.ndarray], e2e_curvature: float, v_ego: float,
             blinkers_active: bool = False, enabled: bool = False) -> float:
    """Return the commanded curvature for this model frame."""
    e2e_curvature = float(e2e_curvature)

    if not enabled:
      self.reset()
      self._log_mode("stock-e2e (lane toggle off)")
      return e2e_curvature

    if blinkers_active:
      self.reset()
      self._log_mode("stock-e2e fallback: blinker")
      return e2e_curvature

    if v_ego < LANE_LOCK_MIN_SPEED:
      # Keep the learned width (it only comes from validated two-line frames)
      # but drop arm progress and any correction so a launch is always fed by
      # a freshly ramped correction instead of stale stationary state.
      self._arm_time = 0.0
      self._full_active = False
      self._has_center_correction = False
      self._center_correction = 0.0
      self._one_line_hold = False
      self._line_loss_time = 0.0
      self._last_turn_sign = 0
      self._turn_release_time = 0.0
      self._log_mode("stock-e2e fallback: low speed")
      return e2e_curvature

    try:
      # Lane axes are [batch, lane, distance, coordinate]. The inner lines are
      # lane 1 (left) and 2 (right); their raw confidences are entries 3 and 5.
      left_y = model_output['lane_lines'][0, 1, :, 0].astype(np.float64)
      right_y = model_output['lane_lines'][0, 2, :, 0].astype(np.float64)
      left_prob, right_prob = get_inner_lane_line_probs(model_output)
      desire_state = model_output['desire_state'][0]
      lane_change_prob = float(desire_state[log.Desire.laneChangeLeft] +
                               desire_state[log.Desire.laneChangeRight])
      if lane_change_prob > LANE_LOCK_MAX_LANE_CHANGE_PROB:
        self.reset()
        self._log_mode("stock-e2e fallback: lane-change intent")
        return e2e_curvature

      fit = (self.x >= LANE_LOCK_FIT_START) & (self.x <= LANE_LOCK_FIT_END)
      if self.x.shape != left_y.shape or np.count_nonzero(fit) < 3:
        raise ValueError("lane-line horizon does not match the model X_IDXS")

      valid_left = (np.isfinite(left_prob) and left_prob >= LANE_LOCK_RETAIN_LINE_PROB and
                    np.all(np.isfinite(left_y[fit])))
      valid_right = (np.isfinite(right_prob) and right_prob >= LANE_LOCK_RETAIN_LINE_PROB and
                     np.all(np.isfinite(right_y[fit])))
      two_line_geometry = False
      two_line_confidence = min(left_prob, right_prob)
      center_y: np.ndarray | None = None

      if valid_left and valid_right:
        measured_width, two_line_geometry = get_lane_width_measurement(left_y, right_y, fit)
        if two_line_geometry:
          center_y = 0.5 * (left_y + right_y)
          self._line_loss_time = 0.0
          self._one_line_hold = False

          # Preserve the old lane planner's long (~10 s) width memory. It is
          # only updated from strong, two-line geometry, never from a single
          # line, so it is always safe to keep across fallbacks.
          if two_line_confidence >= LANE_LOCK_ARM_LINE_PROB:
            if not self._width_valid:
              self._width = measured_width
              self._width_valid = True
            else:
              alpha = min(self.dt / LANE_LOCK_WIDTH_TIME, 1.0)
              self._width += alpha * (measured_width - self._width)

      if center_y is None and self._full_active and self._width_valid:
        # If a side is briefly clipped (intersections, dashed paint, shadows),
        # reconstruct its midpoint from the remaining reliable line and learned
        # width. If both visible lines disagree about width, choose the more
        # confident side rather than injecting their bad midpoint.
        selected_left = valid_left and (not valid_right or not two_line_geometry or left_prob >= right_prob)
        selected_right = valid_right and not selected_left
        if selected_left or selected_right:
          self._line_loss_time += self.dt
          if self._line_loss_time <= LANE_LOCK_ONE_LINE_HOLD_TIME:
            center_y = (left_y + self._width / 2.0 if selected_left else
                        right_y - self._width / 2.0)
            self._one_line_hold = True

      if center_y is None:
        if self._full_active or self._width_valid:
          # While inactive, keep the learned width but restart arming: a few
          # bad frames (overpasses, camera dirt) should not cost the ~10 s of
          # width learning it took to get here.
          self._arm_time = 0.0
          self._full_active = False
          self._has_center_correction = False
          self._center_correction = 0.0
          self._one_line_hold = False
          self._line_loss_time = 0.0
          self._last_turn_sign = 0
          self._turn_release_time = 0.0
        else:
          self.reset()
        self._log_mode("stock-e2e fallback: lane geometry or confidence")
        return e2e_curvature

      if not self._full_active:
        if two_line_geometry and two_line_confidence >= LANE_LOCK_ARM_LINE_PROB:
          self._arm_time = min(self._arm_time + self.dt, LANE_LOCK_ARM_TIME)
        else:
          self._arm_time = 0.0

        if self._arm_time < LANE_LOCK_ARM_TIME:
          self._one_line_hold = False
          self._log_mode("stock-e2e fallback: arming lane confidence")
          return e2e_curvature

        self._full_active = True
        self._line_loss_time = 0.0

      # The center fit uses a farther, measured horizon. The E2E command keeps
      # the curve feed-forward; only lane-center offset and heading become the
      # correction. This is deliberately not an E2E/lane-curvature blend.
      _, heading, offset = np.polyfit(self.x[fit], center_y[fit], 2)
      lookahead = float(np.clip(2.0 * v_ego, LANE_LOCK_MIN_LOOKAHEAD, LANE_LOCK_MAX_LOOKAHEAD))
      center_correction = (2.0 * (LANE_LOCK_HEADING_GAIN * heading * lookahead + offset) /
                           (lookahead * lookahead))
      center_correction = float(np.clip(center_correction,
                                        -LANE_LOCK_MAX_CENTER_CORRECTION,
                                        LANE_LOCK_MAX_CENTER_CORRECTION))
      if abs(center_correction) < LANE_LOCK_CORRECTION_DEADBAND:
        center_correction = 0.0

      # On a meaningful E2E turn-direction change, drop only a correction
      # that still asks for the previous turn. This preserves full lane
      # centering during steady curves while avoiding a direction-change tug.
      if abs(e2e_curvature) >= LANE_LOCK_TURN_CURVATURE:
        e2e_turn_sign = 1 if e2e_curvature > 0.0 else -1
        if self._last_turn_sign and e2e_turn_sign != self._last_turn_sign:
          self._turn_release_time = LANE_LOCK_TURN_RELEASE_TIME
        self._last_turn_sign = e2e_turn_sign
      if self._turn_release_time > 0.0:
        self._turn_release_time = max(0.0, self._turn_release_time - self.dt)
        if center_correction * e2e_curvature < 0.0:
          center_correction = 0.0

      # Enter smoothly after arming. Build correction deliberately, but release
      # it faster when the target shrinks or reverses after a curve/lane change.
      if not self._has_center_correction:
        self._center_correction = 0.0
        self._has_center_correction = True

      correction_step = (LANE_LOCK_CORRECTION_RELEASE_STEP
                         if (abs(center_correction) < abs(self._center_correction) or
                             center_correction * self._center_correction < 0.0)
                         else LANE_LOCK_CORRECTION_ENGAGE_STEP)
      delta = float(np.clip(center_correction - self._center_correction,
                            -correction_step, correction_step))
      self._center_correction += delta

      self._lane_curvature = e2e_curvature + self._center_correction
      self._error_logged = False
      self._log_mode("lane center hold" if self._one_line_hold else "full lane center")
      return self._lane_curvature

    except (KeyError, IndexError, TypeError, ValueError, FloatingPointError, np.linalg.LinAlgError) as err:
      self.reset()
      if not self._error_logged:
        cloudlog.warning(f"ui-lp-full-center input error: {type(err).__name__}: {err}")
        self._error_logged = True
      self._log_mode("stock-e2e fallback: lane-policy input error")
      return e2e_curvature
