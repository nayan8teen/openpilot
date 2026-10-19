"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Laneful v2.

The model's plan is not ground truth.  Laneful instead measures the lane center
from the model's lane lines, measures the car's actual curvature, predicts where
the car will be when a curvature request actually reaches the road, and asks for
the curvature that puts the car on the lane center at the preview point.

The offset is a trim on top of the model action.  It is never a replacement for
the model, it fades out when lane lines are weak or ambiguous, and it is capped
at a small fraction of the lateral acceleration budget.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan

LANE_WIDTH_DEFAULT = 3.7  # m
LANE_WIDTH_CLIP = (2.6, 4.8)  # m
LANE_WIDTH_TAU = 5.0  # s
MIN_LINE_PROB = 0.35
MIN_FRAME_SPAN = 1.0  # m, smaller left/right separation means the lines crossed
LINE_STD_LO = 0.20  # m
LINE_STD_HI = 0.60  # m
TRUST_FULL = 0.80  # both-line confidence at which laneful is fully trusted
LANE_X_MAX = 120.0  # m, ignore lane line points beyond the useful preview range

MIN_SPEED = 2.0  # m/s
LOOKAHEAD_TIME: float = 0.8  # s
MIN_LOOKAHEAD = 10.0  # m
MAX_HORIZON = 3.0  # s, hard stop for the predictor

# Lane-point weighting.  The model reports a std per point; confidence and the
# curvature fit must use the std where the correction is applied, not at x = 0
# where every line converges and the std is artificially small.
POINT_STD = 0.10  # m, floor on the point std so the fit does not over-trust the model
# Confidence is read near the preview distance, not beyond it.  Far-field line
# std grows with range, so sampling past the preview makes sharp curves look
# untrustworthy exactly when the correction is needed there.
CONF_WINDOW = (0.5, 1.0)  # multiples of the preview distance for line confidence
FIT_WINDOW = (0.25, 1.0)  # multiples of the preview distance for the curvature fit
FIT_MIN_POINTS = 6
FIT_MIN_SPAN = 5.0  # m
FIT_MAX_DELTA = 0.02  # 1/m, safety cap on the raw least-squares fit

# Vehicle/actuator response used by the predictor.  These are intentionally
# conservative defaults; calibration can override them.
PLANT_TAU = 0.18  # s
PLANT_DEAD_TIME = 0.06  # s
ACTION_DELAY = DT_MDL / 2.0  # s, controls applies the newest action next cycle

# Trim limits.  Laneful may only use a fraction of the lateral budget; the
# model remains responsible for the bulk of the command.  The budget caps what
# laneful adds, never what the model already asked for.
MAX_CURVATURE = 0.20  # 1/m
MAX_LATERAL_ACCEL = 3.0  # m/s^2 total
MAX_TRIM_ACCEL = 1.5  # m/s^2 from the laneful trim alone
MAX_TRIM_DELTA = 0.004  # 1/m absolute cap
SENSITIVITY_DEADBAND = 5e-4  # 1/m, use the direct formula near zero
DELTA_DEADBAND = 1e-4  # 1/m, ignore sub-centimeter-scale model noise

# Offset smoothing.  Attack faster than release so a developing lane departure
# is caught, while a line dropout does not snap the wheel back.
OFFSET_ATTACK_TAU = 0.10  # s
OFFSET_RELEASE_TAU = 0.30  # s
OFFSET_RATE_LIMIT = 0.05  # 1/m/s

TRUST_ATTACK_TAU = 0.10  # s
TRUST_RELEASE_TAU = 0.30  # s

# Plan intent.  The model's five plan hypotheses describe where it wants to go.
# A construction nudge or an obstacle pass shows up as high-probability
# hypotheses that leave the lane and then return.  Laneful must not pull the
# car back to center through that maneuver.
#
# The raw intent score is compared against its recent p99 baseline, not a fixed
# threshold.  Normal lane keeping produces a small score; a rare intentional
# departure exceeds the 99th percentile of the recent normal-driving scores.
INTENT_HISTORY_SEC = 30.0  # s of normal-driving scores kept for the p99
INTENT_P99_MIN_SAMPLES = 200  # samples before the p99 replaces the floor
INTENT_P99_FLOOR = 0.30  # raw score treated as "high" before a baseline exists
INTENT_FULL = 0.75  # raw score that maps to full suppression
INTENT_DEV_LO = 0.25  # m, departure below this is normal lane keeping
INTENT_DEV_HI = 0.60  # m, departure at or above this is a clear lane departure
INTENT_PATH_TOL_LO = 0.30  # m, base path within this of the plan is agreement
INTENT_PATH_TOL_HI = 0.90  # m, base path beyond this is not following the plan
INTENT_TOP_WEIGHT_FULL = 0.50  # top hypothesis weight for full intent confidence
INTENT_YIELD_LO = 0.50  # m, selected-plan drift below this is ignored
INTENT_YIELD_HI = 1.00  # m, selected-plan drift at or above this fully yields
INTENT_AWAY_LO = 0.00  # m, action path must at least hold the current offset
INTENT_AWAY_HI = 0.40  # m, action path moving this much further off is intentional
INTENT_RETURN_LO = 0.00  # m, action heading back toward center
INTENT_RETURN_HI = 0.60  # m, action returning this much faster is not intentional
# Plans are interpolated trajectories, so on a curve their heading can lag the
# plant and the plan appears offset from lane center by roughly k*x^2/2 even
# when it is only following the lane.  Ignore this fraction of that geometric
# offset before treating a plan/hypothesis departure as intent.
INTENT_CURVE_TOL = 0.25

# Where the intent score came from, published as a diagnostic.
INTENT_SOURCE_NONE = 0
INTENT_SOURCE_ACTION = 1
INTENT_SOURCE_HYPOTHESIS = 2
INTENT_SOURCE_PLAN = 3
INTENT_HISTORY_MAX = 0.50  # scores at or above this are not used for the baseline
INTENT_ATTACK_TAU = 0.15  # s
INTENT_RELEASE_TAU = 0.50  # s
INTENT_MAX_ACTIVE_SEC = 12.0  # s, then taper back toward lane centering
INTENT_TAPER_SEC = 2.0  # s


@dataclass(frozen=True)
class LanefulState:
  """Vehicle state captured before model inference."""
  v_ego: float
  curvature: float  # measured actual curvature, 1/m
  lat_active: bool = True
  steering_pressed: bool = False
  t_mono: float = 0.0


@dataclass(frozen=True)
class LanefulTiming:
  """Latency terms for the predictor."""
  frame_age: float = DT_MDL
  model_exec: float = 0.0
  lat_delay: float = 0.0
  action_delay: float = ACTION_DELAY

  @property
  def apply_time(self) -> float:
    return max(0.0, self.frame_age + self.model_exec + self.lat_delay + self.action_delay)


@dataclass(frozen=True)
class LanefulOutput:
  """Result of one laneful update.

  curvature_offset is the value that should be added to the model's smoothed
  base curvature.  The remaining fields are diagnostic and are published in
  ModelDataV2SP for the UI, replay and calibration.
  """
  curvature_offset: float
  trust: float
  lane_curvature: float
  required_curvature: float
  predicted_curvature: float
  base_curvature: float
  lookahead: float
  lateral_error: float
  saturation: float
  valid: bool
  # Diagnostic: the same curvature change estimated from the lane-center
  # residual over the preview window, and from the rollout sensitivity.
  fit_delta: float = 0.0
  sensitivity_delta: float = 0.0
  # Plan intent: how much laneful defers to a deliberate model departure.
  intent: float = 0.0
  intent_raw: float = 0.0
  intent_threshold: float = INTENT_P99_FLOOR
  plan_departure: float = 0.0
  plan_weight: float = 0.0
  intent_source: int = INTENT_SOURCE_NONE


@dataclass(frozen=True)
class _Intent:
  intent: float
  raw: float
  threshold: float
  departure: float
  weight: float
  source: int = INTENT_SOURCE_NONE


@dataclass(frozen=True)
class _LaneCenter:
  x: np.ndarray
  y: np.ndarray
  trust: float
  width: float
  sigma: np.ndarray  # per-point std of the blended center estimate, m


def _smooth(current: float, target: float, dt: float, tau_attack: float, tau_release: float) -> float:
  """Asymmetric first-order filter.  Larger corrections attack, smaller release."""
  tau = tau_attack if abs(target) >= abs(current) else tau_release
  alpha = 1.0 - np.exp(-dt / max(tau, 1e-3))
  return float(current + alpha * (target - current))


def _smooth_rate(current: float, target: float, dt: float,
                 tau_attack: float, tau_release: float, rate_limit: float) -> float:
  candidate = _smooth(current, target, dt, tau_attack, tau_release)
  max_step = rate_limit * dt
  return float(current + np.clip(candidate - current, -max_step, max_step))


def _smoothstep(lo: float, hi: float, value: float) -> float:
  if hi <= lo:
    return 0.0
  t = float(np.clip((value - lo) / (hi - lo), 0.0, 1.0))
  return t * t * (3.0 - 2.0 * t)


def _interp_conf(prob: float, std: float) -> float:
  if not np.isfinite(prob) or not np.isfinite(std):
    return 0.0
  return float(np.clip(prob, 0.0, 1.0) * np.interp(std, [LINE_STD_LO, LINE_STD_HI], [1.0, 0.0]))


def _window_conf(prob: float, sigma: np.ndarray, x: np.ndarray, lookahead: float) -> float:
  """Line confidence using the std over the preview window, not at the camera."""
  if not np.isfinite(prob):
    return 0.0
  lo = max(CONF_WINDOW[0] * lookahead, float(x[0]))
  hi = min(CONF_WINDOW[1] * lookahead, float(x[-1]))
  mask = np.isfinite(sigma) & (x >= lo) & (x <= hi)
  if mask.sum() < 2:
    mask = np.isfinite(sigma)
  if not mask.any():
    return 0.0
  return _interp_conf(float(prob), float(np.median(sigma[mask])))


def _line_stds(stds: np.ndarray, n: int) -> np.ndarray:
  """Normalize lane_lines_stds to a (4, n) per-point std array."""
  s = np.asarray(stds, dtype=float)
  if s.ndim == 4:
    s = s[0]
  elif s.ndim == 3:
    if s.shape[0] == 1:
      s = s[0]
  elif s.ndim == 1:
    s = np.repeat(s[:, None], n, axis=1)
  if s.ndim == 3:
    s = s[..., 0]
  if s.ndim != 2 or min(s.shape) < 2:
    return np.full((4, n), LINE_STD_HI, dtype=float)
  if s.shape[0] != 4 and s.shape[1] == 4:
    s = s.T
  if s.shape[0] < 4:
    return np.full((4, n), LINE_STD_HI, dtype=float)
  s = s[:4]
  if s.shape[1] != n:
    src = np.linspace(0.0, 1.0, s.shape[1])
    tgt = np.linspace(0.0, 1.0, n)
    s = np.vstack([np.interp(tgt, src, s[i]) for i in range(4)])
  return s


def _fill_nan(y: np.ndarray) -> np.ndarray | None:
  finite = np.isfinite(y)
  if finite.sum() < 2:
    return None
  idx = np.arange(len(y))
  return np.interp(idx, idx[finite], y[finite])


class Laneful:
  def __init__(self, tau: float = PLANT_TAU, dead_time: float = PLANT_DEAD_TIME):
    self.tau = max(float(tau), 1e-3)
    self.dead_time = max(float(dead_time), 0.0)
    self._lane_width = LANE_WIDTH_DEFAULT
    self._offset = 0.0
    self._trust = 0.0
    self._prev_t: float | None = None
    self._intent = 0.0
    self._intent_raw = 0.0
    self._intent_threshold = INTENT_P99_FLOOR
    self._intent_history: deque[float] = deque(maxlen=int(INTENT_HISTORY_SEC / DT_MDL))
    self._intent_p99_timer = 0.0
    self._intent_active_time = 0.0

  def reset(self) -> None:
    """Drop transient state and start from laneless behavior."""
    self._lane_width = LANE_WIDTH_DEFAULT
    self._offset = 0.0
    self._trust = 0.0
    self._prev_t = None
    self._intent = 0.0
    self._intent_raw = 0.0
    self._intent_threshold = INTENT_P99_FLOOR
    self._intent_history.clear()
    self._intent_p99_timer = 0.0
    self._intent_active_time = 0.0

  def update(self, model_output: dict, state: LanefulState, timing: LanefulTiming,
             base_curvature: float, prev_curvature: float | None = None,
             strength: float = 1.0, desire_active: bool = False,
             dt: float | None = None) -> LanefulOutput:
    """Compute the curvature trim for the current model frame.

    base_curvature is the model-requested curvature *before* smoothing, i.e. the
    value from get_base_action().  prev_curvature is the previously applied
    total curvature (base + previous trim).  Keeping them separate prevents the
    trim from feeding back into the base-action smoothing loop.
    """
    if dt is None:
      if self._prev_t is None or state.t_mono <= self._prev_t:
        dt = DT_MDL
      else:
        dt = float(np.clip(state.t_mono - self._prev_t, 0.005, 0.20))
    self._prev_t = state.t_mono
    dt = float(np.clip(dt, 0.005, 0.20))

    base = float(base_curvature) if np.isfinite(base_curvature) else 0.0
    prev = float(prev_curvature) if prev_curvature is not None and np.isfinite(prev_curvature) else base

    # Hard disengagements clear the trim.  Transient suppressions fade it.
    if strength <= 0.0 or not state.lat_active:
      self.reset()
      return self._inactive(base)

    if not np.isfinite(state.v_ego) or state.v_ego < MIN_SPEED:
      return self._fade(base, state, timing, strength, dt)

    if desire_active or state.steering_pressed:
      return self._fade(base, state, timing, strength, dt)

    v = float(state.v_ego)
    preview_x = max(v * LOOKAHEAD_TIME, MIN_LOOKAHEAD)
    lane = self._lane_center(model_output, dt, preview_x)
    if lane is None:
      return self._fade(base, state, timing, strength, dt)

    kappa_meas = float(state.curvature) if np.isfinite(state.curvature) else base
    t_apply = timing.apply_time
    T_p = max(LOOKAHEAD_TIME, MIN_LOOKAHEAD / max(v, 1.0))

    # Roll out the current command to see where the car will actually be when
    # the command reaches the road, plus a small sensitivity pass.
    x_b, y_b, psi_b, kappa_b, apply_idx = self._rollout(
      v, kappa_meas, prev, base, 0.0, t_apply, T_p, dt)
    x_d, y_d, _, _, _ = self._rollout(
      v, kappa_meas, prev, base, 1e-3, t_apply, T_p, dt)

    x_a = float(x_b[apply_idx])
    x_p = float(x_b[-1])
    y_a = float(y_b[apply_idx])
    psi_a = float(psi_b[apply_idx])
    kappa_act_apply = float(kappa_b[apply_idx])

    if not (np.isfinite(x_a) and np.isfinite(x_p) and x_p > x_a + 1.0):
      return self._fade(base, state, timing, strength, dt)

    fit = self._local_fit(lane.x, lane.y, x_a, half=12.0, sigma=lane.sigma)
    if fit is None:
      return self._fade(base, state, timing, strength, dt)
    slope_a, _ = fit

    # If the trusted lane center stops short of the requested preview, shorten
    # the preview instead of dropping the correction.  Noisy far-field lines
    # cost authority at that distance, not all of it.
    x_preview = min(x_p, float(lane.x[-1]))
    if x_preview < x_a + 2.0 or x_preview < lane.x[0]:
      return self._fade(base, state, timing, strength, dt)
    y_lane_p = float(np.interp(x_preview, lane.x, lane.y))
    y_lane_a = float(np.interp(x_a, lane.x, lane.y))
    psi_lane_a = float(np.arctan(slope_a))

    lane_fit_p = self._local_fit(lane.x, lane.y, x_preview, half=12.0, sigma=lane.sigma)
    if lane_fit_p is None:
      return self._fade(base, state, timing, strength, dt)
    slope_p, quad_p = lane_fit_p
    lane_curvature_p = 2.0 * quad_p / (1.0 + slope_p * slope_p) ** 1.5
    L_p = max(x_preview - x_a, 1.0)
    y_b_p = float(np.interp(x_preview, x_b, y_b))
    y_d_p = float(np.interp(x_preview, x_d, y_d))

    # Newer models expose `action`, the direct lateral control output.  Predict
    # where that action settles, without laneful's previous trim and without
    # the plant's current tracking lag.  This avoids interpolating the slower
    # plan and matches the plant heading the action was trained against.
    #
    # The lag matters on sharp curves: entering a corner the car is still
    # under-curved even when the action already asks for the lane curvature, so
    # a transient rollout would look like a deliberate departure and switch
    # laneful off exactly where it is needed.
    action_departure: float | None = None
    action_away = 0.0
    if 'action' in model_output:
      x_act, y_act, _, _, _ = self._rollout(v, base, base, base, 0.0, t_apply, T_p, dt)
      if np.isfinite(x_act[-1]) and x_act[-1] > x_a + 1.0:
        x_action = min(x_preview, float(x_act[-1]))
        y_action = float(np.interp(x_action, x_act, y_act))
        y_lane_action = float(np.interp(x_action, lane.x, lane.y))
        action_departure = y_action - y_lane_action
        center_now = float(np.interp(0.0, lane.x, lane.y))
        action_away = abs(action_departure) - abs(center_now)

    # Deliberate model departures (construction, obstacles) must not be pulled
    # back to center.  Action-derived departure is authoritative when present;
    # hypotheses supply confidence and are the fallback.
    intent = self._plan_intent(model_output, lane, x_preview, y_lane_p, y_b_p, dt,
                               lane_curvature_p, action_departure, action_away)

    # Required curvature at the preview point, composed from:
    #   lane curvature + heading catch-up + lateral error catch-up.
    e_y = y_lane_a - y_a
    e_psi = psi_lane_a - psi_a
    kappa_need = lane_curvature_p + 2.0 * (e_y + L_p * e_psi) / (L_p * L_p)

    # Sensitivity form of the same correction: how far does the preview point
    # move per unit of requested curvature?  This automatically accounts for
    # actuator lag and a changing lane curvature.
    delta_formula = float(kappa_need - kappa_act_apply)
    delta_sens: float | None = None
    H = (y_d_p - y_b_p) / 1e-3
    if np.isfinite(H) and abs(H) > 1e-6:
      candidate = (y_lane_p - y_b_p) / H
      if np.isfinite(candidate):
        delta_sens = float(candidate)

    # Independent estimate from the measured lane center: fit the curvature
    # change that lays the rollout arc onto the lane-center points over the
    # preview window.  This is a fallback for when the finite-difference
    # sensitivity is degenerate or has the wrong sign, and a diagnostic
    # everywhere else.
    delta_fit = self._fit_delta(lane, x_b, y_b, x_a, x_preview)

    delta = delta_formula
    if delta_sens is not None:
      # A tiny H or a sign disagreement means the linearization is not useful.
      if abs(delta_formula) < SENSITIVITY_DEADBAND:
        # Near zero, discrete integration noise in the two rollouts is larger
        # than the signal.  Use the direct feedforward form.
        delta = delta_formula
      elif np.sign(delta_sens) == np.sign(delta_formula):
        delta = delta_sens
      elif delta_fit is not None:
        delta = delta_fit
    elif delta_fit is not None:
      delta = delta_fit

    if abs(delta) < DELTA_DEADBAND:
      delta = 0.0

    # Per-frame caps.  The trim may only use a small part of the lateral budget.
    kappa_max = min(MAX_CURVATURE, MAX_LATERAL_ACCEL / (v * v))
    delta_max = min(MAX_TRIM_DELTA, MAX_TRIM_ACCEL / (v * v))
    delta = float(np.clip(delta, -delta_max, delta_max))
    # The budget limits what laneful may add; it never takes curvature away from
    # the model.  A sharp curve whose base request already exceeds the budget is
    # the model's call, and subtracting here would understeer the car.  A
    # negative trim toward lane center is still allowed.
    if delta > 0.0:
      delta = min(delta, max(0.0, kappa_max - base))

    # Trust smoothing, then offset smoothing with a rate limit.  The offset is
    # filtered after multiplying by trust so a confidence collapse releases
    # the trim instead of building a hidden correction.  Plan intent scales
    # that authority down over the maneuver instead of switching it off.
    authority = float(lane.trust * (1.0 - intent.intent))
    target = float(delta * authority)
    self._offset = _smooth_rate(self._offset, target, dt, OFFSET_ATTACK_TAU,
                                OFFSET_RELEASE_TAU, OFFSET_RATE_LIMIT)

    offset = float(np.clip(self._offset * strength, -delta_max, delta_max))
    saturation = float(abs(offset) / delta_max) if delta_max > 0 else 0.0

    return LanefulOutput(
      curvature_offset=offset,
      trust=float(self._trust),
      lane_curvature=float(lane_curvature_p),
      required_curvature=float(kappa_need),
      predicted_curvature=float(kappa_act_apply),
      base_curvature=base,
      lookahead=float(x_preview),
      lateral_error=float(y_lane_p - y_b_p),
      saturation=saturation,
      valid=True,
      fit_delta=float(delta_fit) if delta_fit is not None else 0.0,
      sensitivity_delta=float(delta_sens) if delta_sens is not None else 0.0,
      intent=float(intent.intent),
      intent_raw=float(intent.raw),
      intent_threshold=float(intent.threshold),
      plan_departure=float(intent.departure),
      plan_weight=float(intent.weight),
      intent_source=int(intent.source),
    )

  def _inactive(self, base: float) -> LanefulOutput:
    return LanefulOutput(
      curvature_offset=0.0, trust=0.0, lane_curvature=0.0,
      required_curvature=base, predicted_curvature=base,
      base_curvature=base, lookahead=0.0, lateral_error=0.0,
      saturation=0.0, valid=False,
    )

  def _fade(self, base: float, state: LanefulState, timing: LanefulTiming,
            strength: float, dt: float) -> LanefulOutput:
    self._offset = _smooth_rate(self._offset, 0.0, dt, OFFSET_ATTACK_TAU,
                                OFFSET_RELEASE_TAU, OFFSET_RATE_LIMIT)
    self._trust = _smooth(self._trust, 0.0, dt, TRUST_ATTACK_TAU, TRUST_RELEASE_TAU)
    self._intent = _smooth(self._intent, 0.0, dt, INTENT_ATTACK_TAU, INTENT_RELEASE_TAU)
    self._intent_raw = 0.0
    self._intent_active_time = 0.0
    return LanefulOutput(
      curvature_offset=float(self._offset * strength),
      trust=float(self._trust),
      lane_curvature=0.0,
      required_curvature=base,
      predicted_curvature=base,
      base_curvature=base,
      lookahead=0.0,
      lateral_error=0.0,
      saturation=0.0,
      valid=False,
      intent=float(self._intent),
      intent_raw=0.0,
      intent_threshold=float(self._intent_threshold),
      intent_source=INTENT_SOURCE_NONE,
    )

  def _lane_center(self, model_output: dict, dt: float, lookahead: float) -> _LaneCenter | None:
    if 'lane_lines' not in model_output or 'lane_lines_prob' not in model_output \
       or 'lane_lines_stds' not in model_output:
      return None

    lines = np.asarray(model_output['lane_lines'])
    if lines.ndim != 4 or lines.shape[0] < 1 or lines.shape[1] < 3 or lines.shape[3] < 2:
      return None
    lines = lines[0]

    # The model tensor layout is (line, point, [y, z]); x is ModelConstants.X_IDXS.
    # This matches fill_model_msg.py: fill_xyzt(..., X_IDXS, lane_lines[...,0], lane_lines[...,1]).
    x = np.asarray(ModelConstants.X_IDXS, dtype=float)
    if lines.shape[1] != x.size:
      x = np.linspace(0.0, float(ModelConstants.X_IDXS[-1]), lines.shape[1])

    keep = np.isfinite(x) & (x >= 0.0) & (x <= LANE_X_MAX)
    if keep.sum() < 6:
      return None
    x = x[keep]

    left = np.asarray(lines[1, :, 0], dtype=float)[keep]
    right = np.asarray(lines[2, :, 0], dtype=float)[keep]

    probs = np.asarray(model_output['lane_lines_prob'], dtype=float).reshape(-1)
    if probs.size < 8:
      return None
    probs = probs[1::2]
    if probs.size < 3:
      return None

    # Keep the std along the line.  Collapsing to the first point makes every
    # line look certain, because all four lines converge at x = 0.
    stds = _line_stds(model_output['lane_lines_stds'], lines.shape[1])
    sigma_l = stds[1][keep]
    sigma_r = stds[2][keep]

    p_l = _window_conf(probs[1], sigma_l, x, lookahead)
    p_r = _window_conf(probs[2], sigma_r, x, lookahead)

    # Lane-line y lives in the model's own lateral frame, and that frame is not
    # fixed across models: some emit y positive to the left, the v2 models emit
    # y positive to the right.  Measure which way the left line runs to the
    # right one instead of assuming a convention.  `direction` is +1 when the
    # right line sits at the larger y, -1 when the left line does.
    widths = right - left
    finite_widths = widths[np.isfinite(widths)]
    if finite_widths.size == 0:
      return None
    median_width = float(np.median(finite_widths))
    if abs(median_width) < MIN_FRAME_SPAN:
      # The lines cross over each other or sit on top of each other, so which
      # one is on the left cannot be resolved.  Drop the correction for this
      # frame instead of guessing.
      return None
    direction = 1.0 if median_width > 0.0 else -1.0
    span = widths * direction  # lane width per point, always positive

    # Lane width is estimated only when both lines are trustworthy at the same
    # point.  A single outlier must not shift a one-line fallback.
    # A one-line fallback must not learn width from the untrusted other line.
    width_mask = (np.isfinite(span) & (x >= 5.0) & (x <= 60.0)
                  & (p_l >= MIN_LINE_PROB) & (p_r >= MIN_LINE_PROB))
    if width_mask.sum() >= 5:
      width_med = float(np.median(span[width_mask]))
      if LANE_WIDTH_CLIP[0] <= width_med <= LANE_WIDTH_CLIP[1]:
        self._lane_width = _smooth(self._lane_width, width_med, dt, LANE_WIDTH_TAU, LANE_WIDTH_TAU)
    width = float(self._lane_width)

    left_f = _fill_nan(left)
    right_f = _fill_nan(right)

    center_left = np.full_like(left, np.nan)
    if left_f is not None:
      dy_l = np.gradient(left_f, x)
      norm = np.sqrt(1.0 + dy_l * dy_l)
      # Half a lane width from the left line, toward the right line.
      center_left = left + direction * width / 2.0 / norm
    center_right = np.full_like(right, np.nan)
    if right_f is not None:
      dy_r = np.gradient(right_f, x)
      norm = np.sqrt(1.0 + dy_r * dy_r)
      # Half a lane width from the right line, back toward the left line.
      center_right = right - direction * width / 2.0 / norm

    if p_l >= MIN_LINE_PROB and p_r >= MIN_LINE_PROB:
      # Normalize so two good lines are fully trusted; weak lines still fade.
      trust_raw = min(1.0, p_l * p_r / TRUST_FULL)
    elif p_l >= MIN_LINE_PROB:
      trust_raw = 0.5 * p_l
    elif p_r >= MIN_LINE_PROB:
      trust_raw = 0.5 * p_r
    else:
      self._trust = _smooth(self._trust, 0.0, dt, TRUST_ATTACK_TAU, TRUST_RELEASE_TAU)
      return None

    # Blend the two center estimates by inverse variance instead of switching
    # hard at the confidence gate.  A noisy line then contributes less without
    # being discarded.  Variance includes the line probability so a low-prob
    # line is also down-weighted point by point.
    var_l = sigma_l * sigma_l / max(float(probs[1]), 1e-4)
    var_r = sigma_r * sigma_r / max(float(probs[2]), 1e-4)
    usable_l = (np.isfinite(center_left) & np.isfinite(var_l) & (var_l > 1e-8)
                & (sigma_l <= LINE_STD_HI) & (probs[1] >= MIN_LINE_PROB))
    usable_r = (np.isfinite(center_right) & np.isfinite(var_r) & (var_r > 1e-8)
                & (sigma_r <= LINE_STD_HI) & (probs[2] >= MIN_LINE_PROB))

    n = x.size
    center = np.full(n, np.nan)
    sigma = np.full(n, np.nan)
    both = usable_l & usable_r
    if both.any():
      w_l = 1.0 / var_l[both]
      w_r = 1.0 / var_r[both]
      denom = w_l + w_r
      center[both] = (w_l * center_left[both] + w_r * center_right[both]) / denom
      sigma[both] = np.sqrt(1.0 / denom)
    only_l = usable_l & ~usable_r
    center[only_l] = center_left[only_l]
    sigma[only_l] = sigma_l[only_l]
    only_r = usable_r & ~usable_l
    center[only_r] = center_right[only_r]
    sigma[only_r] = sigma_r[only_r]

    valid = np.isfinite(center) & np.isfinite(x) & np.isfinite(sigma)
    if valid.sum() < 6 or x[valid][-1] < MIN_LOOKAHEAD:
      self._trust = _smooth(self._trust, 0.0, dt, TRUST_ATTACK_TAU, TRUST_RELEASE_TAU)
      return None

    cx = x[valid]
    cy = center[valid]
    cs = sigma[valid]
    if np.any(np.diff(cx) <= 0.0):
      return None

    self._trust = _smooth(self._trust, trust_raw, dt, TRUST_ATTACK_TAU, TRUST_RELEASE_TAU)
    return _LaneCenter(x=cx, y=cy, trust=float(self._trust), width=width, sigma=cs)

  def _fit_delta(self, lane: _LaneCenter, x_path: np.ndarray, y_path: np.ndarray,
                 x_a: float, x_p: float) -> float | None:
    """Weighted least-squares curvature change that fits the lane center.

    The residual between the lane center and the rollout arc is fit with
    ``e(x) = a + b*x + 0.5*dk*x^2``.  The constant and linear terms absorb the
    current offset and heading error, so only the curvature term is used.
    This is the same arc error the preview-point correction targets, but uses
    every point in the preview window and weights each by its measured std.
    """
    lo = max(FIT_WINDOW[0] * x_p, x_a, float(lane.x[0]))
    # Never extrapolate the rollout: it ends at the preview point.
    hi = min(FIT_WINDOW[1] * x_p, float(x_path[-1]), float(lane.x[-1]))
    mask = (np.isfinite(lane.y) & np.isfinite(lane.sigma)
            & (lane.x >= lo) & (lane.x <= hi))
    if mask.sum() < FIT_MIN_POINTS:
      return None

    xs = lane.x[mask]
    if float(xs[-1] - xs[0]) < FIT_MIN_SPAN or float(x_path[-1]) <= float(x_path[0]):
      return None

    residual = lane.y[mask] - np.interp(xs, x_path, y_path)
    weight = 1.0 / np.sqrt(lane.sigma[mask] ** 2 + POINT_STD ** 2)
    design = np.column_stack((np.ones_like(xs), xs, 0.5 * xs * xs))
    try:
      coeff, *_ = np.linalg.lstsq(design * weight[:, None], residual * weight, rcond=None)
    except np.linalg.LinAlgError:
      return None

    dk = float(coeff[2])
    if not np.isfinite(dk):
      return None
    return float(np.clip(dk, -FIT_MAX_DELTA, FIT_MAX_DELTA))

  def _plan_intent(self, model_output: dict, lane: _LaneCenter, x_preview: float,
                   y_lane: float, y_path: float, dt: float, lane_curvature: float,
                   action_departure: float | None = None,
                   action_away: float = 0.0) -> _Intent:
    """Score whether the model deliberately wants to leave the lane.

    When the model exposes the direct `action` output, its predicted path is
    authoritative: it is the trained control output and already accounts for
    the plant heading.  Plan hypotheses are used only for confidence in that
    case.  Without `action`, the hypotheses (and finally the selected plan)
    provide the departure estimate.

    `action_departure` is measured from a settled rollout, so it carries no
    plant tracking lag.  The curvature tolerance below applies only to the
    plan-derived paths, which do carry interpolation lag.

    Plan-derived departures carry a curvature tolerance (see
    INTENT_CURVE_TOL): on a curve an interpolated plan can sit off the lane
    center without meaning to leave it, and treating that as intent would
    suppress centering on every sharp corner.
    """
    curve_tol = INTENT_CURVE_TOL * abs(float(lane_curvature)) * 0.5 * x_preview * x_preview
    if action_departure is not None:
      top_weight = self._top_hypothesis_weight(model_output)
      confidence = float(np.clip(top_weight / INTENT_TOP_WEIGHT_FULL, 0.0, 1.0))
      departure_mass = _smoothstep(INTENT_DEV_LO, INTENT_DEV_HI, abs(action_departure))
      # Penalize an action that is actively returning to center, but keep
      # yielding while it holds an offset (construction) or moves further out.
      returning = max(0.0, -action_away)
      away_factor = 1.0 - _smoothstep(INTENT_RETURN_LO, INTENT_RETURN_HI, returning)
      raw = departure_mass * away_factor * confidence
      return self._update_intent(raw, action_departure, top_weight, dt, INTENT_SOURCE_ACTION)

    hypotheses = model_output.get('plan_hypotheses')
    weights = model_output.get('plan_weights')
    if hypotheses is None or weights is None:
      return self._selected_plan_intent(model_output, lane, x_preview, y_lane, y_path, dt, curve_tol)

    hyp = np.asarray(hypotheses, dtype=float)
    w = np.asarray(weights, dtype=float).reshape(-1)
    if hyp.ndim == 4:
      hyp = hyp[0]
    if hyp.ndim != 3 or hyp.shape[0] < 1 or w.size != hyp.shape[0]:
      return self._update_intent(0.0, 0.0, 0.0, dt, INTENT_SOURCE_NONE)

    order = np.argsort(w)[::-1]
    hyp = hyp[order]
    w = w[order]

    stds = model_output.get('plan_stds_hypotheses')
    sigma = None
    if stds is not None:
      sigma = np.asarray(stds, dtype=float)
      if sigma.ndim == 4:
        sigma = sigma[0]
      if sigma.shape != hyp.shape:
        sigma = None
      else:
        sigma = sigma[order]

    departures: list[float] = []
    effective_weights: list[float] = []
    top_departure: float | None = None
    top_weight = 0.0
    top_path_error = 0.0
    for i in range(hyp.shape[0]):
      pos = hyp[i][:, Plan.POSITION]
      x = pos[:, 0]
      y = pos[:, 1]
      finite = np.isfinite(x) & np.isfinite(y)
      if finite.sum() < 2 or x_preview < x[finite][0] or x_preview > x[finite][-1]:
        continue
      xf = x[finite]
      if np.any(np.diff(xf) <= 0.0):
        continue
      y_at = float(np.interp(x_preview, xf, y[finite]))
      departure = y_at - y_lane
      path_error = y_at - y_path

      reliability = 1.0
      if sigma is not None:
        sy = sigma[i][:, Plan.POSITION][:, 1]
        if finite.sum() >= 2 and np.all(np.isfinite(sy[finite])):
          sy_at = float(np.interp(x_preview, xf, sy[finite]))
          reliability = 1.0 - _smoothstep(LINE_STD_LO, LINE_STD_HI + 0.2, abs(sy_at))

      departures.append(departure)
      effective_weights.append(float(w[i]) * max(reliability, 0.0))
      if i == 0:
        top_departure = departure
        top_weight = float(w[i]) * max(reliability, 0.0)
        top_path_error = path_error

    if not departures or top_departure is None:
      return self._update_intent(0.0, 0.0, float(w[0]), dt, INTENT_SOURCE_HYPOTHESIS)

    departure_array = np.asarray(departures)
    weight_array = np.asarray(effective_weights)
    weight_sum = float(np.sum(weight_array))
    if weight_sum <= 0.0:
      return self._update_intent(0.0, 0.0, float(w[0]), dt, INTENT_SOURCE_HYPOTHESIS)

    departure_mass = 0.0
    for departure, weight in zip(departure_array, weight_array, strict=True):
      magnitude = max(0.0, abs(departure) - curve_tol)
      departure_mass += weight * _smoothstep(INTENT_DEV_LO, INTENT_DEV_HI, magnitude)
    departure_mass /= weight_sum

    agreement = 1.0 - _smoothstep(INTENT_PATH_TOL_LO, INTENT_PATH_TOL_HI, abs(top_path_error))
    top_conf = float(np.clip(top_weight / INTENT_TOP_WEIGHT_FULL, 0.0, 1.0))

    raw = departure_mass * top_conf * agreement
    if abs(top_departure) - curve_tol < INTENT_DEV_LO:
      raw = 0.0
    return self._update_intent(raw, top_departure, top_weight, dt, INTENT_SOURCE_HYPOTHESIS)

  def _selected_plan_intent(self, model_output: dict, lane: _LaneCenter, x_preview: float,
                            y_lane: float, y_path: float, dt: float,
                            curve_tol: float = 0.0) -> _Intent:
    """Fallback when the model does not expose plan hypotheses.

    This follows the laneful-curves heuristic: if the selected plan ends up
    further off center than the car is now, the model is probably going around
    something and lane centering should yield.  The departure is reduced by the
    curvature tolerance first, because an interpolated plan on a sharp curve can
    sit off lane center without meaning to leave it, and the base-path agreement
    gate is still required so a plan-only departure does not disable laneful.
    """
    plan = model_output.get('plan')
    if plan is None:
      return self._update_intent(0.0, 0.0, 0.0, dt, INTENT_SOURCE_NONE)

    pos = np.asarray(plan, dtype=float)
    if pos.ndim == 4:
      pos = pos[0, 0]
    elif pos.ndim == 3:
      pos = pos[0]
    if pos.ndim != 2 or pos.shape[1] < 3:
      return self._update_intent(0.0, 0.0, 0.0, dt, INTENT_SOURCE_NONE)

    x = pos[:, 0]
    y = pos[:, 1]
    finite = np.isfinite(x) & np.isfinite(y)
    if finite.sum() < 2 or x_preview < x[finite][0] or x_preview > x[finite][-1]:
      return self._update_intent(0.0, 0.0, 0.0, dt, INTENT_SOURCE_NONE)
    xf = x[finite]
    if np.any(np.diff(xf) <= 0.0):
      return self._update_intent(0.0, 0.0, 0.0, dt, INTENT_SOURCE_NONE)

    y_plan = float(np.interp(x_preview, xf, y[finite]))
    departure = y_plan - y_lane
    center_now = float(np.interp(0.0, lane.x, lane.y))
    away = max(0.0, abs(departure) - abs(center_now) - curve_tol)

    reliability = 1.0
    stds = model_output.get('plan_stds')
    if stds is not None:
      s = np.asarray(stds, dtype=float)
      if s.ndim == 3:
        s = s[0]
      if s.ndim == 2 and s.shape[0] == x.size:
        sy = s[:, 1]
        if np.all(np.isfinite(sy[finite])):
          reliability = 1.0 - _smoothstep(LINE_STD_LO, LINE_STD_HI + 0.2,
                                           abs(float(np.interp(x_preview, xf, sy[finite]))))

    agreement = 1.0 - _smoothstep(INTENT_PATH_TOL_LO, INTENT_PATH_TOL_HI, abs(y_plan - y_path))
    raw = _smoothstep(INTENT_YIELD_LO, INTENT_YIELD_HI, away) * agreement * reliability
    return self._update_intent(raw, departure, 0.0, dt, INTENT_SOURCE_PLAN)

  @staticmethod
  def _top_hypothesis_weight(model_output: dict) -> float:
    weights = model_output.get('plan_weights')
    if weights is None:
      return 1.0
    w = np.asarray(weights, dtype=float).reshape(-1)
    if w.size == 0:
      return 1.0
    return float(np.clip(np.max(w), 0.0, 1.0))

  def _update_intent(self, raw: float, departure: float, weight: float, dt: float,
                     source: int = INTENT_SOURCE_NONE) -> _Intent:
    raw = float(np.clip(raw, 0.0, 1.0))

    # Learn the p99 of normal driving only.  Deliberate departures must not
    # raise the threshold that is supposed to detect them.
    if raw < INTENT_HISTORY_MAX:
      self._intent_history.append(raw)
      self._intent_p99_timer += dt
      if self._intent_p99_timer >= 1.0 and len(self._intent_history) >= INTENT_P99_MIN_SAMPLES:
        self._intent_threshold = max(INTENT_P99_FLOOR,
                                     float(np.percentile(np.fromiter(self._intent_history, dtype=float), 99)))
        self._intent_p99_timer = 0.0

    threshold = float(self._intent_threshold)
    span = max(INTENT_FULL - threshold, 0.2)
    target = float(np.clip((raw - threshold) / span, 0.0, 1.0))
    if target > 0.1:
      self._intent_active_time += dt
    else:
      self._intent_active_time = 0.0

    if self._intent_active_time > INTENT_MAX_ACTIVE_SEC:
      target *= max(0.0, 1.0 - (self._intent_active_time - INTENT_MAX_ACTIVE_SEC) / INTENT_TAPER_SEC)

    self._intent = _smooth(self._intent, target, dt, INTENT_ATTACK_TAU, INTENT_RELEASE_TAU)
    self._intent_raw = raw
    return _Intent(intent=float(self._intent), raw=raw, threshold=threshold,
                   departure=float(departure), weight=float(weight), source=source)

  @staticmethod
  def _local_fit(x: np.ndarray, y: np.ndarray, x0: float, half: float,
                 sigma: np.ndarray | None = None) -> tuple[float, float] | None:
    """Local quadratic y = c*u^2 + b*u + a, u = x - x0.  Returns (b, c)."""
    finite = np.isfinite(x) & np.isfinite(y)
    if finite.sum() < 5:
      return None
    x = x[finite]
    y = y[finite]
    if sigma is not None:
      sigma = np.asarray(sigma, dtype=float)[finite]
    mask = np.abs(x - x0) <= half
    if mask.sum() < 5:
      idx = np.argsort(np.abs(x - x0))[:min(7, x.size)]
      mask = np.zeros(x.size, dtype=bool)
      mask[idx] = True
    u = x[mask] - x0
    v = y[mask]
    if u.size < 5 or (u.max() - u.min()) < 2.0:
      return None
    w = None
    if sigma is not None:
      s = sigma[mask]
      if np.all(np.isfinite(s)):
        w = 1.0 / np.sqrt(s * s + POINT_STD ** 2)
    try:
      c, b, _ = np.polyfit(u, v, 2, w=w)
    except (np.linalg.LinAlgError, ValueError):
      return None
    if not (np.isfinite(b) and np.isfinite(c)):
      return None
    return float(b), float(c)

  def _rollout(self, v: float, kappa_meas: float, kappa_prev: float,
               kappa_base: float, delta: float, t_apply: float,
               T_p: float, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
    t_total = min(t_apply + T_p, MAX_HORIZON)
    n = max(int(np.ceil(t_total / dt)) + 1, 2)
    x = np.zeros(n)
    y = np.zeros(n)
    psi = np.zeros(n)
    kappa = np.zeros(n)
    kappa[0] = kappa_meas
    alpha = 1.0 - np.exp(-dt / self.tau)

    for i in range(1, n):
      t = i * dt
      t_cmd = t - self.dead_time
      if t_cmd < 0.0:
        cmd = kappa_meas
      elif t_cmd < t_apply:
        cmd = kappa_prev
      else:
        cmd = kappa_base + delta
      kappa[i] = kappa[i - 1] + (cmd - kappa[i - 1]) * alpha
      psi[i] = psi[i - 1] + kappa[i] * v * dt
      x[i] = x[i - 1] + v * np.cos(psi[i]) * dt
      y[i] = y[i - 1] + v * np.sin(psi[i]) * dt

    apply_idx = int(np.clip(round(t_apply / dt), 0, n - 1))
    return x, y, psi, kappa, apply_idx


if __name__ == "__main__":
  def fake_output(left_y, right_y, prob=0.9, std=0.1, curvature=0.0):
    x = np.array(ModelConstants.X_IDXS, dtype=np.float32)
    yc = 0.5 * curvature * x * x
    lines = np.zeros((1, 4, len(x), 2), dtype=np.float32)
    lines[0, 0, :, 0], lines[0, 0, :, 1] = yc - 1.85, 0.0
    lines[0, 1, :, 0], lines[0, 1, :, 1] = yc + left_y, 0.0
    lines[0, 2, :, 0], lines[0, 2, :, 1] = yc + right_y, 0.0
    probs = np.full((1, 8), prob, dtype=np.float32)
    stds = np.full((1, 4, len(x), 2), std, dtype=np.float32)
    return {'lane_lines': lines, 'lane_lines_prob': probs, 'lane_lines_stds': stds}

  lf = Laneful()
  state = LanefulState(v_ego=25.0, curvature=0.0)
  timing = LanefulTiming(frame_age=0.05, model_exec=0.05, lat_delay=0.25)
  out = lf.update(fake_output(-1.85, 1.85), state, timing, 0.0)
  assert abs(out.curvature_offset) < 1e-4, "centered -> no nudge"
  out = lf.update(fake_output(-1.0, 2.7), state, timing, 0.0)
  assert out.curvature_offset > 0.0, "lane center to the +y side -> nudge toward +y"
  out = lf.update(fake_output(-2.7, 1.0), state, timing, 0.0)
  assert out.curvature_offset < 0.0, "lane center to the -y side -> nudge toward -y"
  print("laneful self-check ok")
