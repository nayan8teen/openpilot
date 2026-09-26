from typing import cast

import numpy as np

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.lane_policy import (
  LANE_LOCK_ARM_LINE_PROB,
  LANE_LOCK_ARM_TIME,
  LANE_LOCK_CORRECTION_ENGAGE_STEP,
  LANE_LOCK_CORRECTION_RELEASE_STEP,
  LANE_LOCK_FIT_END,
  LANE_LOCK_FIT_START,
  LANE_LOCK_MAX_CENTER_CORRECTION,
  LANE_LOCK_ONE_LINE_HOLD_TIME,
  LANE_LOCK_TURN_CURVATURE,
  LanePolicy,
  get_inner_lane_line_probs,
  get_lane_policy_enabled,
)


def make_model_output(left_prob: float = 0.99, right_prob: float = 0.99, lane_width: float = 3.6,
                      lane_width_end: float | None = None, lane_center: float = 0.0) -> dict[str, np.ndarray]:
  x = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
  lane_lines = np.zeros((1, 4, len(x), 2), dtype=np.float64)
  target_width = lane_width if lane_width_end is None else lane_width_end
  width_progress = np.clip((x - LANE_LOCK_FIT_START) /
                           (LANE_LOCK_FIT_END - LANE_LOCK_FIT_START), 0.0, 1.0)
  widths = lane_width + (target_width - lane_width) * width_progress
  # openpilot lateral coordinates are left-negative and right-positive.
  lane_lines[0, 1, :, 0] = lane_center - widths / 2.0
  lane_lines[0, 2, :, 0] = lane_center + widths / 2.0
  lane_line_probs = np.zeros((1, 8), dtype=np.float64)
  lane_line_probs[0, 3] = left_prob
  lane_line_probs[0, 5] = right_prob
  plan = np.zeros((1, len(x), ModelConstants.PLAN_WIDTH), dtype=np.float64)
  plan[0, :, 0] = x
  return {'lane_lines': lane_lines, 'lane_lines_prob': lane_line_probs,
          'desire_state': np.zeros((1, ModelConstants.DESIRE_LEN), dtype=np.float64), 'plan': plan}


class TestLanePolicy(OpenpilotTestCase):
  def make_policy(self) -> LanePolicy:
    return LanePolicy()

  def apply_for(self, policy: LanePolicy, output: dict[str, np.ndarray], seconds: float,
                e2e_curvature: float = 0.0, blinkers_active: bool = False) -> float:
    result = e2e_curvature
    for _ in range(max(1, int(np.ceil(seconds / policy.dt)))):
      result = policy.update(output, e2e_curvature, 20.0,
                             blinkers_active=blinkers_active, enabled=True)
    return result

  def arm_lane_policy(self, output: dict[str, np.ndarray] | None = None) -> tuple[LanePolicy, dict[str, np.ndarray]]:
    policy = self.make_policy()
    output = make_model_output() if output is None else output
    self.apply_for(policy, output, LANE_LOCK_ARM_TIME + policy.dt)
    assert policy.is_active
    assert policy._width_valid
    return policy, output

  def test_disabled_mode_returns_exact_e2e_target(self):
    policy = self.make_policy()
    assert policy.update(make_model_output(), 0.0123, 20.0, enabled=False) == 0.0123
    assert not policy.is_active

  def test_selector_decodes_params_bytes_and_defaults_on_when_unset(self):
    class FakeParams:
      def __init__(self, value):
        self.value = value

      def get(self, key):
        assert key == "LanePolicyEnabled"
        return self.value

    # Unset defaults to disabled: the policy is opt-in so process replay and
    # the model-release pipeline keep producing raw E2E curvature.
    assert not get_lane_policy_enabled(cast(Params, FakeParams(None)))
    assert get_lane_policy_enabled(cast(Params, FakeParams(b"1")))
    assert not get_lane_policy_enabled(cast(Params, FakeParams(b"0")))
    assert not get_lane_policy_enabled(cast(Params, FakeParams(False)))

  def test_raw_probability_indices(self):
    output = make_model_output(0.97, 0.96)
    output['lane_lines_prob'][0, 1] = 0.01
    assert get_inner_lane_line_probs(output) == (0.97, 0.96)

  def test_arms_only_after_clean_two_line_timer(self):
    policy = self.make_policy()
    output = make_model_output()
    self.apply_for(policy, output, LANE_LOCK_ARM_TIME - policy.dt)
    assert not policy.is_active
    self.apply_for(policy, output, 2.0 * policy.dt)
    assert policy.is_active

  def test_full_center_correction_keeps_e2e_curve_feedforward(self):
    policy, output = self.arm_lane_policy(make_model_output(lane_center=0.45))
    e2e = 0.0010
    curvature = policy.update(output, e2e, 20.0, enabled=True)
    assert policy.is_active
    assert curvature > e2e
    assert curvature - e2e <= LANE_LOCK_MAX_CENTER_CORRECTION

    # Entry ramps in rather than causing a one-frame steering step: while the
    # target is growing, each frame moves the correction by at most the
    # engage step.
    previous = policy._center_correction
    policy.update(output, e2e, 20.0, enabled=True)
    assert abs(policy._center_correction - previous) <= LANE_LOCK_CORRECTION_ENGAGE_STEP + 1e-12

    # A smaller target releases faster than it engages, so a curve/lane-change
    # correction does not persist through the midpoint.
    previous = policy._center_correction
    policy.update(make_model_output(lane_center=0.0), e2e, 20.0, enabled=True)
    assert abs(policy._center_correction - previous) <= LANE_LOCK_CORRECTION_RELEASE_STEP + 1e-12

  def test_turn_direction_change_releases_opposed_correction(self):
    policy, _ = self.arm_lane_policy(make_model_output())
    negative_turn = -2.0 * LANE_LOCK_TURN_CURVATURE
    self.apply_for(policy, make_model_output(), 0.1, negative_turn)

    # A steady curve may legitimately need a lane correction opposite E2E;
    # full centering must remain available until E2E changes turn direction.
    output = make_model_output(lane_center=-0.45)
    for _ in range(6):
      policy.update(output, negative_turn, 20.0, enabled=True)
    before = policy._center_correction
    assert before < 0.0

    # When the meaningful E2E turn direction flips, release the stale,
    # opposing correction at the faster release rate rather than carrying it
    # into the new curve.
    positive_turn = 2.0 * LANE_LOCK_TURN_CURVATURE
    policy.update(output, positive_turn, 20.0, enabled=True)
    after = policy._center_correction
    assert after > before
    assert abs(after - before) <= LANE_LOCK_CORRECTION_RELEASE_STEP + 1e-12

  def test_no_plan_gate_for_clean_lanes(self):
    policy, output = self.arm_lane_policy(make_model_output(lane_center=0.35))
    output['plan'][:] = np.nan
    curvature = policy.update(output, 0.0, 20.0, enabled=True)
    assert curvature > 0.0

  def test_hysteresis_retains_full_center_above_exit_threshold(self):
    policy, _ = self.arm_lane_policy()
    reduced_confidence = make_model_output(left_prob=0.80, right_prob=0.80, lane_center=0.25)
    curvature = policy.update(reduced_confidence, 0.001, 20.0, enabled=True)
    assert policy.is_active
    assert not policy._one_line_hold
    assert curvature > 0.001

  def test_below_exit_confidence_releases_to_exact_e2e(self):
    policy, _ = self.arm_lane_policy()
    low_confidence = make_model_output(left_prob=0.65, right_prob=0.65)
    assert policy.update(low_confidence, -0.0012, 20.0, enabled=True) == -0.0012
    assert not policy.is_active
    # A few bad frames keep the learned width so re-arming is fast.
    assert policy._width_valid

  def test_one_line_hold_uses_learned_width(self):
    policy, _ = self.arm_lane_policy()
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.35)
    curvature = self.apply_for(policy, one_line, 0.50)
    assert policy.is_active
    assert policy._one_line_hold
    assert curvature > 0.0

  def test_one_line_hold_expires_to_exact_e2e(self):
    policy, _ = self.arm_lane_policy()
    one_line = make_model_output(left_prob=0.99, right_prob=0.10, lane_center=0.35)
    e2e = -0.0012
    result = self.apply_for(policy, one_line, LANE_LOCK_ONE_LINE_HOLD_TIME + 2.0 * policy.dt, e2e)
    assert result == e2e
    assert not policy.is_active

  def test_blinker_releases_lane_lock(self):
    policy, output = self.arm_lane_policy()
    assert policy.update(output, 0.0123, 20.0, blinkers_active=True, enabled=True) == 0.0123
    assert not policy.is_active

  def test_lane_change_intent_releases_lane_lock(self):
    policy, _ = self.arm_lane_policy()
    output = make_model_output()
    output['desire_state'][0, log.Desire.laneChangeLeft] = 0.2
    assert policy.update(output, 0.0123, 20.0, enabled=True) == 0.0123
    assert not policy.is_active

  def test_robust_geometry_accepts_normal_taper_and_rejects_extreme_taper(self):
    policy, _ = self.arm_lane_policy(make_model_output(lane_width=3.6, lane_width_end=4.1))
    assert policy.is_active

    policy = self.make_policy()
    output = make_model_output(lane_width=3.6, lane_width_end=5.0)
    self.apply_for(policy, output, LANE_LOCK_ARM_TIME + policy.dt)
    assert not policy.is_active

  def test_low_speed_hands_back_e2e_but_keeps_width_memory(self):
    policy, _ = self.arm_lane_policy()
    # Below the minimum speed the policy must not command a correction.
    assert policy.update(make_model_output(lane_center=0.4), 0.0, 1.0, enabled=True) == 0.0
    assert not policy.is_active
    assert policy._width_valid
    # And it must re-arm cleanly once speed comes back.
    curvature = self.apply_for(policy, make_model_output(lane_center=0.4), LANE_LOCK_ARM_TIME + policy.dt)
    assert policy.is_active
    assert curvature > 0.0

  def test_arm_threshold_uses_strong_confidence(self):
    policy = self.make_policy()
    # Just below the arm threshold never arms.
    weak = make_model_output(left_prob=LANE_LOCK_ARM_LINE_PROB - 0.01,
                             right_prob=LANE_LOCK_ARM_LINE_PROB - 0.01)
    self.apply_for(policy, weak, 3 * LANE_LOCK_ARM_TIME)
    assert not policy.is_active


if __name__ == "__main__":
  import unittest
  unittest.main()
