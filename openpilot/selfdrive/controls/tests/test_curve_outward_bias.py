import math
import unittest

from openpilot.selfdrive.controls.lib.latcontrol_torque import (apply_curve_outward_bias, CURVE_OUTWARD_DEADZONE,
                                                                CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V)


class TestCurveOutwardBias(unittest.TestCase):
  def test_straights_and_small_corrections_untouched(self):
    for k in (0.0, 0.001, CURVE_OUTWARD_DEADZONE, -CURVE_OUTWARD_DEADZONE, 0.0025):
      self.assertEqual(apply_curve_outward_bias(k, CURVE_OUTWARD_V_BP[1]), k)

  def test_curves_relaxed_outward_and_sign_preserved(self):
    for k in (0.0067, 0.012, -0.0067, -0.012):
      out = apply_curve_outward_bias(k, CURVE_OUTWARD_V_BP[1])
      self.assertLess(abs(out), abs(k))            # commands a wider radius
      self.assertEqual(math.copysign(1, out), math.copysign(1, k))  # same turn direction

  def test_gentle_high_speed_curves_are_biased(self):
    k = 0.0022  # ~85 km/h curve; must be reduced (used to fall below the deadzone and be untouched)
    self.assertLess(abs(apply_curve_outward_bias(k, CURVE_OUTWARD_V_BP[1])), abs(k))

  def test_reduction_is_a_uniform_fraction_above_the_deadzone(self):
    for k in (0.006, 0.010, 0.02):
      self.assertAlmostEqual(apply_curve_outward_bias(k, CURVE_OUTWARD_V_BP[1]), k * (1.0 - CURVE_OUTWARD_FRAC_V[1]), places=9)

  def test_no_bias_at_low_speed(self):
    # tight low-speed curves must keep their full turn-in (they run wide, not inside)
    for k in (0.006, 0.02, -0.02):
      self.assertEqual(apply_curve_outward_bias(k, CURVE_OUTWARD_V_BP[0]), k)
      self.assertEqual(apply_curve_outward_bias(k, 8.0), k)

  def test_bias_is_bounded(self):
    # never flips sign or over-relaxes: reduction is at most the scheduled fraction of the curvature
    for k in (0.05, -0.05):
      self.assertLessEqual(abs(k - apply_curve_outward_bias(k, CURVE_OUTWARD_V_BP[1])), CURVE_OUTWARD_FRAC_V[-1] * abs(k) + 1e-9)


if __name__ == "__main__":
  unittest.main()
