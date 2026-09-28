"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Torque lateral controller v2.
Low speed handling, curvature-based request buffering, and vehicle-specific refinement
adapted from StarPilot (https://github.com/firestar5683/StarPilot).
"""

import math
from collections import deque

import numpy as np

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.6
KI = 0.3
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

# Error response boost below 20 m/s that falls off with speed; keeps late-lateral
# corrections from being applied full strength before the car can respond.
LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [12, 10.5, 8, 5]

MAX_LAT_JERK_UP = 2.5  # m/s^3
LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.22
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 2

# Steering-integrator decay applied when the driver releases the wheel mid-correction,
# so the controller does not hold a stale bias after hand-over.
STEER_RELEASE_I_DECAY = 0.8

# Carnival-specific refinement (see starpilot latcontrol_vehicle_tunes). Values are
# sigmoid-gated reductions of friction/feedforward/output near the lane center and
# during curve-exit unwinds, where this platform tends to overshoot.
KIA_CARNIVAL_CENTER_TAPER_MAX = 0.20
KIA_CARNIVAL_CENTER_TAPER_LAT = 0.20
KIA_CARNIVAL_CENTER_TAPER_LAT_WIDTH = 0.055
KIA_CARNIVAL_CENTER_TAPER_SPEED = 3.5
KIA_CARNIVAL_CENTER_TAPER_SPEED_WIDTH = 1.8
KIA_CARNIVAL_CENTER_TAPER_SPEED_MAX = 14.5
KIA_CARNIVAL_CENTER_TAPER_SPEED_MAX_WIDTH = 2.0
KIA_CARNIVAL_FRICTION_THRESHOLD_GAIN = 0.24
KIA_CARNIVAL_FRICTION_CENTER_FADE_MAX = 0.34
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_MAX = 0.34
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED = 15.0
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED_WIDTH = 2.0
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED_CUTOFF = 23.0
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED_CUTOFF_WIDTH = 2.0
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_LAT = 0.35
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_LAT_WIDTH = 0.18
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_JERK = 0.65
KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_JERK_WIDTH = 0.25
KIA_CARNIVAL_UNWIND_FF_REDUCTION_MAX = 0.45
KIA_CARNIVAL_UNWIND_FF_SPEED = 9.0
KIA_CARNIVAL_UNWIND_FF_SPEED_WIDTH = 2.0
KIA_CARNIVAL_UNWIND_FF_SPEED_CUTOFF = 23.0
KIA_CARNIVAL_UNWIND_FF_SPEED_CUTOFF_WIDTH = 2.0
KIA_CARNIVAL_UNWIND_FF_OVERSHOOT = 0.08
KIA_CARNIVAL_UNWIND_FF_OVERSHOOT_WIDTH = 0.06
KIA_CARNIVAL_UNWIND_FF_JERK = 0.45
KIA_CARNIVAL_UNWIND_FF_JERK_WIDTH = 0.20
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_MAX = 0.35
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED = 8.0
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED_WIDTH = 2.0
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED_CUTOFF = 16.0
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED_CUTOFF_WIDTH = 2.5
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_OVERSHOOT = 0.25
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_OVERSHOOT_WIDTH = 0.15
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_JERK = 0.45
KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_JERK_WIDTH = 0.20


def _sigmoid(x: float) -> float:
  if x >= 0.0:
    z = math.exp(-x)
    return 1.0 / (1.0 + z)

  z = math.exp(x)
  return z / (1.0 + z)


def _kia_carnival_center_weights(desired_lateral_accel: float, v_ego: float) -> tuple[float, float]:
  speed_onset = _sigmoid((v_ego - KIA_CARNIVAL_CENTER_TAPER_SPEED) / KIA_CARNIVAL_CENTER_TAPER_SPEED_WIDTH)
  speed_cutoff = _sigmoid((KIA_CARNIVAL_CENTER_TAPER_SPEED_MAX - v_ego) / KIA_CARNIVAL_CENTER_TAPER_SPEED_MAX_WIDTH)
  speed_weight = speed_onset * speed_cutoff
  center_weight = _sigmoid((KIA_CARNIVAL_CENTER_TAPER_LAT - abs(desired_lateral_accel)) /
                           KIA_CARNIVAL_CENTER_TAPER_LAT_WIDTH)
  return speed_weight, center_weight


def get_kia_carnival_center_taper_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight, center_weight = _kia_carnival_center_weights(desired_lateral_accel, v_ego)
  return 1.0 - KIA_CARNIVAL_CENTER_TAPER_MAX * speed_weight * center_weight


def get_kia_carnival_friction_threshold(v_ego: float, desired_lateral_accel: float) -> float:
  speed_weight, center_weight = _kia_carnival_center_weights(desired_lateral_accel, v_ego)
  return FRICTION_THRESHOLD * (1.0 + KIA_CARNIVAL_FRICTION_THRESHOLD_GAIN * speed_weight * center_weight)


def get_kia_carnival_friction_center_fade_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight, center_weight = _kia_carnival_center_weights(desired_lateral_accel, v_ego)
  return 1.0 - KIA_CARNIVAL_FRICTION_CENTER_FADE_MAX * speed_weight * center_weight


def get_kia_carnival_friction_jerk_deadzone(v_ego: float, desired_lateral_accel: float,
                                            desired_lateral_jerk: float) -> float:
  """Reduce abrupt friction reversals during mid-speed curve exits only."""
  speed_weight = _sigmoid((v_ego - KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED) /
                          KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED_WIDTH)
  speed_cutoff = _sigmoid((KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED_CUTOFF - v_ego) /
                          KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_SPEED_CUTOFF_WIDTH)
  center_weight = _sigmoid((KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_LAT - abs(desired_lateral_accel)) /
                           KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_LAT_WIDTH)
  jerk_weight = _sigmoid((abs(desired_lateral_jerk) - KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_JERK) /
                         KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_JERK_WIDTH)
  return KIA_CARNIVAL_UNWIND_FRICTION_JERK_DEADZONE_MAX * speed_weight * speed_cutoff * center_weight * jerk_weight


def get_kia_carnival_unwind_ff_scale(setpoint: float, measured_lateral_accel: float,
                                     desired_lateral_jerk: float, v_ego: float) -> float:
  """Remove stale turn feedforward when the measured response carries through an unwind."""
  if setpoint * desired_lateral_jerk >= 0.0:
    return 1.0

  overshoot = max(abs(measured_lateral_accel) - abs(setpoint), 0.0)
  if overshoot <= 0.0:
    return 1.0

  speed_weight = (_sigmoid((v_ego - KIA_CARNIVAL_UNWIND_FF_SPEED) /
                           KIA_CARNIVAL_UNWIND_FF_SPEED_WIDTH) *
                  _sigmoid((KIA_CARNIVAL_UNWIND_FF_SPEED_CUTOFF - v_ego) /
                           KIA_CARNIVAL_UNWIND_FF_SPEED_CUTOFF_WIDTH))
  overshoot_weight = _sigmoid((overshoot - KIA_CARNIVAL_UNWIND_FF_OVERSHOOT) /
                              KIA_CARNIVAL_UNWIND_FF_OVERSHOOT_WIDTH)
  jerk_weight = _sigmoid((abs(desired_lateral_jerk) - KIA_CARNIVAL_UNWIND_FF_JERK) /
                         KIA_CARNIVAL_UNWIND_FF_JERK_WIDTH)
  return 1.0 - KIA_CARNIVAL_UNWIND_FF_REDUCTION_MAX * speed_weight * overshoot_weight * jerk_weight


def get_kia_carnival_unwind_output_scale(setpoint: float, measured_lateral_accel: float,
                                         desired_lateral_jerk: float, v_ego: float) -> float:
  if setpoint * desired_lateral_jerk >= 0.0 or setpoint * measured_lateral_accel <= 0.0:
    return 1.0

  overshoot = max(abs(measured_lateral_accel) - abs(setpoint), 0.0)
  if overshoot <= 0.0:
    return 1.0

  speed_weight = (_sigmoid((v_ego - KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED) /
                           KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED_WIDTH) *
                  _sigmoid((KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED_CUTOFF - v_ego) /
                           KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_SPEED_CUTOFF_WIDTH))
  overshoot_weight = _sigmoid((overshoot - KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_OVERSHOOT) /
                              KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_OVERSHOOT_WIDTH)
  jerk_weight = _sigmoid((abs(desired_lateral_jerk) - KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_JERK) /
                         KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_JERK_WIDTH)
  return 1.0 - KIA_CARNIVAL_UNWIND_OUTPUT_DAMPING_MAX * speed_weight * overshoot_weight * jerk_weight


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.CP = CP
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    # Stores requested CURVATURE, scaled by the current v^2 on read. Storing lateral
    # accel directly makes the delayed request lag the measurement whenever speed is
    # changing (both scale with v^2 but the buffered value used the old speed), which
    # at creep-speed gains reads as a phantom unwind error during every pull-away.
    self.curvature_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * (MAX_LAT_JERK_UP - 0.5)), self.dt)
    self.prev_steering_pressed = False

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2

    if not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
      # Keep the request buffer and rate state primed with the live command (which tracks
      # the measured curvature while inactive) instead of zeroing them. Re-engaging with a
      # wound wheel against a zeroed buffer puts the setpoint ~lat_delay behind the
      # measurement, and the low-speed gains turn that lag into a hard unwind shove.
      self.curvature_request_buffer.append(desired_curvature)
      self.previous_measurement = measurement
      self.measurement_rate_filter.x = 0.0
      self.jerk_filter.x = 0.0
    else:
      if self.prev_steering_pressed and not CS.steeringPressed:
        self.pid.i *= STEER_RELEASE_I_DECAY

      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.curvature_request_buffer[-delay_frames] * CS.vEgo ** 2
      self.curvature_request_buffer.append(desired_curvature)

      raw_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / max(lat_delay, self.dt)
      raw_lateral_jerk = np.clip(raw_lateral_jerk, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
      desired_lateral_jerk = np.clip(self.jerk_filter.update(raw_lateral_jerk), -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)

      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      setpoint = expected_lateral_accel + desired_lateral_jerk * lat_delay

      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      measurement_rate = np.clip(measurement_rate, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
      self.previous_measurement = measurement

      low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_SPEED)) ** 2
      current_kp = np.interp(CS.vEgo, self.pid._k_p[0], self.pid._k_p[1])
      error = setpoint - measurement
      error_with_lsf = error * (1 + low_speed_factor / max(current_kp, 1e-3))

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error_with_lsf)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset

      carnival_active = self.CP.carFingerprint in ("KIA_CARNIVAL_HEV_2026",)
      friction_threshold = get_kia_carnival_friction_threshold(CS.vEgo, setpoint) if carnival_active else FRICTION_THRESHOLD

      friction_jerk_deadzone = get_kia_carnival_friction_jerk_deadzone(CS.vEgo, setpoint, desired_lateral_jerk) if carnival_active else 0.0
      friction_jerk = math.copysign(max(abs(desired_lateral_jerk) - friction_jerk_deadzone, 0.0), desired_lateral_jerk)
      ff += get_friction(error_with_lsf + JERK_GAIN * friction_jerk, lateral_accel_deadzone, friction_threshold, self.torque_params)
      if carnival_active:
        ff *= get_kia_carnival_friction_center_fade_scale(setpoint, CS.vEgo)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                        -measurement_rate,
                                        feedforward=ff,
                                        speed=CS.vEgo,
                                        freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      if carnival_active:
        ff *= get_kia_carnival_unwind_ff_scale(setpoint, measurement, desired_lateral_jerk, CS.vEgo)
        output_torque *= get_kia_carnival_center_taper_scale(setpoint, CS.vEgo)
        output_torque *= get_kia_carnival_unwind_output_scale(setpoint, measurement, desired_lateral_jerk, CS.vEgo)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
                                                     desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    self.prev_steering_pressed = CS.steeringPressed

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
