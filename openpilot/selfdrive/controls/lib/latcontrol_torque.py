import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

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

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

CURVE_OUTWARD_DEADZONE = 0.0005  # 1/m (~radius 2000 m)
CURVE_OUTWARD_V_BP = [11.1, 19.4, 25.0, 33.3]   # m/s
CURVE_OUTWARD_FRAC_V = [0.0, 0.09, 0.155, 0.19]


def apply_curve_outward_bias(desired_curvature: float, v_ego: float) -> float:
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if frac <= 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  return desired_curvature * (1.0 - frac)

LANE_CENTER_MIN_SPEED = 8.0        # m/s (~30 km/h): below this the lines are too close / too curved to trust
LANE_CENTER_MIN_LINE_PROB = 0.5    # the better of the two near lane lines must be at least this confident
LANE_CENTER_MIN_OTHER_PROB = 0.3   # ...and the weaker one at least this: in tight curves the OUTER line's probability
                                   # drops to 0.35-0.5 (route 4d segs 9/11) exactly when the correction is needed
LANE_CENTER_WIDTH_RANGE = (2.6, 4.6)  # m; outside this the pair is not the ego lane
LANE_CENTER_FIT_RANGE = 12.0       # m ahead used to fit the lane centre (offset + heading + curvature)
LANE_CENTER_DEADBAND = 0.03        # m; no position correction inside this
LANE_CENTER_KP = 0.35              # m/s^2 of lateral accel per metre of offset
LANE_CENTER_KD = 1.20              # m/s^2 per m/s of lateral speed toward/away from the centre (damping, ~critical)
LANE_CENTER_KI = 0.05              # m/s^2 per metre per second: slowly takes over what the model keeps pulling
LANE_CENTER_I_LIMIT = 0.25         # m/s^2; cap on the integrated part
LANE_CENTER_MAX_LAT_ACCEL = 0.6    # m/s^2; cap on the total correction as felt by the driver
LANE_CENTER_MAX_CURV = 3.5e-3      # 1/m; cap on the total correction (radius ~290 m)
LANE_CENTER_RATE = 3.0e-3          # 1/m per second; how fast the correction may change
LANE_CENTER_FILTER_TAU = 0.3       # s; low-pass on the measured offset and heading


class LaneCentering:
  def __init__(self, dt: float):
    self.dt = dt
    self.offset_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.heading_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.integral = 0.0
    self.correction = 0.0
    self.offset = 0.0
    self.heading = 0.0
    self.valid = False

  def reset(self):
    self.offset_filter.x = 0.0
    self.heading_filter.x = 0.0
    self.integral = 0.0
    self.correction = 0.0
    self.valid = False

  def measure(self, model_v2) -> tuple[float, float] | None:
    """(offset, heading) of the car relative to the lane centre: offset positive = car LEFT of centre, heading
    positive = car pointing LEFT of the lane direction. None when the lines are unreliable."""
    if model_v2 is None:
      return None
    lines = model_v2.laneLines
    probs = model_v2.laneLineProbs
    if len(lines) < 4 or len(probs) < 4 or len(lines[1].y) < 4 or len(lines[2].y) < 4:
      return None
    if max(probs[1], probs[2]) < LANE_CENTER_MIN_LINE_PROB or min(probs[1], probs[2]) < LANE_CENTER_MIN_OTHER_PROB:
      return None
    yl, yr = lines[1].y[0], lines[2].y[0]
    width = yr - yl
    if not (LANE_CENTER_WIDTH_RANGE[0] <= width <= LANE_CENTER_WIDTH_RANGE[1]):
      return None
    x = np.asarray(lines[1].x)
    n = int(np.sum(x <= LANE_CENTER_FIT_RANGE))
    if n < 4:
      return None
    centre = (np.asarray(lines[1].y)[:n] + np.asarray(lines[2].y)[:n]) / 2.0  # model y: positive = right (capnp lists do not slice)
    # centre(x) ~ c0 + c1 x + c2 x^2: c0 is the centre's lateral position (right of the car), c1 its slope. A lane
    # that runs to the right ahead (c1 > 0) means the car is pointing LEFT of it.
    c2, c1, c0 = np.polyfit(x[:n], centre, 2)
    return float(c0), float(math.atan(c1))

  def update(self, model_v2, v_ego: float, active: bool, steering_pressed: bool) -> float:
    """Curvature to ADD to the desired curvature (positive = right)."""
    meas = self.measure(model_v2) if active and not steering_pressed and v_ego > LANE_CENTER_MIN_SPEED else None
    if meas is None:
      self.reset()
      return 0.0

    self.valid = True
    self.offset = self.offset_filter.update(meas[0])
    self.heading = self.heading_filter.update(meas[1])
    error = self.offset - float(np.clip(self.offset, -LANE_CENTER_DEADBAND, LANE_CENTER_DEADBAND))  # deadband
    lateral_speed = v_ego * math.sin(self.heading)  # positive = drifting LEFT

    self.integral = float(np.clip(self.integral + LANE_CENTER_KI * error * self.dt, -LANE_CENTER_I_LIMIT, LANE_CENTER_I_LIMIT))
    lat_accel = LANE_CENTER_KP * error + LANE_CENTER_KD * lateral_speed + self.integral
    lat_accel = float(np.clip(lat_accel, -LANE_CENTER_MAX_LAT_ACCEL, LANE_CENTER_MAX_LAT_ACCEL))
    target = float(np.clip(lat_accel / max(v_ego, LANE_CENTER_MIN_SPEED) ** 2, -LANE_CENTER_MAX_CURV, LANE_CENTER_MAX_CURV))
    step = LANE_CENTER_RATE * self.dt
    self.correction = float(np.clip(target, self.correction - step, self.correction + step))
    return self.correction


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)
    self.lane_centering = LaneCentering(dt)

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
    desired_curvature = apply_curve_outward_bias(desired_curvature, CS.vEgo)
    desired_curvature += self.lane_centering.update(self.extension.model_v2, CS.vEgo, active, CS.steeringPressed)
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

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

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
