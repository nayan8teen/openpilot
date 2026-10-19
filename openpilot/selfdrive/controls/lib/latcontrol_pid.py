import math
import queue
import threading
from collections import deque

import numpy as np

from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.pid import PIDController, ReferenceModel
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.torque_to_pid import FRICTION_MULT, FRICTION_STEEPNESS, get_pid_schedule

# Gains move by percentages and rates, never by an absolute step, so these transfer across cars.
ADAPTIVE_CONFIG = {
  "eta_p": 3e-3,
  "eta_i": 3e-4,
  "eta_d": 3e-4,
  "k_aw": 1.0,
  "adapt_period": 0.1,
}

ERROR_FILTER_HZ = 10.0
ERROR_RATE_TAU = 0.0354
RATE_DEADZONE = 0.15           # deg/s
FRICTION_DEADZONE = 0.3        # deg
ADAPT_MAX_ERROR = 3.0          # deg, reject outliers rather than learning from them
FF_LEAD_MAX_DEG = 5.0
# The reference model must never be slower than the loop it supervises, or it biases every update
# in one direction. It runs on the delay-compensated command and is kept comfortably fast.
REF_MODEL_MIN_WN = 6.0
REF_MODEL_MAX_WN = 12.0
REF_MODEL_MAX_RATE = 250.0     # deg/s

PERSIST_KEY = "LivePidParameters"
PERSIST_VERSION = 2
PERSIST_PERIOD_FRAMES = 6000


class _AsyncParamWriter:
  """Writes params off the control thread.

  The previous implementation called params.put() inline every 60 s, which is blocking file I/O on
  the realtime thread.
  """

  def __init__(self, key):
    self._key = key
    self._queue = queue.Queue(maxsize=1)
    self._thread = threading.Thread(target=self._run, name="pid_param_writer", daemon=True)
    self._thread.start()

  def submit(self, value):
    try:
      self._queue.put_nowait(value)
    except queue.Full:
      # keep only the newest snapshot
      try:
        self._queue.get_nowait()
      except queue.Empty:
        pass
      try:
        self._queue.put_nowait(value)
      except queue.Full:
        pass

  def _run(self):
    params = Params()
    while True:
      value = self._queue.get()
      if value is None:
        return
      try:
        params.put(self._key, value)
      except Exception:
        pass


class LatControlPID(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)

    schedule = get_pid_schedule(CP)
    self.schedule_native = schedule.native
    self.pid = PIDController((schedule.kp_bp, schedule.kp_v),
                             (schedule.ki_bp, schedule.ki_v),
                             (schedule.kd_bp, schedule.kd_v),
                             pos_limit=self.steer_max, neg_limit=-self.steer_max,
                             rate=1.0 / dt, mrac_config=ADAPTIVE_CONFIG)
    self.ff_factor = schedule.kf
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

    # friction feedforward has its own gain: sharing kf meant changing the curvature feedforward
    # silently rescaled the friction term as well
    self.friction_ff_gain = self.ff_factor * FRICTION_MULT

    self.error_filter = FirstOrderFilter(0.0, 1.0 / (2 * math.pi * ERROR_FILTER_HZ), dt)
    self.error_alpha = dt / (ERROR_RATE_TAU + dt)
    self.last_error = 0.0
    self.error_rate_filter = 0.0

    # phase advance of the feedforward command, to cover the actuator transport delay
    self.ff_lead_time = max(float(CP.steerActuatorDelay), 0.0)
    self.angle_rate_alpha = dt / (0.1 + dt)
    self.last_angle_des: float | None = None
    self.angle_des_rate = 0.0

    delay = max(float(CP.steerActuatorDelay), 0.0)
    self.ref_delay = delay + 0.04
    self.ref_model_omega = float(np.clip(1.0 / (0.5 * self.ref_delay), REF_MODEL_MIN_WN, REF_MODEL_MAX_WN))
    self.ref_model = ReferenceModel(dt, self.ref_model_omega, REF_MODEL_MAX_RATE)

    # delay line: the reference is the command the plant should have caught up with by now. Without
    # this the model lags the plant in steady state and drives the gains in one direction forever.
    self.ref_delay_frames = int(round(self.ref_delay / dt))
    self.angle_des_buffer = deque([0.0] * (self.ref_delay_frames + 1), maxlen=self.ref_delay_frames + 1)

    self.last_reference_error = 0.0
    self.adapt_ok = False
    self.frame = 0

    self._car_fingerprint = str(CP.carFingerprint)
    self._writer: _AsyncParamWriter | None = None
    self._restore_trims()

  def _restore_trims(self):
    try:
      data = Params().get(PERSIST_KEY)
    except Exception:
      return

    if not isinstance(data, dict) or data.get("version") != PERSIST_VERSION:
      return
    if data.get("car") != self._car_fingerprint:
      return

    try:
      self.pid.set_trims(float(data["theta_p"]), float(data["theta_i"]), float(data["theta_d"]))
    except (KeyError, TypeError, ValueError):
      pass

  def _persist_trims(self):
    if self._writer is None:
      try:
        self._writer = _AsyncParamWriter(PERSIST_KEY)
      except Exception:
        return
    theta_p, theta_i, theta_d = self.pid.trims
    self._writer.submit({"version": PERSIST_VERSION, "car": self._car_fingerprint,
                         "theta_p": theta_p, "theta_i": theta_i, "theta_d": theta_d})

  def debug_snapshot(self) -> dict:
    theta_p, theta_i, theta_d = self.pid.trims
    return {
      "theta_p": theta_p,
      "theta_i": theta_i,
      "theta_d": theta_d,
      "k_p": self.pid.k_p,
      "k_i": self.pid.k_i,
      "k_d": self.pid.k_d,
      "reference_error": self.last_reference_error,
      "adapt_ok": self.adapt_ok,
      "ref_model_omega": self.ref_model_omega,
      "native_tuning": self.schedule_native,
    }

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
    raw_error = angle_steers_des - CS.steeringAngleDeg
    error = self.error_filter.update(raw_error)

    pid_log.steeringAngleDesiredDeg = float(angle_steers_des)
    pid_log.angleError = float(error)

    if not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
      self.last_error = 0.0
      self.error_rate_filter = 0.0
      self.error_filter.x = 0.0
      self.last_angle_des = None
      self.angle_des_rate = 0.0
      self.last_reference_error = 0.0
      self.adapt_ok = False
      self.ref_model.reset(CS.steeringAngleDeg)
      self.angle_des_buffer.extend([float(CS.steeringAngleDeg)] * len(self.angle_des_buffer))

    else:
      raw_error_rate = (error - self.last_error) / self.dt
      self.last_error = error
      self.error_rate_filter = (1.0 - self.error_alpha) * self.error_rate_filter + self.error_alpha * raw_error_rate

      if self.last_angle_des is None:
        self.last_angle_des = angle_steers_des
      raw_angle_rate = (angle_steers_des - self.last_angle_des) / self.dt
      self.last_angle_des = angle_steers_des
      self.angle_des_rate = (1.0 - self.angle_rate_alpha) * self.angle_des_rate + self.angle_rate_alpha * raw_angle_rate

      # reference model gives the adaptation a real fixed point: epsilon -> 0 when the plant matches
      # the achievable response, instead of the constant drift of an error-only law
      self.angle_des_buffer.append(angle_steers_des)
      y_ref = self.ref_model.update(self.angle_des_buffer[0])
      reference_error = CS.steeringAngleDeg - y_ref
      self.last_reference_error = reference_error

      # learn only from a loop that is actually regulating: not overridden, not saturated, and not
      # chasing an outlier. pid.control is the previous frame's output, which is what we want here.
      self.adapt_ok = bool(not steer_limited_by_safety and not CS.steeringPressed
                           and not curvature_limited and CS.vEgo > 5.0
                           and abs(error) < ADAPT_MAX_ERROR
                           and abs(self.pid.control) < self.steer_max - 1e-3)
      self.pid.update_adaptive_gains(error, reference_error, self.error_rate_filter, enabled=self.adapt_ok)

      self.frame += 1
      if self.frame % PERSIST_PERIOD_FRAMES == 0:
        self._persist_trims()

      # feedforward, phase advanced by the actuator delay
      lead = float(np.clip(self.angle_des_rate * self.ff_lead_time, -FF_LEAD_MAX_DEG, FF_LEAD_MAX_DEG))
      ff = self.ff_factor * self.get_steer_feedforward(angle_steers_des_no_offset + lead, CS.vEgo)

      error_deadzone = error - float(np.clip(error, -FRICTION_DEADZONE, FRICTION_DEADZONE))
      ff += self.friction_ff_gain * float(np.tanh(error_deadzone * FRICTION_STEEPNESS))

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      rate_deadzone = RATE_DEADZONE
      error_rate_clean = self.error_rate_filter - float(np.clip(self.error_rate_filter, -rate_deadzone, rate_deadzone))

      output_torque = self.pid.update(error,
                                      error_rate=error_rate_clean,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    return output_torque, angle_steers_des, pid_log
