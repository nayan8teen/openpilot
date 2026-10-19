import math
from dataclasses import dataclass, field

from opendbc.car.interfaces import get_torque_params

# These are idealized parameters derived from a closed-loop adam optimizer running on ~12,500 hours of control samples
# Higher proportional gain is needed at low speed to overcome tire slip/scrubbing and sheer weight of vehicle to overcome.
INTERP_SPEEDS = [1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0, 30.0]
KP_TORQUE_ARRAY = [5.3309, 5.3, 5.2974, 5.1916, 5.1552, 1.9585, 0.9083, 0.273, 0.2]
KI_TORQUE_ARRAY = [0.4127, 0.3902, 0.3882, 0.3826, 0.3335, 0.2241, 0.1992, 0.1832, 0.1695]

# Derivative as a time constant rather than an independent schedule. Kd = TD_S * Kp keeps the
# derivative action consistent with the proportional gain at every speed; separate Kd breakpoints
# produced derivative times that swung from 40 ms to over a second across the speed range.
TD_S = 0.06

FRICTION_MULT = 1445.91
FRICTION_STEEPNESS = 0.1945


@dataclass
class PidSchedule:
  kp_bp: list[float] = field(default_factory=list)
  kp_v: list[float] = field(default_factory=list)
  ki_bp: list[float] = field(default_factory=list)
  ki_v: list[float] = field(default_factory=list)
  kd_bp: list[float] = field(default_factory=list)
  kd_v: list[float] = field(default_factory=list)
  kf: float = 0.0
  # True when the car ships its own validated gains, False when they were derived from the torque model
  native: bool = False


def get_pid_from_torque(CP):
  torque_cache = get_torque_params()
  lat_accel_factor = 2.0
  friction = 0.1

  if CP.carFingerprint in torque_cache:
    lat_accel_factor = torque_cache[CP.carFingerprint].get('LAT_ACCEL_FACTOR')
    friction = torque_cache[CP.carFingerprint].get('FRICTION')

  # guard against cars with no usable geometry rather than emitting inf gains
  steer_ratio = max(float(CP.steerRatio), 1e-3)
  wheelbase = max(float(CP.wheelbase), 1e-3)
  lat_accel_factor = max(float(lat_accel_factor), 1e-3)

  kpV = []
  kiV = []

  for i, v in enumerate(INTERP_SPEEDS):
    denominator = wheelbase * 180.0 * steer_ratio
    numerator = KP_TORQUE_ARRAY[i] * v**2 * math.pi * lat_accel_factor
    kpV.append(float(numerator / denominator))

    # the integral is frozen below 5 m/s in the controller, so no velocity clip is needed here
    ki_numerator = KI_TORQUE_ARRAY[i] * v**2 * math.pi * lat_accel_factor
    kiV.append(float(ki_numerator / denominator))

  kf = float(friction * 0.001)
  return list(INTERP_SPEEDS), kpV, list(INTERP_SPEEDS), kiV, kf


def get_pid_schedule(CP) -> PidSchedule:
  """Per-car gain schedule.

  Cars that ship their own PID tuning keep it untouched; everything else falls back to the
  torque-model conversion. The derivative schedule is always derived from the proportional one so
  the two never disagree about what speed they are being interpolated at.
  """
  if CP.lateralTuning.which() == 'pid' and len(CP.lateralTuning.pid.kpV) > 0:
    tune = CP.lateralTuning.pid
    kp_bp = list(tune.kpBP)
    kp_v = list(tune.kpV)
    ki_bp = list(tune.kiBP)
    ki_v = list(tune.kiV)
    native = True
    kf = float(tune.kf)
    if len(ki_bp) != len(ki_v) or len(ki_bp) == 0:
      ki_bp, ki_v = list(kp_bp), [0.3 * k for k in kp_v]
    if len(kp_bp) != len(kp_v) or len(kp_bp) == 0:
      raise ValueError(f"{CP.carFingerprint}: malformed PID tuning, kpBP={kp_bp} kpV={kp_v}")
  else:
    kp_bp, kp_v, ki_bp, ki_v, kf = get_pid_from_torque(CP)
    native = False

  kd_bp = list(kp_bp)
  kd_v = [TD_S * k for k in kp_v]

  return PidSchedule(kp_bp=kp_bp, kp_v=kp_v, ki_bp=ki_bp, ki_v=ki_v,
                     kd_bp=kd_bp, kd_v=kd_v, kf=kf, native=native)
