from collections.abc import Sequence

import numpy as np

Gain = int | float | tuple[Sequence[float], Sequence[float]] | list[list[float]]


class AdaptiveGainConfig:
  """Bounds and rates for the online gain trim.

  The trims are multiplicative, ``K = K_base(v) * (1 + theta)``, so ``theta = 0`` means "trust the
  offline schedule" and the trim is unit free across every car and every breakpoint.

  Every rate below is expressed per adaptation sample. The controller throttles itself to
  ``adapt_period``, so the caller may call ``update_adaptive_gains`` at the full control rate.
  """

  DEFAULTS = {
    "eta_p": 3e-3,
    "eta_i": 3e-4,
    "eta_d": 3e-4,
    "bounds_p": (-0.50, 0.35),
    "bounds_i": (-0.50, 0.50),
    "bounds_d": (-0.40, 0.50),
    "slew": 0.01,           # max |dtheta/dt| in theta units per second
    "smoother_tau": 0.5,    # s, first order smoothing of the applied trim
    "variance_decay": 0.98,  # per adaptation sample
    "eps_deadband": 0.05,   # reference tracking error deadband
    "min_run": 50,          # adaptation samples above the deadband before updates start
    "leak_tau": 120.0,      # s, decay of the trims back to the baseline
    "leak_error": 0.1,      # error below which the leak is armed
    "leak_delay": 5.0,      # s of small error before leaking
    "adapt_period": 0.1,    # s, adaptation sample period
    "k_aw": 0.0,            # back calculation anti-windup gain, 1/s
  }

  def __init__(self, overrides: dict | None = None):
    if overrides:
      unknown = set(overrides) - set(self.DEFAULTS)
      if unknown:
        raise ValueError(f"unknown adaptive gain config keys: {sorted(unknown)}")
    values = dict(self.DEFAULTS)
    values.update(overrides or {})

    for name, value in values.items():
      setattr(self, name, value)

    for name in ("eta_p", "eta_i", "eta_d", "slew", "smoother_tau", "leak_tau", "leak_delay", "k_aw"):
      if getattr(self, name) < 0.0:
        raise ValueError(f"adaptive gain config {name} must be >= 0, got {getattr(self, name)}")
    if self.adapt_period <= 0.0:
      raise ValueError(f"adaptive gain config adapt_period must be > 0, got {self.adapt_period}")
    if not 0.0 <= self.variance_decay < 1.0:
      raise ValueError(f"adaptive gain config variance_decay must be in [0, 1), got {self.variance_decay}")
    if self.min_run < 1:
      raise ValueError(f"adaptive gain config min_run must be >= 1, got {self.min_run}")
    for name in ("bounds_p", "bounds_i", "bounds_d"):
      lo, hi = getattr(self, name)
      if not lo < 0.0 < hi:
        raise ValueError(f"adaptive gain config {name} must bracket zero, got {getattr(self, name)}")


class ReferenceModel:
  """Critically damped second order reference model, unit agnostic.

  The output is rate limited so the model never demands a response the actuator cannot produce,
  which would otherwise show up as a permanent tracking error and integrate the gains away.
  """

  def __init__(self, dt, omega_n, max_rate=250.0):
    if dt <= 0.0:
      raise ValueError(f"dt must be > 0, got {dt}")
    if omega_n <= 0.0:
      raise ValueError(f"omega_n must be > 0, got {omega_n}")
    self.dt = dt
    self.omega_n = float(omega_n)
    self.max_rate = float(max_rate)
    self.reset()

  def reset(self, x=0.0):
    self.x = float(x)
    self.x_dot = 0.0

  def update(self, x_des):
    x_des = float(x_des)
    x_ddot = self.omega_n**2 * (x_des - self.x) - 2.0 * self.omega_n * self.x_dot
    self.x_dot += x_ddot * self.dt
    self.x_dot = float(np.clip(self.x_dot, -self.max_rate, self.max_rate))
    self.x += self.x_dot * self.dt
    return self.x


class PIDController:
  def __init__(self, k_p: Gain, k_i: Gain, k_d: Gain = 0., pos_limit=1e308, neg_limit=-1e308, rate=100,
               mrac_config: dict | None = None):
    self._k_p = ([0], [k_p]) if isinstance(k_p, (int, float)) else k_p
    self._k_i = ([0], [k_i]) if isinstance(k_i, (int, float)) else k_i
    self._k_d = ([0], [k_d]) if isinstance(k_d, (int, float)) else k_d

    for name, gain in (("k_p", self._k_p), ("k_i", self._k_i), ("k_d", self._k_d)):
      if len(gain[0]) == 0 or len(gain[1]) == 0:
        raise ValueError(f"{name} gain schedule is empty")

    self.config = AdaptiveGainConfig(mrac_config)
    self.set_limits(pos_limit, neg_limit)

    self.rate = rate
    self.i_dt = 1.0 / rate
    self.speed = 0.0

    # multiplicative trims, K = K_base * (1 + theta)
    self._theta_p = 0.0
    self._theta_i = 0.0
    self._theta_d = 0.0
    self._theta_target_p = 0.0
    self._theta_target_i = 0.0
    self._theta_target_d = 0.0

    self.reset()

  def _base_gain(self, gain, speed):
    speeds, values = gain
    if len(speeds) == 1:
      return float(values[0])
    return float(np.interp(speed, speeds, values))

  @property
  def k_p(self):
    return self._base_gain(self._k_p, self.speed) * (1.0 + self._theta_p)

  @property
  def k_i(self):
    return self._base_gain(self._k_i, self.speed) * (1.0 + self._theta_i)

  @property
  def k_d(self):
    return self._base_gain(self._k_d, self.speed) * (1.0 + self._theta_d)

  @property
  def gains(self) -> tuple[float, float, float]:
    return self.k_p, self.k_i, self.k_d

  @property
  def trims(self) -> tuple[float, float, float]:
    return self._theta_p, self._theta_i, self._theta_d

  def set_trims(self, theta_p=0.0, theta_i=0.0, theta_d=0.0):
    """Set the applied and target trims, e.g. when restoring persisted gains."""
    bounds = [self.config.bounds_p, self.config.bounds_i, self.config.bounds_d]
    thetas = [float(theta_p), float(theta_i), float(theta_d)]
    for i, (lo, hi) in enumerate(bounds):
      thetas[i] = float(np.clip(thetas[i], lo, hi))
    self._theta_p, self._theta_i, self._theta_d = thetas
    self._theta_target_p, self._theta_target_i, self._theta_target_d = thetas

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0.0
    self._adapt_accum = 0.0
    self._adapt_run = 0
    self._leak_time = 0.0
    # statistics, not state to be cleared: keep the normalization meaningful across engagements.
    # Priors are typical magnitudes so the first updates are not quenched by a zero variance.
    if not hasattr(self, "error_variance"):
      self.error_variance = 0.04      # deg^2
      self.integral_variance = 1.0    # (deg*s)^2
      self.error_rate_variance = 1.0  # (deg/s)^2

  def set_limits(self, pos_limit, neg_limit):
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

  def _step_trims(self, dt):
    """Rate limit and smooth the applied trims toward their targets."""
    cfg = self.config
    max_step = cfg.slew * dt
    for name in ("p", "i", "d"):
      theta = getattr(self, f"_theta_{name}")
      target = getattr(self, f"_theta_target_{name}")
      if cfg.smoother_tau > 0.0:
        desired = theta + (dt / (cfg.smoother_tau + dt)) * (target - theta)
      else:
        desired = target
      theta += float(np.clip(desired - theta, -max_step, max_step))
      setattr(self, f"_theta_{name}", theta)

  def update(self, error, error_rate=0.0, speed=0.0, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self._step_trims(self.i_dt)

    self.p = self.k_p * float(error)
    self.d = self.k_d * float(error_rate)
    self.f = feedforward

    if not freeze_integrator:
      i = self.i + self.k_i * self.i_dt * float(error)

      # Don't allow windup if already clipping
      test_control = self.p + i + self.d + self.f
      i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
      i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
      self.i = float(np.clip(i, i_lowerbound, i_upperbound))

    control = self.p + self.i + self.d + self.f
    self.control = float(np.clip(control, self.neg_limit, self.pos_limit))

    # back calculation: pull the integrator toward the torque that was actually delivered. Bounded
    # by the same limits as the conditional integration above, otherwise it winds up exactly when
    # the output is pinned.
    if self.config.k_aw > 0.0 and not freeze_integrator and control != self.control:
      self.i += self.config.k_aw * (self.control - control) * self.i_dt
      self.i = float(np.clip(self.i, self.neg_limit, self.pos_limit))

    return self.control

  def _leak_trims(self, small_error):
    cfg = self.config
    if small_error:
      self._leak_time += cfg.adapt_period
    else:
      self._leak_time = 0.0

    if cfg.leak_tau > 0.0 and self._leak_time >= cfg.leak_delay:
      decay = 1.0 - min(cfg.adapt_period / cfg.leak_tau, 1.0)
      self._theta_target_p *= decay
      self._theta_target_i *= decay
      self._theta_target_d *= decay

  def update_adaptive_gains(self, error, reference_error, error_rate, enabled=True):
    """One step of the normalized MIT rule, throttled to the configured adaptation period.

    ``error`` is the plant tracking error, ``reference_error`` is ``y - y_ref`` against the
    reference model. Keeping them separate is what gives the law an equilibrium at ``y == y_ref``;
    adapting on ``error`` alone has no such fixed point.
    """
    cfg = self.config
    self._adapt_accum += self.i_dt
    if self._adapt_accum < cfg.adapt_period:
      return
    # keep the remainder so the average rate is exact for any control rate
    self._adapt_accum = max(self._adapt_accum - cfg.adapt_period, 0.0)

    error = float(error)
    reference_error = float(reference_error)
    error_rate = float(error_rate)
    integral = self.i / max(self.k_i, 1e-6)

    # Each regressor is normalized by its own running RMS. Sharing one denominator let the integral
    # term (which has different units and grows without bound) dominate the norm and quench every
    # update.
    decay = cfg.variance_decay
    self.error_variance = decay * self.error_variance + (1.0 - decay) * error**2
    self.integral_variance = decay * self.integral_variance + (1.0 - decay) * integral**2
    self.error_rate_variance = decay * self.error_rate_variance + (1.0 - decay) * error_rate**2

    if not enabled:
      self._adapt_run = 0
      self._leak_trims(abs(error) < cfg.leak_error)
      return

    if abs(reference_error) < cfg.eps_deadband:
      # a brief dip must not restart the warmup, but it must not be learned from either
      self._leak_trims(abs(error) < cfg.leak_error)
      return

    self._adapt_run += 1
    if self._adapt_run < cfg.min_run:
      return

    sigma = np.sqrt(np.array([max(self.error_variance, 1e-4),
                              max(self.integral_variance, 1e-2),
                              max(self.error_rate_variance, 1e-2)]))
    phi = np.array([error, integral, error_rate], dtype=float) / sigma
    norm = 1.0 + float(phi @ phi)

    self._theta_target_p += float(-cfg.eta_p * reference_error * phi[0] / norm)
    self._theta_target_i += float(-cfg.eta_i * reference_error * phi[1] / norm)
    self._theta_target_d += float(-cfg.eta_d * reference_error * phi[2] / norm)

    bounds = (cfg.bounds_p, cfg.bounds_i, cfg.bounds_d)
    targets = [self._theta_target_p, self._theta_target_i, self._theta_target_d]
    for i, (lo, hi) in enumerate(bounds):
      targets[i] = float(np.clip(targets[i], lo, hi))
    self._theta_target_p, self._theta_target_i, self._theta_target_d = targets
