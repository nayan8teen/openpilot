"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Trip tracking for the LOCAL sunnylink surface.

The device UI's trip summary (sunnypilot/sunnypilot#2088) is a widget: it accumulates distance in
memory while its own frame loop runs and shows a dialog when the car is shifted into park. Nothing
about that trip outlives the UI process, and nothing about it is reachable from a phone.

This module is the same arithmetic in an importable form: no cereal, no raylib, no messaging. The
local sunnylink daemon feeds it samples and serves `getTripSummary` from it, so a paired phone can
fetch a trip while it is happening and keep a copy after it ends.
"""
from __future__ import annotations

import secrets
import time

from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field

# Wire protocol version for getTripSummary. Bump only on a breaking change: additive fields are
# backward compatible and an app that predates them simply ignores what it does not know.
TRIP_PROTOCOL_VERSION = 1

# How many finished trips the daemon keeps for a phone that was away. Twenty is ~a month of daily
# driving and well under 32 KiB of RAM; the app is the system of record beyond that.
MAX_ENDED_TRIPS = 20

# One sample may never claim more than this much time. A stalled thread (a suspended daemon, a
# debugger) would otherwise book minutes of driving into a single frame.
MAX_SAMPLE_DT = 0.5

GEAR_UNKNOWN = "unknown"
GEAR_PARK = "park"

# The gears that mean "this vehicle is moving forward", mirroring the UI's FORWARD set.
FORWARD_GEARS = frozenset({"drive", "sport", "low", "brake", "eco", "manumatic"})

ENGAGED = "engaged"
LAT_ONLY = "lat_only"
LONG_ONLY = "long_only"
DISENGAGED = "disengaged"
OVERRIDE = "override"

TRIP_STATUSES = frozenset({ENGAGED, LAT_ONLY, LONG_ONLY, DISENGAGED, OVERRIDE})

# engaged% is the union of lateral and longitudinal, exactly like the device dialog.
ENGAGED_STATUSES = frozenset({ENGAGED, LAT_ONLY, LONG_ONLY})
LATERAL_STATUSES = frozenset({ENGAGED, LAT_ONLY})
LONGITUDINAL_STATUSES = frozenset({ENGAGED, LONG_ONLY})

# MADS states that mean "the driver, not sunnypilot, is doing the driving right now".
_OVERRIDE_MADS_STATES = frozenset({"paused", "overriding"})

# OpenpilotState names that stand above engagement (from selfdriveState.state).
_OVERRIDE_SS_STATES = frozenset({"preEnabled", "overriding"})


def derive_status(*, ss_state: str, ss_enabled: bool, mads_available: bool, mads_enabled: bool,
                  mads_state: str, override_longitudinal: bool = False) -> str:
  """The engagement bucket for one sample, mirroring UIStateSP.update_status.

  Same six cases as the UI, expressed on the values the daemon can read off cereal messages, so a
  trip counted on the phone matches the one the driver saw in the car.
  """
  if ss_state == "preEnabled":
    return OVERRIDE

  if ss_state == "overriding":
    if not mads_available:
      return OVERRIDE
    if override_longitudinal:
      return OVERRIDE

  if mads_state in _OVERRIDE_MADS_STATES:
    return OVERRIDE

  if not mads_available:
    return ENGAGED if ss_enabled else DISENGAGED

  if not mads_enabled and not ss_enabled:
    return DISENGAGED

  if mads_enabled and ss_enabled:
    return ENGAGED

  if mads_enabled:
    return LAT_ONLY

  if ss_enabled:
    return LONG_ONLY

  return DISENGAGED


@dataclass
class Trip:
  """One drive, accumulated in SI units and serialized for the phone."""

  trip_id: str
  started_at: float
  car_name: str = ""
  duration: float = 0.0
  distance: float = 0.0
  takeovers: int = 0
  drove: bool = False
  ended_at: float | None = None
  segments: list[dict] = field(default_factory=list)

  @property
  def ended(self) -> bool:
    return self.ended_at is not None

  def add(self, status: str, meters: float) -> None:
    """Attribute [meters] to [status], coalescing with the previous segment."""
    self.distance += meters
    if self.segments and self.segments[-1]["status"] == status:
      self.segments[-1]["meters"] += meters
      return
    self.segments.append({"status": status, "meters": meters})

  def meters_in(self, statuses: Iterable[str]) -> float:
    return sum(segment["meters"] for segment in self.segments if segment["status"] in statuses)

  def to_dict(self) -> dict:
    return {
      "id": self.trip_id,
      "state": "ended" if self.ended else "active",
      "startedAt": _millis(self.started_at),
      "endedAt": None if self.ended_at is None else _millis(self.ended_at),
      "durationSeconds": round(self.duration, 1),
      "distanceMeters": round(self.distance, 1),
      "engagedMeters": round(self.meters_in(ENGAGED_STATUSES), 1),
      "lateralMeters": round(self.meters_in(LATERAL_STATUSES), 1),
      "longitudinalMeters": round(self.meters_in(LONGITUDINAL_STATUSES), 1),
      "takeovers": self.takeovers,
      "carName": self.car_name or None,
      "segments": [{"status": s["status"], "meters": round(s["meters"], 1)} for s in self.segments],
    }


def _millis(seconds: float) -> int:
  return int(round(seconds * 1000.0))


class TripTracker:
  """The trip state machine. Pure: every input arrives as a sample, every clock is injected.

  A trip starts on the first sample with the car in gear, accumulates distance while it is not
  parked, and is finished by park (the summary moment) or by the ignition going off. A finished
  trip with no distance is discarded: a car idling in park is not a trip.
  """

  def __init__(self, wall_clock: Callable[[], float] | None = None,
               max_ended: int = MAX_ENDED_TRIPS, max_dt: float = MAX_SAMPLE_DT,
               boot_token: str | None = None):
    # Epoch seconds, not monotonic: the phone renders trip dates from these, so the wall clock is
    # the point. Everything that measures elapsed time uses the caller's monotonic `now`.
    self._wall_clock = wall_clock if wall_clock is not None else time.time  # noqa: TID251
    self._max_dt = max_dt
    # A reboot can never reproduce an id the phone already stored.
    self._boot_token = boot_token or secrets.token_hex(4)
    self._ended: deque[Trip] = deque(maxlen=max(1, max_ended))
    self._sequence = 0
    self._last_sample: float | None = None
    self._prev_status: str | None = None
    self.current: Trip | None = None
    self.car_name = ""

  def update(self, *, now: float, status: str, v_ego: float, gear: str | None,
             started: bool) -> None:
    """Feed one sample. [now] is monotonic seconds; [gear] is the CarState gear name."""
    if not started or gear is None or gear == GEAR_UNKNOWN:
      # Off, no car, or a platform that does not report gears: the drive is over.
      self.finish()
      return

    if gear == GEAR_PARK:
      self.finish()
      return

    if status not in TRIP_STATUSES:
      status = DISENGAGED

    trip = self.current
    if trip is None:
      self._sequence += 1
      trip = Trip(trip_id=f"{self._boot_token}-{self._sequence}",
                  started_at=self._wall_clock(), car_name=self.car_name)
      self.current = trip

    if gear in FORWARD_GEARS:
      trip.drove = True

    dt = 0.0 if self._last_sample is None else min(max(now - self._last_sample, 0.0), self._max_dt)
    self._last_sample = now

    if self._prev_status not in (None, DISENGAGED) and status == DISENGAGED:
      trip.takeovers += 1
    self._prev_status = status

    if dt <= 0.0:
      return

    trip.duration += dt
    meters = max(v_ego, 0.0) * dt
    if meters > 0.0:
      trip.add(status, meters)

  def finish(self) -> Trip | None:
    """End the current trip, keeping it when it covered ground. Idempotent."""
    trip = self.current
    self.current = None
    self._last_sample = None
    self._prev_status = None
    # Never drove forward (parked, or shuffled in reverse only): not a trip, exactly like the UI.
    if trip is None or trip.distance <= 0.0 or not trip.drove:
      return None
    trip.ended_at = self._wall_clock()
    self._ended.append(trip)
    return trip

  def snapshot(self, metric: bool) -> dict:
    """The `getTripSummary` payload: the active trip first, then finished ones, newest first."""
    trips: list[dict] = []
    if self.current is not None and self.current.distance > 0.0:
      trips.append(self.current.to_dict())
    trips.extend(trip.to_dict() for trip in reversed(self._ended))
    return {"v": TRIP_PROTOCOL_VERSION, "metric": bool(metric), "trips": trips}
