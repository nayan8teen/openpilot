"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace
from unittest import mock

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.sunnylink.athena import local_trip
from openpilot.sunnypilot.sunnylink.athena.local_auth import LOCAL_METHODS
from openpilot.sunnypilot.sunnylink.athena.local_trip import (
  DISENGAGED,
  ENGAGED,
  GEAR_PARK,
  LAT_ONLY,
  LONG_ONLY,
  OVERRIDE,
  TripTracker,
  derive_status,
)
from openpilot.sunnypilot.sunnylink.athena.localsunnylinkd import LocalSunnylinkd
from openpilot.sunnypilot.sunnylink.athena.sunnylinkd import Sunnylinkd

STEP = 0.1
SPEED = 10.0  # m/s → 1 m per 0.1 s sample


class _Clock:
  """A monotonic clock that only advances when a test says so."""

  def __init__(self, start: float = 1_000.0):
    self.value = start

  def __call__(self) -> float:
    return self.value


def _feed(tracker: TripTracker, clock: _Clock, count: int, *, status: str = ENGAGED,
          v_ego: float = SPEED, gear: str = "drive", started: bool = True,
          step: float = STEP) -> None:
  for _ in range(count):
    clock.value += step
    tracker.update(now=clock.value, status=status, v_ego=v_ego, gear=gear, started=started)


def _park(tracker: TripTracker, clock: _Clock) -> None:
  _feed(tracker, clock, 1, status=DISENGAGED, v_ego=0.0, gear=GEAR_PARK)


class TestTripTracker(OpenpilotTestCase):
  def setup_method(self):
    self.clock = _Clock()
    self.tracker = TripTracker(wall_clock=self.clock, boot_token="boot")

  def test_distance_is_attributed_to_the_engagement_bucket(self):
    # The first sample of a trip establishes the baseline: n samples cover n - 1 intervals.
    _feed(self.tracker, self.clock, 101, status=ENGAGED)   # 100 intervals → 100 m engaged
    _feed(self.tracker, self.clock, 51, status=LAT_ONLY)   # 51 intervals → 51 m lateral-only

    trip = self.tracker.snapshot(metric=True)["trips"][0]
    self.assertAlmostEqual(trip["distanceMeters"], 151.0, places=1)
    # engaged% is the union of lateral and longitudinal, like the device dialog.
    self.assertAlmostEqual(trip["engagedMeters"], 151.0, places=1)
    self.assertAlmostEqual(trip["lateralMeters"], 151.0, places=1)
    self.assertAlmostEqual(trip["longitudinalMeters"], 100.0, places=1)
    self.assertEqual([s["status"] for s in trip["segments"]], [ENGAGED, LAT_ONLY])
    self.assertEqual(trip["state"], "active")

  def test_consecutive_samples_of_one_status_coalesce_into_one_segment(self):
    _feed(self.tracker, self.clock, 51, status=ENGAGED)
    _feed(self.tracker, self.clock, 51, status=LONG_ONLY)
    _feed(self.tracker, self.clock, 51, status=ENGAGED)

    segments = self.tracker.snapshot(metric=True)["trips"][0]["segments"]
    self.assertEqual([s["status"] for s in segments], [ENGAGED, LONG_ONLY, ENGAGED])

  def test_a_finished_trip_reports_its_ended_state_and_end_time(self):
    _feed(self.tracker, self.clock, 11)
    _park(self.tracker, self.clock)

    trips = self.tracker.snapshot(metric=True)["trips"]
    self.assertEqual(len(trips), 1)
    self.assertEqual(trips[0]["state"], "ended")
    self.assertIsNotNone(trips[0]["endedAt"])
    self.assertGreaterEqual(trips[0]["endedAt"], trips[0]["startedAt"])

  def test_the_ignition_going_off_finishes_the_trip(self):
    _feed(self.tracker, self.clock, 11)
    _feed(self.tracker, self.clock, 1, status=DISENGAGED, v_ego=0.0, gear=None, started=False)

    trips = self.tracker.snapshot(metric=True)["trips"]
    self.assertEqual(len(trips), 1)
    self.assertEqual(trips[0]["state"], "ended")

  def test_a_car_that_never_moved_is_not_a_trip(self):
    # Parked with the ignition on: the state machine runs, nothing is kept.
    for _ in range(20):
      self.clock.value += STEP
      self.tracker.update(now=self.clock.value, status=DISENGAGED, v_ego=0.0,
                          gear=GEAR_PARK, started=True)

    self.assertEqual(self.tracker.snapshot(metric=True)["trips"], [])

  def test_shuffling_in_reverse_only_is_not_a_trip(self):
    _feed(self.tracker, self.clock, 11, gear="reverse")
    _park(self.tracker, self.clock)

    self.assertEqual(self.tracker.snapshot(metric=True)["trips"], [])

  def test_one_sample_is_not_enough_to_start_a_trip(self):
    _feed(self.tracker, self.clock, 1)
    _park(self.tracker, self.clock)

    self.assertEqual(self.tracker.snapshot(metric=True)["trips"], [])

  def test_a_stalled_loop_cannot_book_minutes_into_one_sample(self):
    _feed(self.tracker, self.clock, 2, step=5.0)   # second sample: dt capped at MAX_SAMPLE_DT

    trip = self.tracker.snapshot(metric=True)["trips"][0]
    self.assertAlmostEqual(trip["durationSeconds"], local_trip.MAX_SAMPLE_DT, places=1)
    self.assertAlmostEqual(trip["distanceMeters"], SPEED * local_trip.MAX_SAMPLE_DT, places=1)

  def test_duration_keeps_running_while_the_car_stands_still(self):
    _feed(self.tracker, self.clock, 11, v_ego=0.0)

    trip = self.tracker.current
    self.assertIsNotNone(trip)
    self.assertAlmostEqual(trip.duration, 1.0, places=6)
    self.assertEqual(trip.distance, 0.0)
    # Nothing to show while it has not covered ground.
    self.assertEqual(self.tracker.snapshot(metric=True)["trips"], [])

  def test_takeovers_count_engagements_lost_not_samples(self):
    _feed(self.tracker, self.clock, 11, status=ENGAGED)
    _feed(self.tracker, self.clock, 11, status=DISENGAGED)
    _feed(self.tracker, self.clock, 11, status=ENGAGED)
    _feed(self.tracker, self.clock, 11, status=DISENGAGED)

    self.assertEqual(self.tracker.current.takeovers, 2)

  def test_an_override_is_not_a_takeover_until_it_disengages(self):
    _feed(self.tracker, self.clock, 11, status=ENGAGED)
    _feed(self.tracker, self.clock, 11, status=OVERRIDE)
    _feed(self.tracker, self.clock, 11, status=DISENGAGED)

    self.assertEqual(self.tracker.current.takeovers, 1)

  def test_the_ring_keeps_the_newest_finished_trips(self):
    for _ in range(local_trip.MAX_ENDED_TRIPS + 5):
      _feed(self.tracker, self.clock, 3)
      _park(self.tracker, self.clock)

    trips = self.tracker.snapshot(metric=True)["trips"]
    self.assertEqual(len(trips), local_trip.MAX_ENDED_TRIPS)
    self.assertEqual(trips[0]["id"], f"boot-{local_trip.MAX_ENDED_TRIPS + 5}")
    self.assertEqual(trips[-1]["id"], "boot-6")

  def test_ids_are_stable_for_a_trip_and_distinct_across_boots(self):
    _feed(self.tracker, self.clock, 11)
    first = self.tracker.current.trip_id
    _feed(self.tracker, self.clock, 11)
    self.assertEqual(self.tracker.current.trip_id, first)

    other = TripTracker(wall_clock=_Clock(), boot_token="other-boot")
    _feed(other, _Clock(), 11)
    self.assertNotEqual(other.current.trip_id, first)

  def test_metric_is_reported_from_the_caller(self):
    self.assertTrue(self.tracker.snapshot(metric=True)["metric"])
    self.assertFalse(self.tracker.snapshot(metric=False)["metric"])


class TestDeriveStatus(OpenpilotTestCase):
  def _status(self, **kwargs):
    defaults = {"ss_state": "enabled", "ss_enabled": False, "mads_available": False,
                "mads_enabled": False, "mads_state": "disabled", "override_longitudinal": False}
    return derive_status(**(defaults | kwargs))

  def test_a_plain_platform_reports_engagement_only(self):
    self.assertEqual(self._status(ss_enabled=True), ENGAGED)
    self.assertEqual(self._status(ss_enabled=False), DISENGAGED)

  def test_mads_splits_lateral_from_longitudinal(self):
    self.assertEqual(self._status(mads_available=True, mads_enabled=True, ss_enabled=True), ENGAGED)
    self.assertEqual(self._status(mads_available=True, mads_enabled=True), LAT_ONLY)
    self.assertEqual(self._status(mads_available=True, ss_enabled=True), LONG_ONLY)
    self.assertEqual(self._status(mads_available=True), DISENGAGED)

  def test_override_states_outrank_engagement(self):
    self.assertEqual(self._status(ss_state="preEnabled", ss_enabled=True), OVERRIDE)
    self.assertEqual(self._status(ss_state="overriding", ss_enabled=True), OVERRIDE)
    self.assertEqual(self._status(mads_available=True, mads_enabled=True, mads_state="paused"), OVERRIDE)
    self.assertEqual(self._status(mads_available=True, mads_enabled=True, mads_state="overriding"), OVERRIDE)

  def test_an_overriding_mads_without_longitudinal_override_still_reports_engagement(self):
    self.assertEqual(
      self._status(ss_state="overriding", ss_enabled=True, mads_available=True,
                   mads_enabled=True, override_longitudinal=False),
      ENGAGED,
    )


class _FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get_bool(self, key):
    return bool(self.values.get(key))

  def get(self, key, *args, **kwargs):
    return self.values.get(key)


class _CallingSession:
  """Stands in for AuthorizedSession.invoke: runs the gated handler as the daemon does."""

  def __init__(self):
    self.grant = SimpleNamespace(app_id="grant-key")
    self.calls: list[str] = []

  def invoke(self, method, handler, *args, **kwargs):
    self.calls.append(method)
    return handler(*args, **kwargs)

  def close(self):
    pass


class TestTripSummaryRpc(OpenpilotTestCase):
  def _daemon(self, values=None):
    return LocalSunnylinkd(params=_FakeParams(values), discovery=mock.Mock())

  def test_the_method_is_local_only(self):
    self.assertIn("getTripSummary", LOCAL_METHODS)
    # Never part of the cloud surface the athena backend serves.
    self.assertNotIn("getTripSummary", Sunnylinkd.RPC_METHODS)

  def test_a_local_session_is_served_the_empty_contract(self):
    methods = self._daemon({"IsMetric": True}).session_methods(_CallingSession())
    self.assertIn("getTripSummary", methods)
    self.assertEqual(methods["getTripSummary"](),
                     {"v": local_trip.TRIP_PROTOCOL_VERSION, "metric": True, "trips": []})

  def test_metric_follows_the_device_param(self):
    self.assertTrue(self._daemon({"IsMetric": True}).trip_summary()["metric"])
    self.assertFalse(self._daemon({"IsMetric": False}).trip_summary()["metric"])

  def test_the_summary_reflects_the_tracker(self):
    daemon = self._daemon()
    clock = _Clock()
    _feed(daemon.trip, clock, 11)
    _park(daemon.trip, clock)

    trips = daemon.trip_summary()["trips"]
    self.assertEqual(len(trips), 1)
    self.assertEqual(trips[0]["state"], "ended")
