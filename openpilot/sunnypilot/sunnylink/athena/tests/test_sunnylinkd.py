"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
from openpilot.sunnypilot.sunnylink.athena.local_discovery import AppBeacon
from openpilot.sunnypilot.sunnylink.athena.local_pairing import LOCAL_APPS_KEY, LocalApp, add_local_app


class TestSunnylinkdMethods(OpenpilotTestCase):
  def setup_method(self):
    self.saved_params = []

    self.original_save = sunnylinkd.save_param_from_base64_encoded_string

    def mock_save_param(key, value, compression=False):
      self.saved_params.append((key, value, compression))

    sunnylinkd.save_param_from_base64_encoded_string = mock_save_param  # ty: ignore[invalid-assignment]

  def teardown_method(self):
    sunnylinkd.save_param_from_base64_encoded_string = self.original_save  # ty: ignore[invalid-assignment]

  def test_saveParams_blocked(self):
    blocked_params = {
      "GithubUsername": "attacker",
      "GithubSshKeys": "ssh-rsa attacker_key",
    }

    sunnylinkd.saveParams(blocked_params)

    assert len(self.saved_params) == 0

  def test_saveParams_allowed(self):
    allowed_params = {
      "SpeedLimitOffset": "5",
      "MyCustomParam": "123"
    }

    sunnylinkd.saveParams(allowed_params)

    # verify content
    assert len(self.saved_params) == 2
    keys_saved = [p[0] for p in self.saved_params]
    assert "SpeedLimitOffset" in keys_saved
    assert "MyCustomParam" in keys_saved

  def test_saveParams_mixed(self):
    mixed_params = {
      "GithubUsername": "attacker",
      "SpeedLimitOffset": "10"
    }

    sunnylinkd.saveParams(mixed_params)

    # should save allowed one
    assert len(self.saved_params) == 1
    assert self.saved_params[0][0] == "SpeedLimitOffset"
    assert self.saved_params[0][1] == "10"


class TestHandlePairedRefresh(OpenpilotTestCase):
  """A fresh beacon from a paired app must clear stale dial backoffs and force
  a prompt re-selection from the cloud link — but never thrash it."""

  APP_ID = "app-1"
  ENDPOINT = "ws://192.168.1.50:8443"

  def setup_method(self):
    self.saved = Params().get(LOCAL_APPS_KEY)
    add_local_app(LocalApp(app_id=self.APP_ID, endpoint=self.ENDPOINT))
    self.backoffs: dict[str, float] = {}
    self.force_attempts: dict[str, float] = {}
    self.saved_active_local = sunnylinkd._active_local_endpoint
    self.saved_active_ws = sunnylinkd._active_ws

  def teardown_method(self):
    sunnylinkd._active_local_endpoint = self.saved_active_local
    sunnylinkd._active_ws = self.saved_active_ws
    if self.saved is None:
      Params().remove(LOCAL_APPS_KEY)
    else:
      Params().put(LOCAL_APPS_KEY, self.saved, block=True)

  def _beacon(self) -> AppBeacon:
    return AppBeacon(app_id=self.APP_ID, ws_port=8443, source_ip="192.168.1.50")

  def test_fresh_beacon_clears_backoff_and_forces_reconnect_from_cloud(self):
    """On the cloud link with a stale dial backoff: the beacon clears the
    backoff and closes the active ws so the loop re-selects local promptly."""
    self.backoffs[self.ENDPOINT] = time.monotonic() + 300
    closed = []
    class FakeWs:
      def close(self):
        closed.append(True)
    sunnylinkd._active_local_endpoint = None
    sunnylinkd._active_ws = FakeWs()

    sunnylinkd._handle_paired_refresh(self.backoffs, self.force_attempts, self._beacon())

    assert self.ENDPOINT not in self.backoffs
    assert closed == [True]
    assert self.force_attempts[self.ENDPOINT] > 0

  def test_reconnect_guarded_to_once_per_staleness_window(self):
    """A failed forced dial must not thrash the cloud link: repeated fresh
    beacons for the same endpoint force a re-selection at most once per
    LOCAL_BEACON_FRESH_S."""
    closed = []
    class FakeWs:
      def close(self):
        closed.append(True)
    sunnylinkd._active_local_endpoint = None
    sunnylinkd._active_ws = FakeWs()
    beacon = self._beacon()

    sunnylinkd._handle_paired_refresh(self.backoffs, self.force_attempts, beacon)
    sunnylinkd._handle_paired_refresh(self.backoffs, self.force_attempts, beacon)  # 5s later
    sunnylinkd._handle_paired_refresh(self.backoffs, self.force_attempts, beacon)

    assert len(closed) == 1

  def test_already_connected_to_endpoint_is_noop(self):
    """Already serving this app locally — the beacon changes nothing."""
    self.backoffs[self.ENDPOINT] = time.monotonic() + 300
    closed = []
    class FakeWs:
      def close(self):
        closed.append(True)
    sunnylinkd._active_local_endpoint = self.ENDPOINT
    sunnylinkd._active_ws = FakeWs()

    sunnylinkd._handle_paired_refresh(self.backoffs, self.force_attempts, self._beacon())

    assert closed == []
    # The stale backoff is still cleared — the beacon proves the app is up.
    assert self.ENDPOINT not in self.backoffs

  def test_serving_another_local_app_is_noop(self):
    """Connected to a DIFFERENT local app — no forced switch (the natural
    reconnect cycle re-picks, and selection prefers the fresh beacon)."""
    closed = []
    class FakeWs:
      def close(self):
        closed.append(True)
    sunnylinkd._active_local_endpoint = "ws://192.168.1.99:8443"
    sunnylinkd._active_ws = FakeWs()

    sunnylinkd._handle_paired_refresh(self.backoffs, self.force_attempts, self._beacon())

    assert closed == []
    assert self.force_attempts == {}
