"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import socket
import time

from openpilot.common.params import Params, params_put
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot.sunnylink.athena.local_discovery import (
  LOCAL_BEACON_FRESH_S,
  LocalDiscovery,
  latest_discovered_app,
  parse_beacon,
)
from openpilot.sunnypilot.sunnylink.athena.local_pairing import (
  DISCOVERED_APP_KEY,
  LOCAL_APPS_KEY,
  PAIRING_CODE_KEY,
  PAIRING_REQUEST_KEY,
  LocalApp,
  add_local_app,
  arm_pairing,
  clear_pairing_request,
  get_local_apps,
)

# Params keys these tests write (restored in teardown).
_WRITTEN_KEYS = (LOCAL_APPS_KEY, DISCOVERED_APP_KEY, PAIRING_REQUEST_KEY, PAIRING_CODE_KEY)


def beacon(app_id="app-1", ws_port=8443) -> bytes:
  return f'SUNNYLINK1 {json.dumps({"v": 1, "role": "app", "app_id": app_id, "ws_port": ws_port})}'.encode()


class TestParseBeacon(OpenpilotTestCase):
  def test_valid_beacon(self):
    parsed = parse_beacon(beacon("app-1", 8443), "192.168.1.50")
    assert parsed is not None
    assert parsed.app_id == "app-1"
    assert parsed.ws_port == 8443
    assert parsed.endpoint == "ws://192.168.1.50:8443"

  def test_valid_beacon_bytes_and_whitespace(self):
    parsed = parse_beacon(b"  " + beacon() + b"\n", "10.0.0.3")
    assert parsed is not None
    assert parsed.endpoint == "ws://10.0.0.3:8443"

  def test_rejects_non_app_role(self):
    device = f'SUNNYLINK1 {json.dumps({"v": 1, "role": "device", "dongle_id": "x"})}'
    assert parse_beacon(device) is None

  def test_rejects_garbage(self):
    assert parse_beacon(b"random noise") is None
    assert parse_beacon(b"SUNNYLINK1 not-json") is None
    assert parse_beacon(b"") is None
    assert parse_beacon('SUNNYLINK1 {"v":1,"role":"app"}') is None  # missing app_id
    assert parse_beacon('SUNNYLINK1 {"v":1,"role":"app","app_id":"x","ws_port":999999}') is None


class TestLocalDiscovery(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = {key: self.params.get(key) for key in _WRITTEN_KEYS}

  def teardown_method(self):
    for key, value in self.saved.items():
      if value is None:
        self.params.remove(key)
      else:
        self.params.put(key, value, block=True)

  def _make_pair(self):
    """Listener socket (bound, loopback) + a sender socket."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    listener.settimeout(0.2)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return listener, sender, listener.getsockname()

  def _wait_for_endpoint(self, discovery, expected, timeout_s=3):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
      if discovery.latest_endpoint() == expected:
        return True
      time.sleep(0.02)
    return False

  def _wait_for_param(self, key, timeout_s=3):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
      value = self.params.get(key)
      if value is not None:
        return value
      time.sleep(0.02)
    return None

  def test_records_app_endpoint_while_window_armed(self):
    arm_pairing(self.params)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon(), addr)
      assert self._wait_for_endpoint(discovery, "ws://127.0.0.1:8443")
      assert discovery.latest_app_id() == "app-1"
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_writes_discovered_param_while_window_armed(self):
    """A fresh app beacon is mirrored into the status param for the settings UI."""
    arm_pairing(self.params)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("app-9", 9000), addr)
      assert self._wait_for_endpoint(discovery, "ws://127.0.0.1:9000")
      data = self._wait_for_param(DISCOVERED_APP_KEY)
      assert data, "discovered-app status never written"
      assert data["endpoint"] == "ws://127.0.0.1:9000"
      assert data["app_id"] == "app-9"
      assert abs(int(data["ts"]) - int(time.time())) < 5  # noqa: TID251 -- wall-clock ts
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_ignores_beacons_without_window(self):
    """No window armed → beacons ignored entirely (pairing is button-driven)."""
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("other-app"), addr)
      time.sleep(0.3)
      assert discovery.latest_endpoint() is None
      assert discovery.latest_app_id() is None
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_records_new_app_beacon_while_paired(self):
    """A paired device still listens during a window, so a SECOND app can pair."""
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    arm_pairing(self.params)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("app-2", 8443), addr)
      assert self._wait_for_endpoint(discovery, "ws://127.0.0.1:8443")
      assert discovery.latest_app_id() == "app-2"
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_paired_beacon_refreshes_registry_outside_window(self):
    """A beacon from an ALREADY-PAIRED app updates its cached endpoint even
    without a window — IPs change; the app_id is the identity."""
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443",
                           app_name="Pixel"), self.params)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("app-1"), addr)
      deadline = time.monotonic() + 3
      while time.monotonic() < deadline:
        apps = get_local_apps(self.params)
        if apps and apps[0].endpoint == "ws://127.0.0.1:8443":
          break
        time.sleep(0.02)
      apps = get_local_apps(self.params)
      assert len(apps) == 1
      assert apps[0].endpoint == "ws://127.0.0.1:8443"
      assert apps[0].app_name == "Pixel"  # untouched
      assert apps[0].paired_at > 0
      # In-memory paired-beacon state drives connection selection.
      assert discovery.latest_paired_endpoint() == "ws://127.0.0.1:8443"
      assert discovery.latest_paired_app_id() == "app-1"
      assert discovery.latest_paired_seen_ago() is not None
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_unknown_beacon_outside_window_does_not_touch_registry(self):
    """Beacons from unknown apps are still ignored outside a window — only
    already-paired apps are re-learned."""
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("stranger"), addr)
      time.sleep(0.3)
      apps = get_local_apps(self.params)
      assert [a.app_id for a in apps] == ["app-1"]
      assert apps[0].endpoint == "ws://10.0.0.5:8443"
      assert discovery.latest_paired_endpoint() is None
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_paired_beacon_fires_callback_even_without_change(self):
    """Callback fires on EVERY fresh beacon, not just on address change — a
    beacon proves the app's server is up, so the daemon can re-select even on
    the same address. The registry is still written only on change."""
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    calls: list[str] = []
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener,
                               paired_refresh_cb=lambda b: calls.append(b.endpoint))
    discovery.start()
    try:
      sender.sendto(beacon("app-1"), addr)
      deadline = time.monotonic() + 3
      while time.monotonic() < deadline:
        apps = get_local_apps(self.params)
        if apps and apps[0].endpoint == "ws://127.0.0.1:8443":
          break
        time.sleep(0.02)
      time.sleep(0.1)
      sender.sendto(beacon("app-1"), addr)  # same address — still a fresh beacon
      time.sleep(0.2)
      # Both beacons fired the callback even though the second changed nothing.
      assert calls == ["ws://127.0.0.1:8443", "ws://127.0.0.1:8443"]
      apps = get_local_apps(self.params)
      assert apps[0].endpoint == "ws://127.0.0.1:8443"
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_paired_refresh_also_runs_while_window_armed(self):
    """Endpoint refresh is orthogonal to the window — arming it doesn't stop
    paired-app refreshes."""
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    arm_pairing(self.params)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("app-1"), addr)
      deadline = time.monotonic() + 3
      while time.monotonic() < deadline:
        apps = get_local_apps(self.params)
        if apps and apps[0].endpoint == "ws://127.0.0.1:8443":
          break
        time.sleep(0.02)
      apps = get_local_apps(self.params)
      assert apps[0].endpoint == "ws://127.0.0.1:8443"
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_clears_discovered_param_when_window_closed(self):
    """Once the window closes, beacons are ignored AND any stale discovered
    status is dropped."""
    arm_pairing(self.params)
    clear_pairing_request(self.params)
    self.params.put(DISCOVERED_APP_KEY,
                    {"endpoint": "ws://10.0.0.9:8443", "app_id": "old",
                     "ts": int(time.time())},  # noqa: TID251 -- wall-clock ts
                    block=True)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("other-app"), addr)
      time.sleep(0.3)
      assert self.params.get(DISCOVERED_APP_KEY) is None
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_garbage_beacons_do_not_write_status(self):
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(b"junk", addr)
      sender.sendto(b'SUNNYLINK1 {"v":1,"role":"device"}', addr)
      time.sleep(0.3)
      assert discovery.latest_endpoint() is None
      assert self.params.get(DISCOVERED_APP_KEY) is None
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()


class TestLatestDiscoveredApp(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = self.params.get(DISCOVERED_APP_KEY)

  def teardown_method(self):
    if self.saved is None:
      self.params.remove(DISCOVERED_APP_KEY)
    else:
      self.params.put(DISCOVERED_APP_KEY, self.saved, block=True)

  def _put(self, ts):
    self.params.put(DISCOVERED_APP_KEY,
                    {"endpoint": "ws://192.168.1.50:8443", "app_id": "app-1", "ts": ts},
                    block=True)

  def test_none_when_empty(self):
    assert latest_discovered_app(self.params) is None

  def test_fresh_beacon(self):
    self._put(int(time.time()))  # noqa: TID251 -- wall-clock ts
    info = latest_discovered_app(self.params)
    assert info is not None
    endpoint, age = info
    assert endpoint == "ws://192.168.1.50:8443"
    assert 0 <= age < 5

  def test_stale_beacon(self):
    self._put(int(time.time()) - LOCAL_BEACON_FRESH_S - 10)  # noqa: TID251 -- wall-clock ts
    assert latest_discovered_app(self.params) is None

  def test_corrupt_status(self):
    # Legacy STRING-era value (raw JSON text) or external corruption.
    params_put(self.params.p, b"SunnylinkLocalDiscoveredApp", b"not-json{", len(b"not-json{"), True)
    assert latest_discovered_app(self.params) is None

  def test_wrong_shape_status(self):
    self.params.put(DISCOVERED_APP_KEY, ["not", "a", "dict"], block=True)
    assert latest_discovered_app(self.params) is None
