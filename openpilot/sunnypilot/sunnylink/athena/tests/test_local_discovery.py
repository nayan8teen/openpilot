"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import socket
import time

from openpilot.common.params import Params
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
  LocalApp,
  add_local_app,
)

# Params keys these tests write (restored in teardown).
_WRITTEN_KEYS = (LOCAL_APPS_KEY, DISCOVERED_APP_KEY)


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

  def test_records_app_endpoint_when_unpaired(self):
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon(), addr)
      assert self._wait_for_endpoint(discovery, "ws://127.0.0.1:8443")
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_writes_discovered_param_when_unpaired(self):
    """A fresh app beacon is mirrored into the status param for the settings UI."""
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("app-9", 9000), addr)
      assert self._wait_for_endpoint(discovery, "ws://127.0.0.1:9000")
      raw = self._wait_for_param(DISCOVERED_APP_KEY)
      assert raw, "discovered-app status never written"
      data = json.loads(raw)
      assert data["endpoint"] == "ws://127.0.0.1:9000"
      assert data["app_id"] == "app-9"
      assert abs(int(data["ts"]) - int(time.time())) < 5  # noqa: TID251 -- wall-clock ts
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_ignores_beacons_when_paired(self):
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    listener, sender, addr = self._make_pair()
    discovery = LocalDiscovery(self.params, sock=listener)
    discovery.start()
    try:
      sender.sendto(beacon("other-app"), addr)
      time.sleep(0.3)
      assert discovery.latest_endpoint() is None
    finally:
      discovery.stop()
      discovery.join(timeout=2)
      sender.close()

  def test_clears_discovered_param_when_paired(self):
    """Once paired, beacons are ignored AND any stale discovered status is dropped."""
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    self.params.put(DISCOVERED_APP_KEY,
                    json.dumps({"endpoint": "ws://10.0.0.9:8443", "app_id": "old",
                                "ts": int(time.time())}),  # noqa: TID251 -- wall-clock ts
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
                    json.dumps({"endpoint": "ws://192.168.1.50:8443", "app_id": "app-1", "ts": ts}),
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
    self.params.put(DISCOVERED_APP_KEY, "not-json{", block=True)
    assert latest_discovered_app(self.params) is None
