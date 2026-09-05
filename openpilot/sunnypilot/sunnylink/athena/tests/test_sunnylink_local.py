"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import queue
import time

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from websocket import WebSocketTimeoutException

from openpilot.system.athena.rpc import dumps_call
from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
from openpilot.sunnypilot.sunnylink.athena.local_pairing import LocalApp


class FakePairingWs:
  """Minimal ws stand-in for the pairing session: recv pulls JSON from a queue."""

  def __init__(self):
    self.inbox = queue.Queue()
    self.sent: list[str] = []
    self.timeout: float | None = None
    self.closed = False

  def recv(self):
    # Mirrors a real socket: blocks up to ~0.2s, then raises a timeout so the
    # pairing session's deadline loop can make progress on an idle connection.
    try:
      item = self.inbox.get(timeout=0.2)
    except queue.Empty:
      raise WebSocketTimeoutException() from None
    if isinstance(item, Exception):
      raise item
    return item

  def send(self, data: str, opcode=None):
    self.sent.append(data)

  def settimeout(self, seconds: float):
    self.timeout = seconds

  def close(self):
    self.closed = True

  def feed_call(self, method: str, params=None):
    self.inbox.put(dumps_call(method, params, request_id=42))


def _reset_module_state(mocker):
  mocker.patch.object(sunnylinkd, "_active_local_endpoint", None)


class TestPairLocalAppHandler(OpenpilotTestCase):
  def setup_method(self):
    self.registry: list[LocalApp] = []

  def test_success_pins_active_endpoint(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.5:8443")
    mocker.patch.object(sunnylinkd, "verify_pairing_code", return_value=True)
    mocker.patch.object(sunnylinkd, "add_local_app", side_effect=lambda app: self.registry.append(app))

    result = sunnylinkd.pairLocalApp(code="ABC123", app_id="app-1", app_name="Pixel 9")
    assert result == {"success": True}
    assert len(self.registry) == 1
    app = self.registry[0]
    assert app.app_id == "app-1"
    assert app.endpoint == "ws://10.0.0.5:8443"
    assert app.app_name == "Pixel 9"

  def test_invalid_code_rejected(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.5:8443")
    mocker.patch.object(sunnylinkd, "verify_pairing_code", return_value=False)
    mocker.patch.object(sunnylinkd, "add_local_app", side_effect=AssertionError("must not pair"))

    result = sunnylinkd.pairLocalApp(code="WRONG")
    assert result["success"] is False
    assert "invalid code" in str(result["error"])

  def test_rejected_without_local_connection(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", None)
    mocker.patch.object(sunnylinkd, "verify_pairing_code", return_value=True)
    result = sunnylinkd.pairLocalApp(code="ABC123")
    assert result["success"] is False

  def test_unpair_removes_app(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.5:8443")
    mocker.patch.object(sunnylinkd, "remove_local_app", return_value=True)
    result = sunnylinkd.unpairLocalApp(app_id="app-1")
    assert result == {"success": True, "removed": True}
    sunnylinkd.remove_local_app.assert_called_once_with("app-1")  # type: ignore[attr-defined]


class TestPairingSession(OpenpilotTestCase):
  def test_pairs_over_session(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.5:8443")
    paired = {"value": False}

    def fake_verify(code):
      return code == "ABC123"

    def fake_add(app):
      paired["value"] = True  # the registry gains the app

    mocker.patch.object(sunnylinkd, "verify_pairing_code", side_effect=fake_verify)
    mocker.patch.object(sunnylinkd, "add_local_app", side_effect=fake_add)
    mocker.patch.object(sunnylinkd, "is_locally_paired", side_effect=lambda: paired["value"])

    ws = FakePairingWs()
    ws.feed_call("pairLocalApp", {"code": "ABC123", "app_id": "app-1"})
    assert sunnylinkd._pairing_session(ws, timeout_s=5)
    assert len(ws.sent) == 1
    resp = json.loads(ws.sent[0])
    assert resp["result"] == {"success": True}
    assert ws.timeout == 70  # restored to the long-poll timeout afterwards

  def test_invalid_code_times_out_unpaired(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.5:8443")
    mocker.patch.object(sunnylinkd, "verify_pairing_code", return_value=False)
    mocker.patch.object(sunnylinkd, "is_locally_paired", return_value=False)
    mocker.patch.object(sunnylinkd, "add_local_app", side_effect=AssertionError("must not pair"))

    ws = FakePairingWs()
    ws.feed_call("pairLocalApp", {"code": "WRONG"})
    assert not sunnylinkd._pairing_session(ws, timeout_s=0.1)
    resp = json.loads(ws.sent[0])
    assert resp["result"]["success"] is False

  def test_refuses_non_pairing_rpc_until_paired(self, mocker):
    _reset_module_state(mocker)
    mocker.patch.object(sunnylinkd, "is_locally_paired", return_value=False)

    ws = FakePairingWs()
    ws.feed_call("getParams", {"params_keys": ["SpeedLimitOffset"], "compression": False})
    ws.inbox.put(WebSocketTimeoutException())
    assert not sunnylinkd._pairing_session(ws, timeout_s=1)
    assert ws.sent == []  # nothing but pairing RPC is answered


class DummyDiscovery:
  def __init__(self, latest=None, seen_ago=None):
    self._latest = latest
    self._seen = seen_ago

  def latest_endpoint(self):
    return self._latest

  def last_seen_ago(self):
    return self._seen


class TestConnectionSelection(OpenpilotTestCase):
  def test_paired_local_first_most_recent(self, mocker):
    apps = [
      LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443", app_name="older"),
      LocalApp(app_id="b", endpoint="ws://10.0.0.3:8443", app_name="newer"),
    ]
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery(), {})
    assert (uri, kind) == ("ws://10.0.0.3:8443", "paired_local")

  def test_backoff_skips_local_endpoint(self, mocker):
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    future = time.monotonic() + 9999
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery(), {"ws://10.0.0.2:8443": future})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")

  def test_fresh_beacon_offers_pairing_when_unpaired(self, mocker):
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=[])
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.5:8443", 2), {})
    assert (uri, kind) == ("ws://10.0.0.5:8443", "pairing_offer")

  def test_stale_beacon_ignored(self, mocker):
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=[])
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.5:8443", 9999), {})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")

  def test_cloud_when_nothing_local(self, mocker):
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=[])
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery(None, None), {})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")

  def test_paired_device_ignores_discovery(self, mocker):
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.99:8443", 1), {})
    assert (uri, kind) == ("ws://10.0.0.2:8443", "paired_local")


class TestServiceable(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = {key: self.params.get(key) for key in ("SunnylinkEnabled", "SunnylinkTempFault")}
    self.params.put_bool("SunnylinkEnabled", True, block=True)
    self.params.put_bool("SunnylinkTempFault", False, block=True)

  def teardown_method(self):
    for key, value in self.saved.items():
      if value is None:
        self.params.remove(key)
      else:
        self.params.put(key, value, block=True)

  def test_gate(self):
    assert sunnylinkd._serviceable(self.params)
    self.params.put_bool("SunnylinkTempFault", True, block=True)
    assert not sunnylinkd._serviceable(self.params)
    self.params.put_bool("SunnylinkTempFault", False, block=True)
    self.params.put_bool("SunnylinkEnabled", False, block=True)
    assert not sunnylinkd._serviceable(self.params)
