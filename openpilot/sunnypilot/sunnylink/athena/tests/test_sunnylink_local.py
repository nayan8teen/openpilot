"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import queue
import threading
import time

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from websocket import WebSocketTimeoutException

from openpilot.system.athena.rpc import dumps_call
from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
from openpilot.sunnypilot.sunnylink.athena.local_discovery import AppBeacon
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

  def test_success_pins_active_endpoint_and_closes_window(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.5:8443")
    mocker.patch.object(sunnylinkd, "verify_pairing_code", return_value=True)
    mocker.patch.object(sunnylinkd, "add_local_app", side_effect=lambda app: self.registry.append(app))
    mocker.patch.object(sunnylinkd, "clear_pairing_request")  # window closes on success

    result = sunnylinkd.pairLocalApp(code="ABC123", app_id="app-1", app_name="Pixel 9")
    assert result == {"success": True}
    assert len(self.registry) == 1
    app = self.registry[0]
    assert app.app_id == "app-1"
    assert app.endpoint == "ws://10.0.0.5:8443"
    assert app.app_name == "Pixel 9"
    sunnylinkd.clear_pairing_request.assert_called_once()

  def test_invalid_code_rejected(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.5:8443")
    mocker.patch.object(sunnylinkd, "verify_pairing_code", return_value=False)
    mocker.patch.object(sunnylinkd, "add_local_app", side_effect=AssertionError("must not pair"))
    mocker.patch.object(sunnylinkd, "clear_pairing_request",
                        side_effect=AssertionError("window must stay open on failure"))

    result = sunnylinkd.pairLocalApp(code="WRONG")
    assert result["success"] is False
    assert "invalid code" in str(result["error"])

  def test_rejected_without_local_connection(self, mocker):
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", None)
    mocker.patch.object(sunnylinkd, "verify_pairing_code", return_value=True)
    mocker.patch.object(sunnylinkd, "clear_pairing_request",
                        side_effect=AssertionError("window must stay open on failure"))
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
    armed = {"value": True}

    def fake_verify(code):
      return code == "ABC123"

    def fake_add(app):
      paired["value"] = True  # the registry gains the app

    def fake_clear():
      armed["value"] = False  # pairLocalApp closes the pairing window

    mocker.patch.object(sunnylinkd, "verify_pairing_code", side_effect=fake_verify)
    mocker.patch.object(sunnylinkd, "add_local_app", side_effect=fake_add)
    mocker.patch.object(sunnylinkd, "is_locally_paired", side_effect=lambda: paired["value"])
    mocker.patch.object(sunnylinkd, "pairing_requested", side_effect=lambda: armed["value"])
    mocker.patch.object(sunnylinkd, "clear_pairing_request", side_effect=fake_clear)

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
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=True)
    mocker.patch.object(sunnylinkd, "clear_pairing_request",
                        side_effect=AssertionError("window must stay open on failure"))

    ws = FakePairingWs()
    ws.feed_call("pairLocalApp", {"code": "WRONG"})
    assert not sunnylinkd._pairing_session(ws, timeout_s=0.1)
    resp = json.loads(ws.sent[0])
    assert resp["result"]["success"] is False

  def test_refuses_non_pairing_rpc_until_paired(self, mocker):
    _reset_module_state(mocker)
    mocker.patch.object(sunnylinkd, "is_locally_paired", return_value=False)
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=True)

    ws = FakePairingWs()
    ws.feed_call("getParams", {"params_keys": ["SpeedLimitOffset"], "compression": False})
    ws.inbox.put(WebSocketTimeoutException())
    assert not sunnylinkd._pairing_session(ws, timeout_s=1)
    assert ws.sent == []  # nothing but pairing RPC is answered


class DummyDiscovery:
  def __init__(self, latest=None, seen_ago=None, app_id=None,
               paired_endpoint=None, paired_app_id=None, paired_seen_ago=None):
    self._latest = latest
    self._seen = seen_ago
    self._app_id = app_id
    self._paired_endpoint = paired_endpoint
    self._paired_app_id = paired_app_id
    self._paired_seen_ago = paired_seen_ago

  def latest_endpoint(self):
    return self._latest

  def latest_app_id(self):
    return self._app_id

  def last_seen_ago(self):
    return self._seen

  def latest_paired_endpoint(self):
    return self._paired_endpoint

  def latest_paired_app_id(self):
    return self._paired_app_id

  def latest_paired_seen_ago(self):
    return self._paired_seen_ago


class TestConnectionSelection(OpenpilotTestCase):
  def test_paired_local_first_most_recent(self, mocker):
    apps = [
      LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443", app_name="older"),
      LocalApp(app_id="b", endpoint="ws://10.0.0.3:8443", app_name="newer"),
    ]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery(), {})
    assert (uri, kind) == ("ws://10.0.0.3:8443", "paired_local")

  def test_backoff_skips_local_endpoint(self, mocker):
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    future = time.monotonic() + 9999
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery(), {"ws://10.0.0.2:8443": future})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")

  def test_armed_window_offers_pairing_to_new_app(self, mocker):
    """An empty registry + an armed window + a fresh beacon → pairing offer."""
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=True)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=[])
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.5:8443", 2, "new-app"), {})
    assert (uri, kind) == ("ws://10.0.0.5:8443", "pairing_offer")

  def test_armed_window_offers_pairing_to_new_app_while_paired(self, mocker):
    """A device already paired to one app can pair ANOTHER during a window."""
    apps = [LocalApp(app_id="app-a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=True)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.9:8443", 1, "app-b"), {})
    assert (uri, kind) == ("ws://10.0.0.9:8443", "pairing_offer")

  def test_armed_window_never_dials_paired_app_beacon(self, mocker):
    """While armed, a beacon from an already-paired app is NOT a pairing
    offer (paired endpoints are skipped too — the window targets the NEW app,
    so the loop waits rather than re-dialing the existing app)."""
    apps = [LocalApp(app_id="app-a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=True)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.2:8443", 1, "app-a"), {})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")

  def test_stale_beacon_ignored(self, mocker):
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=True)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=[])
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.5:8443", 9999, "new-app"), {})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")

  def test_cloud_when_nothing_local(self, mocker):
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=[])
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery(None, None), {})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")

  def test_paired_device_ignores_discovery_without_window(self, mocker):
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    uri, kind = sunnylinkd._pick_ws_uri(DummyDiscovery("ws://10.0.0.99:8443", 1, "new-app"), {})
    assert (uri, kind) == ("ws://10.0.0.2:8443", "paired_local")

  def test_fresh_paired_beacon_preferred_over_stored_endpoint(self, mocker):
    """A paired app's FRESH beacon is the most current truth — its IP can
    change between networks, so selection dials the beacon address over the
    (possibly stale) endpoint stored at pairing time."""
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    disc = DummyDiscovery(paired_endpoint="ws://10.0.0.77:8443",
                          paired_app_id="a", paired_seen_ago=2)
    uri, kind = sunnylinkd._pick_ws_uri(disc, {})
    assert (uri, kind) == ("ws://10.0.0.77:8443", "paired_local")

  def test_stale_paired_beacon_ignored(self, mocker):
    """A paired-app beacon older than the freshness window no longer counts —
    fall back to the stored endpoint (the app may have left the network)."""
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    disc = DummyDiscovery(paired_endpoint="ws://10.0.0.77:8443",
                          paired_app_id="a", paired_seen_ago=9999)
    uri, kind = sunnylinkd._pick_ws_uri(disc, {})
    assert (uri, kind) == ("ws://10.0.0.2:8443", "paired_local")

  def test_fresh_paired_beacon_of_unknown_app_ignored(self, mocker):
    """A fresh beacon whose app_id is NOT in the registry is not a paired
    selection candidate (outside a window it is ignored entirely)."""
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    disc = DummyDiscovery(paired_endpoint="ws://10.0.0.77:8443",
                          paired_app_id="stranger", paired_seen_ago=2)
    uri, kind = sunnylinkd._pick_ws_uri(disc, {})
    assert (uri, kind) == ("ws://10.0.0.2:8443", "paired_local")

  def test_backoff_blocks_fresh_paired_beacon(self, mocker):
    """A fresh beacon endpoint that recently failed to dial stays backoff-
    blocked (same rule as stored endpoints)."""
    apps = [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443")]
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=apps)
    disc = DummyDiscovery(paired_endpoint="ws://10.0.0.77:8443",
                          paired_app_id="a", paired_seen_ago=2)
    future = time.monotonic() + 9999
    uri, kind = sunnylinkd._pick_ws_uri(disc, {"ws://10.0.0.77:8443": future})
    assert (uri, kind) == (sunnylinkd.SUNNYLINK_ATHENA_HOST, "cloud")


class TestPairedRefresh(OpenpilotTestCase):
  def _apps(self):
    return [LocalApp(app_id="a", endpoint="ws://10.0.0.2:8443", app_name="Pixel")]

  def _beacon(self, source_ip="10.0.0.77", app_id="a"):
    return AppBeacon(app_id=app_id, ws_port=8443, source_ip=source_ip)

  def test_closes_cloud_connection_and_clears_stale_backoffs(self, mocker):
    """On the cloud link, a paired app announcing a NEW endpoint forces a
    prompt re-selection (the loop re-picks and dials the fresh address)."""
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=self._apps())
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", None)  # on cloud
    mocker.patch.object(sunnylinkd, "_active_ws", FakePairingWs())
    sunnylinkd._pairing_in_progress.clear()
    backoffs = {"ws://10.0.0.2:8443": time.monotonic() + 9999}

    sunnylinkd._handle_paired_refresh(backoffs, self._beacon())

    assert sunnylinkd._active_ws.closed
    assert backoffs == {}, "the app's stale endpoint must not stay backoff-locked"

  def test_noop_when_already_on_fresh_endpoint(self, mocker):
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=self._apps())
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.77:8443")
    ws = FakePairingWs()
    mocker.patch.object(sunnylinkd, "_active_ws", ws)
    sunnylinkd._pairing_in_progress.clear()

    sunnylinkd._handle_paired_refresh({}, self._beacon())

    assert not ws.closed

  def test_noop_while_serving_another_local_app(self, mocker):
    """Connected to another app that is working: no forced switch (the natural
    reconnect cycle re-picks, and selection prefers the fresh beacon)."""
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=self._apps())
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", "ws://10.0.0.9:8443")
    ws = FakePairingWs()
    mocker.patch.object(sunnylinkd, "_active_ws", ws)
    sunnylinkd._pairing_in_progress.clear()

    sunnylinkd._handle_paired_refresh({}, self._beacon())

    assert not ws.closed

  def test_noop_during_pairing_session(self, mocker):
    """Never kill the connection the pairing code is being typed over."""
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=self._apps())
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", None)
    ws = FakePairingWs()
    mocker.patch.object(sunnylinkd, "_active_ws", ws)
    sunnylinkd._pairing_in_progress.set()
    try:
      sunnylinkd._handle_paired_refresh({}, self._beacon())
    finally:
      sunnylinkd._pairing_in_progress.clear()
    assert not ws.closed

  def test_ignores_unknown_app_beacon(self, mocker):
    mocker.patch.object(sunnylinkd, "get_local_apps", return_value=self._apps())
    mocker.patch.object(sunnylinkd, "_active_local_endpoint", None)
    ws = FakePairingWs()
    mocker.patch.object(sunnylinkd, "_active_ws", ws)
    sunnylinkd._pairing_in_progress.clear()

    sunnylinkd._handle_paired_refresh({}, self._beacon(app_id="stranger"))

    assert not ws.closed


class TestPairingWatchdog(OpenpilotTestCase):
  def test_closes_connection_and_clears_backoffs_when_armed(self, mocker):
    """Arming the window mid-session forces re-selection: the live connection
    is closed and local backoffs cleared so the loop dials the new app."""
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=True)
    ws = FakePairingWs()
    backoffs = {"ws://10.0.0.2:8443": time.monotonic() + 9999}
    stop = threading.Event()
    sunnylinkd._pairing_watchdog(ws, backoffs, stop, interval_s=0.01)
    assert ws.closed
    assert backoffs == {}

  def test_does_nothing_when_not_armed(self, mocker):
    mocker.patch.object(sunnylinkd, "pairing_requested", return_value=False)
    ws = FakePairingWs()
    backoffs = {"ws://10.0.0.2:8443": 1.0}
    stop = threading.Event()
    thread = threading.Thread(target=sunnylinkd._pairing_watchdog,
                              args=(ws, backoffs, stop), kwargs={"interval_s": 0.01})
    thread.start()
    time.sleep(0.05)
    assert not ws.closed
    assert backoffs == {"ws://10.0.0.2:8443": 1.0}
    stop.set()
    thread.join(timeout=2)


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
