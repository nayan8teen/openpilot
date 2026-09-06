"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import threading
import time

from openpilot.common.params import Params, params_put
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot.sunnylink.athena.local_pairing import (
  PAIRING_CODE_ALPHABET,
  PAIRING_CODE_KEY,
  PAIRING_CODE_LENGTH,
  PAIRING_REQUEST_KEY,
  PAIRING_WINDOW_S,
  LOCAL_APPS_KEY,
  LocalApp,
  PairingCodeRotator,
  add_local_app,
  arm_pairing,
  clear_pairing_request,
  generate_pairing_code,
  get_local_apps,
  get_pairing_code,
  is_locally_paired,
  local_app_display_name,
  local_identity,
  pairing_requested,
  read_pairing_code,
  remove_all_local_apps,
  remove_local_app,
  set_local_app_alias,
  update_local_app_endpoint,
  verify_pairing_code,
)


def _put_raw(params: Params, key: str, value: bytes) -> None:
  """Write raw bytes under a key, bypassing the Python JSON cast (simulates
  legacy STRING-era values or external corruption)."""
  params_put(params.p, key.encode(), value, len(value), True)

# Params keys these tests write (restored in teardown).
_WRITTEN_KEYS = (LOCAL_APPS_KEY, PAIRING_CODE_KEY, PAIRING_REQUEST_KEY, "DongleId", "HardwareSerial")


class TestPairingCode(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = {key: self.params.get(key) for key in _WRITTEN_KEYS}

  def teardown_method(self):
    for key, value in self.saved.items():
      if value is None:
        self.params.remove(key)
      else:
        self.params.put(key, value, block=True)

  def test_generate_pairing_code_format(self):
    for _ in range(50):
      code = generate_pairing_code()
      assert len(code) == PAIRING_CODE_LENGTH
      assert code.isdigit(), "pairing code must be numeric (the app accepts digits only)"
      assert all(c in PAIRING_CODE_ALPHABET for c in code)

  def test_get_pairing_code_creates_and_persists(self):
    self.params.remove(PAIRING_CODE_KEY)
    code = get_pairing_code(self.params)
    assert len(code) == PAIRING_CODE_LENGTH
    doc = self.params.get(PAIRING_CODE_KEY)
    assert doc["code"] == code
    assert isinstance(doc.get("ts"), int)  # armed-at timestamp drives the window
    assert read_pairing_code(self.params) == code
    # Stable across reads until rotated.
    assert get_pairing_code(self.params) == code

  def test_read_pairing_code_handles_missing_and_wrong_shape(self):
    self.params.remove(PAIRING_CODE_KEY)
    assert read_pairing_code(self.params) is None
    self.params.put(PAIRING_CODE_KEY, {"not_code": 1}, block=True)
    assert read_pairing_code(self.params) is None

  def test_verify_pairing_code(self):
    code = get_pairing_code(self.params)
    assert verify_pairing_code(code, self.params)
    assert verify_pairing_code(code.lower(), self.params)  # user input normalized
    assert not verify_pairing_code("XXXXXX", self.params)

  def test_verify_pairing_code_none_stored(self):
    self.params.remove(PAIRING_CODE_KEY)
    assert not verify_pairing_code("ABCDEF", self.params)

  def test_local_identity_prefers_dongle_id(self):
    self.params.put("DongleId", "comma-dongle", block=True)
    self.params.put("HardwareSerial", "serial-1", block=True)
    assert local_identity(self.params) == "comma-dongle"
    self.params.remove("DongleId")
    assert local_identity(self.params) == "serial-1"


class TestLocalAppsRegistry(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = {key: self.params.get(key) for key in (LOCAL_APPS_KEY,)}

  def teardown_method(self):
    for key, value in self.saved.items():
      if value is None:
        self.params.remove(key)
      else:
        self.params.put(key, value, block=True)

  def app(self, app_id="app-1", endpoint="ws://10.0.0.5:8443"):
    return LocalApp(app_id=app_id, endpoint=endpoint, app_name="Pixel")

  def test_add_list_dedupe(self):
    assert get_local_apps(self.params) == []
    add_local_app(self.app(), self.params)
    add_local_app(LocalApp(app_id="app-2", endpoint="ws://10.0.0.6:8443", app_name="iPad"), self.params)
    apps = get_local_apps(self.params)
    assert [a.app_id for a in apps] == ["app-1", "app-2"]

    # Re-pairing the same app updates in place (no duplicates; the updated
    # entry moves to the end, matching the "most recent pairing wins" order).
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.9:8443", app_name="Pixel 9"), self.params)
    apps = get_local_apps(self.params)
    assert len(apps) == 2
    updated = next(a for a in apps if a.app_id == "app-1")
    assert updated.endpoint == "ws://10.0.0.9:8443"
    assert updated.app_name == "Pixel 9"
    assert updated.paired_at > 0

  def test_is_locally_paired(self):
    assert not is_locally_paired(self.params)
    add_local_app(self.app(), self.params)
    assert is_locally_paired(self.params)

  def test_remove_local_app(self):
    add_local_app(self.app("a"), self.params)
    add_local_app(self.app("b"), self.params)
    assert remove_local_app("a", self.params)
    apps = get_local_apps(self.params)
    assert [a.app_id for a in apps] == ["b"]
    assert not remove_local_app("missing", self.params)

  def test_update_local_app_endpoint(self):
    """A paired app's beacon refreshes its cached endpoint (the app's IP can
    change between networks) without touching identity fields."""
    add_local_app(self.app(), self.params)
    assert update_local_app_endpoint("app-1", "ws://10.0.0.99:8443", self.params)
    apps = get_local_apps(self.params)
    assert len(apps) == 1
    assert apps[0].endpoint == "ws://10.0.0.99:8443"
    assert apps[0].app_name == "Pixel"  # preserved
    assert apps[0].paired_at > 0        # preserved

  def test_update_local_app_endpoint_preserves_alias(self):
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443",
                           app_name="Pixel", alias="My Phone"), self.params)
    assert update_local_app_endpoint("app-1", "ws://10.0.0.99:8443", self.params)
    apps = get_local_apps(self.params)
    assert apps[0].alias == "My Phone"

  def test_alias_round_trip(self):
    """The app-set alias is persisted with the rest of the app's details."""
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443",
                           app_name="Pixel", alias="My Phone"), self.params)
    apps = get_local_apps(self.params)
    assert apps[0].alias == "My Phone"
    # from_dict round-trip via the stored document.
    raw = self.params.get(LOCAL_APPS_KEY)
    assert raw[0]["alias"] == "My Phone"
    assert raw[0]["app_name"] == "Pixel"

  def test_set_local_app_alias_updates_in_place(self):
    add_local_app(self.app(), self.params)
    assert set_local_app_alias("app-1", "My Pixel", self.params)
    apps = get_local_apps(self.params)
    assert apps[0].alias == "My Pixel"
    assert apps[0].app_name == "Pixel"  # untouched
    # Same value again → no change.
    assert not set_local_app_alias("app-1", "My Pixel", self.params)

  def test_set_local_app_alias_unknown_app_noop(self):
    add_local_app(self.app(), self.params)
    assert not set_local_app_alias("stranger", "X", self.params)
    apps = get_local_apps(self.params)
    assert len(apps) == 1
    assert apps[0].alias == ""

  def test_local_app_display_name_precedence(self):
    """The device UI label: alias → app_name → app_id."""
    assert local_app_display_name(LocalApp(app_id="a", endpoint="", app_name="Pixel", alias="My Phone")) == "My Phone"
    assert local_app_display_name(LocalApp(app_id="a", endpoint="", app_name="Pixel")) == "Pixel"
    assert local_app_display_name(LocalApp(app_id="a", endpoint="")) == "a"
    # Legacy entries (no alias) fall back exactly as before.
    assert local_app_display_name(LocalApp(app_id="a", endpoint="", app_name="sunnylink mobile")) == "sunnylink mobile"

  def test_update_local_app_endpoint_no_change_or_unknown(self):
    add_local_app(self.app(), self.params)
    # Same endpoint → not a change.
    assert not update_local_app_endpoint("app-1", "ws://10.0.0.5:8443", self.params)
    # Unknown app → never creates an entry.
    assert not update_local_app_endpoint("stranger", "ws://10.0.0.9:8443", self.params)
    apps = get_local_apps(self.params)
    assert len(apps) == 1
    assert apps[0].endpoint == "ws://10.0.0.5:8443"

  def test_remove_all(self):
    add_local_app(self.app("a"), self.params)
    remove_all_local_apps(self.params)
    assert get_local_apps(self.params) == []
    assert not is_locally_paired(self.params)

  def test_ignores_corrupt_registry(self):
    # Legacy STRING-era value (raw JSON text, not a JSON document) or external
    # corruption — must not crash, treated as unpaired.
    _put_raw(self.params, LOCAL_APPS_KEY, b"not-json{")
    assert get_local_apps(self.params) == []
    assert not is_locally_paired(self.params)

  def test_ignores_non_list_registry(self):
    self.params.put(LOCAL_APPS_KEY, {"not": "a list"}, block=True)
    assert get_local_apps(self.params) == []
    assert not is_locally_paired(self.params)


class TestPairingWindow(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = {key: self.params.get(key) for key in (PAIRING_REQUEST_KEY, PAIRING_CODE_KEY)}

  def teardown_method(self):
    for key, value in self.saved.items():
      if value is None:
        self.params.remove(key)
      else:
        self.params.put(key, value, block=True)

  def test_arm_pairing_sets_flag_and_fresh_code(self):
    code = arm_pairing(self.params)
    assert len(code) == PAIRING_CODE_LENGTH
    assert self.params.get_bool(PAIRING_REQUEST_KEY)
    assert read_pairing_code(self.params) == code
    assert pairing_requested(self.params)

  def test_arm_pairing_rerolls_code_each_time(self):
    first = arm_pairing(self.params)
    second = arm_pairing(self.params)
    assert pairing_requested(self.params)
    assert read_pairing_code(self.params) == second
    assert second != first, "a re-armed window always rolls a fresh code"

  def test_clear_pairing_request_drops_flag_and_code(self):
    arm_pairing(self.params)
    clear_pairing_request(self.params)
    assert not pairing_requested(self.params)
    assert self.params.get(PAIRING_CODE_KEY) is None

  def test_requested_false_without_flag(self):
    assert not pairing_requested(self.params)

  def test_requested_ignores_flag_without_valid_code(self):
    self.params.put_bool(PAIRING_REQUEST_KEY, True, block=True)
    assert not pairing_requested(self.params)
    assert not self.params.get_bool(PAIRING_REQUEST_KEY)  # self-cleared

  def test_window_expires_and_self_clears(self):
    arm_pairing(self.params)
    # Age the code's armed-at timestamp beyond the window.
    self.params.put(PAIRING_CODE_KEY,
                    {"code": "ABC123", "ts": int(time.time()) - PAIRING_WINDOW_S - 1},  # noqa: TID251
                    block=True)
    assert not pairing_requested(self.params)
    assert not self.params.get_bool(PAIRING_REQUEST_KEY)
    assert self.params.get(PAIRING_CODE_KEY) is None


class TestPairingCodeRotator(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = {key: self.params.get(key) for key in (PAIRING_REQUEST_KEY, PAIRING_CODE_KEY)}

  def teardown_method(self):
    for key, value in self.saved.items():
      if value is None:
        self.params.remove(key)
      else:
        self.params.put(key, value, block=True)

  def test_rotate_generates_code_while_window_armed(self):
    arm_pairing(self.params)
    rotator = PairingCodeRotator(self.params)
    rotator.rotate()
    assert read_pairing_code(self.params) is not None
    assert len(read_pairing_code(self.params)) == PAIRING_CODE_LENGTH
    assert pairing_requested(self.params)

  def test_rotate_rerolls_code_while_armed(self):
    arm_pairing(self.params)
    first = read_pairing_code(self.params)
    rotator = PairingCodeRotator(self.params)
    rotator.rotate()
    second = read_pairing_code(self.params)
    assert second is not None and second != first

  def test_rotate_clears_code_when_window_closed(self):
    arm_pairing(self.params)
    clear_pairing_request(self.params)
    rotator = PairingCodeRotator(self.params)
    rotator.rotate()
    assert self.params.get(PAIRING_CODE_KEY) is None

  def test_thread_rotates_on_interval(self):
    arm_pairing(self.params)
    stop = threading.Event()
    rotator = PairingCodeRotator(self.params, rotation_s=0.05, stop_event=stop)
    rotator.start()
    try:
      first = None
      deadline = time.monotonic() + 3
      seen: set[str] = set()
      while time.monotonic() < deadline:
        code = read_pairing_code(self.params)
        if code is not None:
          seen.add(code)
          first = first or code
        if len(seen) >= 2:
          break
        time.sleep(0.02)
      assert first is not None, "rotator never wrote a pairing code"
      assert len(seen) >= 2, f"code did not rotate (seen {len(seen)} unique)"
    finally:
      stop.set()
      rotator.join(timeout=2)
