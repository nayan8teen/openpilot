"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from __future__ import annotations

import secrets
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, UTC
from typing import Any
from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

# Fixed LAN ports shared with the sunnylink mobile app (the app is the local
# "backend": it broadcasts UDP beacons and runs the WebSocket server the device
# dials). Do not change without a coordinated app update.
SUNNYLINK_LOCAL_UDP_PORT = 53133
SUNNYLINK_LOCAL_WS_PORT = 8443

LOCAL_APPS_KEY = "SunnylinkLocalApps"
PAIRING_CODE_KEY = "SunnylinkLocalPairingCode"
# Set True by the on-device "Pair App" button while a pairing window is armed.
# Discovery, the pairing code, and the pairing-offer dial run ONLY inside this
# window (pairing is an explicit device-side action, never an auto-offer).
PAIRING_REQUEST_KEY = "SunnylinkLocalPairingRequest"
# Status written by the discovery listener (most recent app beacon) so the
# on-device settings UI can show "app discovered" across processes.
DISCOVERED_APP_KEY = "SunnylinkLocalDiscoveredApp"

# 6 NUMERIC digits — the mobile app's pairing-code field accepts digits only
# (it filters non-digits and requires exactly 6, see LocalModeScreen). The
# alphabet must stay numeric; 10^6 combinations over the 5-minute window is
# plenty for a LAN pairing handshake.
PAIRING_CODE_LENGTH = 6
PAIRING_CODE_ALPHABET = "0123456789"
DEFAULT_CODE_ROTATION_S = 10 * 60  # re-roll the displayed code every 10 min
# How long an armed pairing window stays open before it self-expires (and the
# request flag is dropped). The device-side "Pair App" action is a deliberate
# short-lived window; after it lapses the device returns to its normal
# connection selection (paired local endpoints / cloud).
PAIRING_WINDOW_S = 5 * 60

# Beacons carry the app's identity + WS port. The device itself never
# broadcasts — discovery lives on the app side (it announces, we listen).
BEACON_PREFIX = "SUNNYLINK1"


@dataclass
class LocalApp:
  """One app paired with this device (the app runs the local "backend")."""
  app_id: str
  endpoint: str
  app_name: str = ""
  paired_at: int = 0  # epoch seconds

  @staticmethod
  def from_dict(data: dict[str, Any]) -> LocalApp:
    return LocalApp(
      app_id=str(data.get("app_id", "")),
      endpoint=str(data.get("endpoint", "")),
      app_name=str(data.get("app_name", "")),
      paired_at=int(data.get("paired_at") or 0),
    )


def is_locally_paired(params: Params | None = None) -> bool:
  """True when at least one app is paired (local mode is available)."""
  return len(get_local_apps(params)) > 0


def get_local_apps(params: Params | None = None) -> list[LocalApp]:
  """The paired-app registry (a JSON list persisted in `SunnylinkLocalApps`)."""
  params = params or Params()
  data = params.get(LOCAL_APPS_KEY)
  if not isinstance(data, list):
    return []
  return [LocalApp.from_dict(item) for item in data if isinstance(item, dict) and item.get("app_id")]


def _save_local_apps(apps: list[LocalApp], params: Params | None = None) -> None:
  params = params or Params()
  if apps:
    params.put(LOCAL_APPS_KEY, [asdict(app) for app in apps], block=True)
  else:
    params.remove(LOCAL_APPS_KEY)


def add_local_app(app: LocalApp, params: Params | None = None) -> None:
  """Pair an app: append (or update by app_id) and persist."""
  if not app.paired_at:
    app.paired_at = int(datetime.now(UTC).replace(tzinfo=None).timestamp())
  apps = [existing for existing in get_local_apps(params) if existing.app_id != app.app_id]
  apps.append(app)
  _save_local_apps(apps, params)
  cloudlog.event("local_pairing.app_paired", app_id=app.app_id, endpoint=app.endpoint)


def remove_local_app(app_id: str, params: Params | None = None) -> bool:
  """Unpair an app by id. Returns True when an app was removed."""
  apps = get_local_apps(params)
  remaining = [app for app in apps if app.app_id != app_id]
  if len(remaining) == len(apps):
    return False
  _save_local_apps(remaining, params)
  cloudlog.event("local_pairing.app_unpaired", app_id=app_id)
  return True


def remove_all_local_apps(params: Params | None = None) -> None:
  """Unpair every app (the device-side "forget local apps" action)."""
  _save_local_apps([], params)
  cloudlog.event("local_pairing.all_apps_unpaired")


def generate_pairing_code() -> str:
  """A 6-digit numeric pairing code (the mobile app's code field is digits-only)."""
  return "".join(secrets.choice(PAIRING_CODE_ALPHABET) for _ in range(PAIRING_CODE_LENGTH))


def _write_pairing_code(code: str, params: Params) -> None:
  """Persist the code together with its armed-at timestamp (the pairing window
  is derived from that timestamp, so the code and window always agree)."""
  params.put(PAIRING_CODE_KEY, {"code": code, "ts": int(time.time())}, block=True)  # noqa: TID251


def read_pairing_code(params: Params | None = None) -> str | None:
  """The stored pairing code, or None when cleared / not yet generated."""
  params = params or Params()
  data = params.get(PAIRING_CODE_KEY)
  if not isinstance(data, dict):
    return None
  code = data.get("code")
  return str(code) if code else None


def get_pairing_code(params: Params | None = None) -> str:
  """The current displayed pairing code, generating (and persisting) one on
  first use. A code is only meaningful inside a pairing window — `arm_pairing`
  is the button-driven path that always rolls a fresh one."""
  params = params or Params()
  code = read_pairing_code(params)
  if code is None:
    code = generate_pairing_code()
    _write_pairing_code(code, params)
  return code


def pairing_requested(params: Params | None = None) -> bool:
  """
  True while the on-device "Pair App" window is armed and still fresh.

  The window is self-expiring: when the request flag is set but the pairing
  code (which carries the armed-at timestamp) is missing or older than
  [PAIRING_WINDOW_S], the flag is dropped and False is returned. Every caller
  (the discovery listener, sunnylinkd's connection selection, the settings UI
  row visibility) consults this, so the window closes itself wherever it is
  next read.
  """
  params = params or Params()
  if not params.get_bool(PAIRING_REQUEST_KEY):
    return False
  data = params.get(PAIRING_CODE_KEY)
  ts = data.get("ts") if isinstance(data, dict) else None
  if not isinstance(ts, (int, float)) or time.time() - ts > PAIRING_WINDOW_S:  # noqa: TID251
    clear_pairing_request(params)
    return False
  return True


def arm_pairing(params: Params | None = None) -> str:
  """
  Arm a pairing window and return the code the user types into the app.

  Always rolls a fresh code (and a fresh window timestamp); the flag is set
  only after the code is persisted, so `pairing_requested` never observes an
  armed flag without a valid code. Re-arming after a timeout or an unpair just
  starts a new window.
  """
  params = params or Params()
  code = generate_pairing_code()
  _write_pairing_code(code, params)
  params.put_bool(PAIRING_REQUEST_KEY, True, block=True)
  return code


def clear_pairing_request(params: Params | None = None) -> None:
  """Close the pairing window: drop the request flag and the code together."""
  params = params or Params()
  params.remove(PAIRING_REQUEST_KEY)
  params.remove(PAIRING_CODE_KEY)


def verify_pairing_code(code: str, params: Params | None = None) -> bool:
  """Constant-time check of a code typed into the app against the displayed one."""
  params = params or Params()
  current = read_pairing_code(params)
  if current is None:
    return False
  return secrets.compare_digest(str(code).strip().upper(), current)


class PairingCodeRotator(threading.Thread):
  """
  Keeps the on-screen pairing code fresh while a pairing window is armed.

  - Immediately on start, and every [rotation_s] after: re-roll the code while
    a pairing window is armed (fresh code + fresh window timestamp).
  - No window armed: the code is cleared and the thread idles. Pairing is an
    explicit device-side action (the "Pair App" button arms the window), so
    the code is never generated on its own.
  """

  def __init__(self, params: Params | None = None, rotation_s: float = DEFAULT_CODE_ROTATION_S,
               stop_event: threading.Event | None = None, tick_cb: Callable[[], None] | None = None):
    super().__init__(name="local_pairing_code_rotator", daemon=True)
    self.params = params or Params()
    self.rotation_s = rotation_s
    self.stop_event = stop_event or threading.Event()
    # Test seam: invoked once per loop iteration after state is updated.
    self.tick_cb = tick_cb

  def rotate(self) -> None:
    """One rotation pass: re-roll the code while the window is armed, clear it
    (and the window, if it expired) otherwise."""
    if pairing_requested(self.params):
      _write_pairing_code(generate_pairing_code(), self.params)
    else:
      self.params.remove(PAIRING_CODE_KEY)

  def run(self) -> None:
    self.rotate()
    while not self.stop_event.wait(self.rotation_s):
      try:
        self.rotate()
        if self.tick_cb is not None:
          self.tick_cb()
      except Exception:
        cloudlog.exception("local_pairing.code_rotator.exception")


def format_endpoint(host: str, ws_port: int = SUNNYLINK_LOCAL_WS_PORT) -> str:
  return f"ws://{host}:{ws_port}"


def local_identity(params: Params | None = None) -> str:
  """
  Identity claim the device presents on LOCAL connections (the app's WebSocket
  server). The comma `DongleId` is used — it always exists on comma hardware
  (unlike `SunnylinkDongleId`, which is "UnregisteredDevice" until cloud
  registration) and it is what the mobile app matches against the backend
  device list (`comma_dongle_id`) to dedupe cloud + local entries.
  """
  params = params or Params()
  return params.get("DongleId") or params.get("HardwareSerial") or ""
