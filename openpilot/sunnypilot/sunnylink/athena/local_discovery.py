"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.sunnylink.athena.local_pairing import (
  BEACON_PREFIX,
  SUNNYLINK_LOCAL_UDP_PORT,
  format_endpoint,
  is_locally_paired,
)


@dataclass
class AppBeacon:
  """A parsed app beacon — the app announcing it is acting as the local backend."""
  app_id: str
  ws_port: int
  source_ip: str

  @property
  def endpoint(self) -> str:
    return format_endpoint(self.source_ip, self.ws_port)


def parse_beacon(raw: str | bytes, source_ip: str = "") -> AppBeacon | None:
  """
  Parse one UDP beacon line from the app.

  Wire format: `SUNNYLINK1 {"v":1,"role":"app","app_id":"<uuid>","ws_port":8443}`

  Returns None for anything that is not a well-formed v1 app beacon (device
  beacons from other participants, garbage, other protocols). Discovery is
  unauthenticated phonebook data — ids + addresses only, no secrets.
  """
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", errors="replace")
  raw = raw.strip()
  if not raw.startswith(BEACON_PREFIX + " "):
    return None
  try:
    data = json.loads(raw[len(BEACON_PREFIX) + 1:])
  except ValueError:
    return None
  if not isinstance(data, dict):
    return None
  if data.get("role") != "app" or data.get("v") != 1:
    return None
  app_id = data.get("app_id")
  ws_port = data.get("ws_port")
  if not isinstance(app_id, str) or not app_id:
    return None
  if not isinstance(ws_port, int) or not (0 < ws_port <= 65535):
    return None
  return AppBeacon(app_id=app_id, ws_port=ws_port, source_ip=source_ip)


class LocalDiscovery(threading.Thread):
  """
  The device-side half of LAN discovery: a passive stdlib UDP listener.

  The APP drives discovery — it periodically broadcasts its beacon to
  <broadcast>:53133/udp. This thread listens on the same port and remembers the
  most recent app endpoint heard:

  - Unpaired device: [latest_endpoint] is exposed so sunnylinkd can dial the
    app and run the pairing handshake (the app then asks the user for the code
    displayed on this device's screen).
  - Paired device: beacons are IGNORED. The app endpoint is pinned at pairing
    time (`SunnylinkLocalApps`) and a random LAN beacon must never redirect a
    paired device.

  No new dependencies: stdlib `socket` only.
  """

  def __init__(self, params: Params | None = None, port: int = SUNNYLINK_LOCAL_UDP_PORT,
               sock: socket.socket | None = None):
    super().__init__(name="local_discovery_listener", daemon=True)
    self.params = params or Params()
    self.port = port
    # Test seam: inject a bound UDP socket. None → bind the fixed LAN port.
    self._sock = sock
    self._latest_endpoint: str | None = None
    self._last_seen_monotonic: float = 0.0
    self._lock = threading.Lock()
    self._stop_event = threading.Event()

  def stop(self) -> None:
    self._stop_event.set()
    if self._sock is not None:
      try:
        self._sock.close()
      except OSError:
        pass

  def latest_endpoint(self) -> str | None:
    """The most recently announced app endpoint (None while paired or nothing heard)."""
    with self._lock:
      return self._latest_endpoint

  def last_seen_ago(self) -> float | None:
    """Seconds since the last app beacon was heard (None when none heard yet)."""
    with self._lock:
      if self._last_seen_monotonic == 0.0:
        return None
      return time.monotonic() - self._last_seen_monotonic

  def _handle(self, raw: bytes, source_ip: str) -> None:
    if is_locally_paired(self.params):
      # Paired devices pin the endpoint from pairing — never a random beacon.
      return
    beacon = parse_beacon(raw, source_ip)
    if beacon is None:
      return
    with self._lock:
      self._latest_endpoint = beacon.endpoint
      self._last_seen_monotonic = time.monotonic()
    cloudlog.debug(f"local_discovery.app_found {beacon.app_id} at {beacon.endpoint}")

  def _bind(self) -> socket.socket:
    if self._sock is not None:
      return self._sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", self.port))
    sock.settimeout(0.5)
    return sock

  def run(self) -> None:
    sock = self._bind()
    try:
      while not self._stop_event.is_set():
        try:
          data, addr = sock.recvfrom(4096)
          self._handle(data, addr[0] if len(addr) > 0 else "")
        except TimeoutError:
          continue
        except OSError:
          # Socket closed by stop() — exit quietly.
          if self._stop_event.is_set():
            break
          cloudlog.exception("local_discovery.recv.exception")
          break
    finally:
      try:
        sock.close()
      except OSError:
        pass
