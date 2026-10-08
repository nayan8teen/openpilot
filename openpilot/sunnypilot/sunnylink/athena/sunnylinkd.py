#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from __future__ import annotations

import base64
import errno
import gzip
import json
import os
import ssl
import threading
import time

from collections.abc import Callable
from functools import partial

from openpilot.system.athena.rpc import dispatcher
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.realtime import set_core_affinity
from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware.hw import Paths
from openpilot.system.athena.athenad import ws_send, jsonrpc_handler, \
  recv_queue, UploadQueueCache, upload_queue, cur_upload_items, backoff, ws_manage, log_handler, start_local_proxy_shim, upload_handler, stat_handler
from websocket import (ABNF, WebSocket, WebSocketException, WebSocketTimeoutException,
                       create_connection, WebSocketConnectionClosedException)

import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot.models.model_name import DEFAULT_MODEL, DEFAULT_BIG_MODEL
from openpilot.sunnypilot.selfdrive.car.sync_sunnylink_params import update_car_list_param
from openpilot.sunnypilot.sunnylink.api import SunnylinkApi
from openpilot.sunnypilot.sunnylink.utils import sunnylink_ready, sunnylink_need_register, get_param_as_byte, save_param_from_base64_encoded_string
from openpilot.sunnypilot.sunnylink.capabilities import generate_capabilities, CAPABILITY_LABELS
from openpilot.sunnypilot.sunnylink.tools.generate_settings_schema import generate_schema
from openpilot.sunnypilot.sunnylink.athena.local_params import cloud_may_write, is_local_param

SUNNYLINK_ATHENA_HOST = os.getenv('SUNNYLINK_ATHENA_HOST', 'wss://athena.sunnylink.ai')
HANDLER_THREADS = int(os.getenv('HANDLER_THREADS', "4"))
SUNNYLINK_LOG_ATTR_NAME = "user.sunny.upload"
SUNNYLINK_RECONNECT_TIMEOUT_S = 70  # FYI changing this will also would require a change on sidebar.cc

# Parameters that should never be remotely modified
BLOCKED_PARAMS = {
  "AdbEnabled",
  "CompletedSunnylinkConsentVersion",
  "CompletedTrainingVersion",
  "GithubUsername",  # Could grant SSH access
  "GithubSshKeys",   # Direct SSH key injection
  "HasAcceptedTerms",
  "HasAcceptedTermsSP",
  "OnroadCycleRequested",      # Prevent remote cycle trigger
  "ParamsVersion",         # Device-managed version counter
}


class Sunnylinkd:
  """The cloud half of sunnylink: one long-poll athena connection serving the settings RPCs.

  The LAN half is `localsunnylinkd.LocalSunnylinkd`, which subclasses this. Nothing here knows
  about local endpoints, so the cloud link and a local session run side by side.
  """

  RPC_METHODS = ("toggleLogUpload", "getParamsAllKeys", "getParamsMetadata", "getParams", "saveParams", "startLocalProxy")

  def __init__(self, params: Params | None = None):
    self.params = params or Params()
    self.disallow_log_upload = threading.Event()
    self.active_ws: WebSocket | None = None

  def methods(self) -> dict[str, Callable[..., object]]:
    """The RPC surface this daemon serves, keyed by wire method name."""
    return {name: getattr(self, name) for name in self.RPC_METHODS}

  def toggleLogUpload(self, enabled: bool):
    if enabled and self.disallow_log_upload.is_set():
      self.disallow_log_upload.clear()
    else:
      self.disallow_log_upload.set()

  def getParamsAllKeys(self) -> list[str]:
    keys = [k.decode('utf-8') for k in self.params.all_keys()]
    return [key for key in keys if not is_local_param(key)]

  def getParamsMetadata(self) -> str:
    """settings_ui.json + live capabilities as gzip-compressed, base64-encoded string."""
    try:
      schema = generate_schema()
      schema["capabilities"] = generate_capabilities()
      schema["capability_labels"] = CAPABILITY_LABELS
      schema["default_model"] = DEFAULT_MODEL
      schema["default_big_model"] = DEFAULT_BIG_MODEL
      schema["chestnut_active"] = self.params.get_bool("ChestnutActive")
      raw = json.dumps(schema, separators=(",", ":")).encode("utf-8")
      return base64.b64encode(gzip.compress(raw)).decode("utf-8")
    except Exception:
      cloudlog.exception("sunnylinkd.getParamsMetadata.exception")
      raise

  def getParams(self, params_keys: list[str], compression: bool = False) -> str | dict[str, str]:
    available_keys = {k.decode('utf-8') for k in self.params.all_keys()}
    zero_values = {
      ParamKeyType.STRING.value: b"",
      ParamKeyType.BOOL.value: b"0",
      ParamKeyType.INT.value: b"0",
      ParamKeyType.FLOAT.value: b"0.0",
      ParamKeyType.TIME.value: b"",
      ParamKeyType.JSON.value: b"{}",
      ParamKeyType.BYTES.value: b"",
    }

    try:
      entries = []
      for key in (key for key in params_keys if key in available_keys and not is_local_param(key)):
        value = get_param_as_byte(key)
        if value is None:
          value = get_param_as_byte(key, get_default=True)
        if value is None:
          value = zero_values.get(self.params.get_type(key).value, b"")

        entries.append({
          "key": key,
          "value": base64.b64encode(gzip.compress(value) if compression else value).decode('utf-8'),
          "type": int(self.params.get_type(key).value),
          "is_compressed": compression,
        })

      response = {str(entry.get('key')): str(entry.get('value')) for entry in entries}
      response |= {"params": json.dumps(entries)}  # Upcoming for settings v1
      return response
    except Exception as e:
      cloudlog.exception("sunnylinkd.getParams.exception", e)
      raise

  def saveParams(self, params_to_update: dict[str, str], compression: bool = False) -> None:
    for key, value in params_to_update.items():
      if key in BLOCKED_PARAMS or not cloud_may_write(key):
        cloudlog.warning(f"sunnylinkd.saveParams.blocked: Attempted to modify blocked parameter '{key}'")
        continue

      try:
        save_param_from_base64_encoded_string(key, value, compression)
      except Exception as e:
        cloudlog.error(f"sunnylinkd.saveParams.exception {e}")

    # Increment version counter for frontend change detection
    try:
      current = int(self.params.get("ParamsVersion") or "0")
      self.params.put("ParamsVersion", str(current + 1), block=True)
    except Exception:
      pass

  def startLocalProxy(self, global_end_event: threading.Event, remote_ws_uri: str, local_port: int) -> dict[str, int]:
    sunnylink_dongle_id = self.params.get("SunnylinkDongleId")
    sunnylink_api = SunnylinkApi(sunnylink_dongle_id)

    cloudlog.debug("athena.startLocalProxy.starting")
    ws = create_connection(
      remote_ws_uri, header={"Authorization": f"Bearer {sunnylink_api.get_token()}"}, enable_multithread=True, sslopt={"cert_reqs": ssl.CERT_NONE}
    )

    return start_local_proxy_shim(global_end_event, local_port, ws)

  def ws_recv(self, ws: WebSocket, end_event: threading.Event) -> None:
    last_ping = int(time.monotonic() * 1e9)
    while not end_event.is_set():
      try:
        opcode, data = ws.recv_data(control_frame=True)
        if opcode in (ABNF.OPCODE_TEXT, ABNF.OPCODE_BINARY):
          if opcode == ABNF.OPCODE_TEXT:
            data = data.decode("utf-8")
          recv_queue.put_nowait(data)
          cloudlog.debug(f"sunnylinkd.ws_recv.recv {data}")
        elif opcode in (ABNF.OPCODE_PING, ABNF.OPCODE_PONG):
          cloudlog.debug("sunnylinkd.ws_recv.pong")
          last_ping = int(time.monotonic() * 1e9)
          Params().put("LastSunnylinkPingTime", last_ping, block=True)
      except WebSocketTimeoutException:
        ns_since_last_ping = int(time.monotonic() * 1e9) - last_ping
        if ns_since_last_ping > SUNNYLINK_RECONNECT_TIMEOUT_S * 1e9:
          cloudlog.warning("sunnylinkd.ws_recv.timeout")
          end_event.set()
      except Exception as e:
        if isinstance(e, WebSocketConnectionClosedException):
          cloudlog.warning(f"sunnylinkd.ws_recv.{type(e).__name__}")
        else:
          cloudlog.exception("sunnylinkd.ws_recv.exception")
        end_event.set()

  def ws_ping(self, ws: WebSocket, end_event: threading.Event) -> None:
    ws.ping()  # Send the first ping
    while not end_event.wait(SUNNYLINK_RECONNECT_TIMEOUT_S * 0.7):  # Sleep about 70% before a timeout
      try:
        ws.ping()
        cloudlog.debug("sunnylinkd.ws_recv.ws_ping: Pinging")
      except Exception:
        cloudlog.exception("sunnylinkd.ws_ping.exception")
        end_event.set()
    cloudlog.debug("sunnylinkd.ws_ping.end_event is set, exiting ws_ping thread")

  def sunny_log_handler(self, end_event: threading.Event, cellular_end_event: threading.Event) -> None:
    while not end_event.wait(0.1):
      if not cellular_end_event.is_set():
        log_handler(cellular_end_event, SUNNYLINK_LOG_ATTR_NAME)
    cellular_end_event.set()

  def serviceable(self) -> bool:
    """Keep the cloud link up while sunnylink is on, including before registration."""
    return self.params.get_bool("SunnylinkEnabled") and not self.params.get_bool("SunnylinkTempFault")

  def handle_long_poll(self, ws: WebSocket, exit_event: threading.Event | None) -> None:
    cloudlog.info("sunnylinkd.handle_long_poll started")
    sm = messaging.SubMaster(['deviceState'])
    end_event = threading.Event()
    cellular_end_event = threading.Event()

    threads = [
                threading.Thread(target=ws_manage, args=(ws, end_event), name='ws_manage'),
                threading.Thread(target=self.ws_recv, args=(ws, end_event), name='ws_recv'),
                threading.Thread(target=ws_send, args=(ws, end_event), name='ws_send'),
                threading.Thread(target=self.ws_ping, args=(ws, end_event), name='ws_ping'),
                threading.Thread(target=upload_handler, args=(end_event,), name='upload_handler'),
                threading.Thread(target=self.sunny_log_handler, args=(end_event, cellular_end_event), name='log_handler'),
                threading.Thread(target=stat_handler, args=(end_event, Paths.stats_sp_root(), True), name='stat_handler'),
              ] + [
                threading.Thread(target=jsonrpc_handler, args=(end_event, partial(self.startLocalProxy, end_event),), name=f'worker_{x}')
                for x in range(HANDLER_THREADS)
              ]

    for thread in threads:
      thread.start()
    try:
      while not end_event.wait(0.1):
        if not sunnylink_ready(self.params):
          cloudlog.warning("Exiting sunnylinkd.handle_long_poll as sunnylink is not ready")
          break

        sm.update(0)
        if exit_event is not None and exit_event.is_set():
          end_event.set()
          cellular_end_event.set()

        prime_type = self.params.get("PrimeType") or 0
        metered = sm['deviceState'].networkMetered

        if self.disallow_log_upload.is_set() and not cellular_end_event.is_set():
          cloudlog.debug("sunnylinkd.handle_long_poll: DISALLOW_LOG_UPLOAD, setting comma_prime_cellular_end_event")
          cellular_end_event.set()
        elif metered and int(prime_type) > 2:
          cloudlog.debug(f"sunnylinkd.handle_long_poll: PrimeType({prime_type}) > 2 and networkMetered({metered})")
          cellular_end_event.set()
        elif cellular_end_event.is_set() and not self.disallow_log_upload.is_set():
          cloudlog.debug(
            f"sunnylinkd.handle_long_poll: comma_prime_cellular_end_event is set and not PrimeType({prime_type}) > 2 or not networkMetered({metered})")
          cellular_end_event.clear()
    finally:
      end_event.set()
      cellular_end_event.set()
      for thread in threads:
        cloudlog.debug(f"sunnylinkd athena.joining {thread.name}")
        thread.join()
        cloudlog.debug(f"sunnylinkd athena.joined {thread.name}")

  def connect(self) -> WebSocket:
    api = SunnylinkApi(self.params.get("SunnylinkDongleId"))
    return create_connection(
      SUNNYLINK_ATHENA_HOST,
      header={"Authorization": f"Bearer {api.get_token()}"},
      enable_multithread=True,
      # A locally hosted athena skips PKIX; the real cloud host is verified.
      sslopt={"cert_reqs": ssl.CERT_NONE if "localhost" in SUNNYLINK_ATHENA_HOST else ssl.CERT_REQUIRED},
      timeout=SUNNYLINK_RECONNECT_TIMEOUT_S,
    )

  def run(self, exit_event: threading.Event | None = None) -> None:
    """Stay connected to the cloud, reconnecting with the shared backoff."""
    UploadQueueCache.initialize(upload_queue)
    update_car_list_param()

    conn_start = None
    retries = 0

    while (exit_event is None or not exit_event.is_set()) and self.serviceable():
      if sunnylink_need_register(self.params):
        cloudlog.info("Waiting for sunnylink registration to complete")
        time.sleep(10)
        continue

      if conn_start is None:
        conn_start = time.monotonic()

      cloudlog.event("sunnylinkd.main.connecting_ws", retries=retries)
      try:
        ws = self.connect()
      except Exception as e:
        retries += 1
        self.params.remove("LastSunnylinkPingTime")
        self.log_connection_error(e)
        time.sleep(backoff(retries))
        continue

      cloudlog.event("sunnylinkd.main.connected_ws", retries=retries, duration=time.monotonic() - conn_start)
      conn_start = None
      retries = 0
      cur_upload_items.clear()
      self.active_ws = ws

      try:
        self.handle_long_poll(ws, exit_event)
      except (KeyboardInterrupt, SystemExit):
        break
      except Exception as e:
        retries += 1
        self.params.remove("LastSunnylinkPingTime")
        self.log_connection_error(e)
      finally:
        self.active_ws = None

      time.sleep(backoff(retries))

    if not self.serviceable():
      cloudlog.debug("Reached end of sunnylinkd.main while sunnylink is not serviceable. Waiting 60s before retrying")
      time.sleep(60)

  def log_connection_error(self, e: Exception) -> None:
    if isinstance(e, (ConnectionError, TimeoutError, WebSocketException)):
      cloudlog.warning(f"sunnylinkd.main.{type(e).__name__}")
    elif isinstance(e, OSError):
      name = errno.errorcode.get(e.errno or -1, "UNKNOWN")
      msg = f"sunnylinkd.main.OSError.{name} ({e.errno})"
      is_expected_error = e.errno in (errno.ENETDOWN, errno.ENETRESET, errno.ENETUNREACH)
      cloudlog.warning(msg) if is_expected_error else cloudlog.exception(msg)
    else:
      cloudlog.exception("sunnylinkd.main.exception")


def main(exit_event: threading.Event | None = None):
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("failed to set core affinity")

  daemon = Sunnylinkd()
  for method in daemon.methods().values():
    dispatcher.add_method(method)
  daemon.run(exit_event)


if __name__ == "__main__":
  main()
