#!/usr/bin/env python3
"""
Local HTTP server for sunnylink direct browser access.

Runs alongside sunnylinkd on the LAN interface so the sunnylink web frontend
can connect directly to the device when on the same network — no cloud proxy.

Exposes a subset of the Athena REST API (same wire format) so the frontend's
existing fetch + decode logic works unchanged.

Discovery: advertises via mDNS as sunnylink-<dongle_id>.local so the frontend
can probe paired devices without manual IP entry.
"""
from __future__ import annotations

import base64
import gzip
import json
import os
import socket
import subprocess
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

from openpilot.common.params import Params, ParamKeyType
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.sunnylink.api import UNREGISTERED_SUNNYLINK_DONGLE_ID
from openpilot.sunnypilot.sunnylink.utils import (
    get_param_as_byte, save_param_from_base64_encoded_string, sunnylink_ready
)

DEFAULT_PORT = 8456

# Parameters blocked from remote modification (mirrors sunnylinkd.py)
BLOCKED_PARAMS = {
    "AdbEnabled", "CompletedSunnylinkConsentVersion", "CompletedTrainingVersion",
    "GithubUsername", "GithubSshKeys", "HasAcceptedTerms", "HasAcceptedTermsSP",
    "OnroadCycleRequested", "ParamsVersion",
}

_mdns_process: subprocess.Popen | None = None
_mdns_thread: threading.Thread | None = None


def _get_dongle_id(params: Params) -> str:
    return (params.get("SunnylinkDongleId") or UNREGISTERED_SUNNYLINK_DONGLE_ID).decode("utf-8") if isinstance(params.get("SunnylinkDongleId"), bytes) else (params.get("SunnylinkDongleId") or UNREGISTERED_SUNNYLINK_DONGLE_ID)


def _get_local_ip() -> str | None:
    """Get the primary LAN IP address of the device."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        pass

    # Fallback: scan interfaces
    try:
        for iface in socket.getaddrinfo(socket.gethostname(), None):
            addr = iface[4][0]
            if addr and not addr.startswith("127.") and ":" not in addr:
                return addr
    except Exception:
        pass

    return None


def _start_mdns(dongle_id: str, port: int) -> None:
    """Advertise this device via mDNS so the frontend can discover it.

    Tries avahi-publish-service first, then a pure-Python UDP responder
    that listens on 5353 and answers A-record queries for our hostname.
    """
    global _mdns_process, _mdns_thread
    hostname = f"sunnylink-{dongle_id}"

    # Strategy 1: avahi-publish-service CLI (fast, minimal code)
    try:
        _mdns_process = subprocess.Popen(
            ["avahi-publish-service", hostname, "_sunnylink._tcp", str(port),
             f"dongle_id={dongle_id}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        cloudlog.info(f"local_server: mDNS via avahi-publish-service: {hostname}.local:{port}")
        return
    except FileNotFoundError:
        pass
    except Exception as e:
        cloudlog.debug(f"local_server: avahi-publish-service failed: {e}")

    # Strategy 2: pure-Python mDNS responder (no system deps)
    _mdns_thread = _start_mdns_responder(hostname, dongle_id, port)


def _start_mdns_responder(hostname: str, dongle_id: str, port: int) -> threading.Thread | None:
    """Start a pure-Python mDNS responder in a background thread.

    Binds to the mDNS multicast group (224.0.0.251:5353) with SO_REUSEADDR
    + SO_REUSEPORT so it coexists peacefully with any existing mDNS daemon
    (avahi, Bonjour).  Listens for A-record queries for our hostname and
    responds with the device's LAN IP.

    Based on the approach described at:
    https://github.com/angrycoding/js-local-service-discovery
    """
    import struct
    import select

    local_ip = _get_local_ip()
    if not local_ip:
        cloudlog.warning("local_server: cannot determine local IP, mDNS unavailable")
        return None

    fqdn = f"{hostname}.local"
    ip_parts = bytes(int(b) for b in local_ip.split("."))

    # Pre-encode the hostname as DNS labels
    def _encode_name(name: str) -> bytes:
        out = bytearray()
        for label in name.encode("idna").split(b"."):
            out.append(len(label))
            out.extend(label)
        out.append(0)
        return bytes(out)

    _encoded_name = _encode_name(fqdn)
    _encoded_name_lower = _encode_name(fqdn.lower())

    def _build_answer(query_id: int, question_data: bytes) -> bytes:
        """Build a complete mDNS response: question echoed + our A-record answer."""
        pkt = bytearray()
        # Header: response (QR=1) + authoritative (AA=1)
        pkt += struct.pack(">H", query_id)
        pkt += struct.pack(">H", 0x8400)  # QR=1, OPCODE=0, AA=1, TC=0, RD=0
        pkt += struct.pack(">H", 1)       # QDCOUNT = 1 (echo the question)
        pkt += struct.pack(">H", 1)       # ANCOUNT = 1 (our answer)
        pkt += struct.pack(">H", 0)       # NSCOUNT
        pkt += struct.pack(">H", 0)       # ARCOUNT
        # Echo the original question section
        pkt += question_data
        # Answer: name pointer to question start (offset 12 from start of packet)
        pkt += struct.pack(">H", 0xc000 | 12)
        # Type A (1), Class IN (1), TTL 120, RDLENGTH 4
        pkt += struct.pack(">HHIH", 1, 1, 120, 4)
        pkt += ip_parts
        return bytes(pkt)

    def _responder() -> None:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # SO_REUSEPORT lets us bind alongside avahi-daemon on Linux
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except (AttributeError, OSError):
                pass

            # Join multicast group on all interfaces
            mreq = struct.pack("4s4s",
                socket.inet_aton("224.0.0.251"),
                socket.inet_aton("0.0.0.0"))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

            # Disable multicast loopback — don't receive our own responses
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

            sock.bind(("0.0.0.0", 5353))
            sock.setblocking(False)

            cloudlog.info(f"local_server: mDNS responder listening for {fqdn} → {local_ip}")

            while True:
                ready, _, _ = select.select([sock], [], [], 5.0)
                if not ready:
                    continue

                try:
                    data, addr = sock.recvfrom(2048)
                except Exception:
                    continue

                if len(data) < 12 + 5:  # header + minimal question
                    continue

                # Parse DNS header (big-endian)
                query_id = struct.unpack(">H", data[0:2])[0]
                flags = struct.unpack(">H", data[2:4])[0]
                qdcount = struct.unpack(">H", data[4:6])[0]

                # Only respond to queries (QR=0), not responses
                if flags & 0x8000:
                    continue
                if qdcount == 0:
                    continue

                # Quick check: does the question section contain our hostname?
                # Compare the encoded name bytes directly (case-insensitive).
                question_start = 12
                question_end = len(data)

                # The question name ends at a zero-length label (\x00).
                for i in range(question_start, min(question_start + 80, len(data))):
                    if data[i] == 0:
                        question_end = i + 1
                        break

                question_name = data[question_start:question_end]

                # Compare against both exact and lowercase encodings.
                # DNS is case-insensitive but some resolvers preserve case.
                if question_name != _encoded_name and question_name != _encoded_name_lower:
                    # br-lan / bridge interfaces: the query may contain
                    # our hostname as suffix of a larger query.  Only match
                    # exact or multi-label queries that START with our name.
                    continue

                # Read type from the question (right after the name)
                qtype_start = question_end
                if qtype_start + 4 > len(data):
                    continue
                qtype = struct.unpack(">H", data[qtype_start:qtype_start + 2])[0]

                # Only answer A (1) and ANY (255) queries
                if qtype not in (1, 255):
                    continue

                # Build and send response
                question_data = data[question_start:qtype_start + 4]
                answer = _build_answer(query_id, question_data)
                try:
                    sock.sendto(answer, addr)
                    cloudlog.debug(
                        f"local_server: mDNS answered {fqdn} → {local_ip} "
                        f"to {addr[0]}:{addr[1]}"
                    )
                except Exception as e:
                    cloudlog.debug(f"local_server: mDNS send error: {e}")

        except Exception as e:
            cloudlog.warning(f"local_server: mDNS responder crashed: {e}")
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

    t = threading.Thread(target=_responder, name="mdns_responder", daemon=True)
    t.start()
    cloudlog.info(f"local_server: mDNS responder started: {hostname}.local → {local_ip}:{port}")
    return t


def _stop_mdns() -> None:
    global _mdns_process, _mdns_thread
    if _mdns_process:
        try:
            _mdns_process.terminate()
            _mdns_process.wait(timeout=2)
        except Exception:
            try:
                _mdns_process.kill()
            except Exception:
                pass
        _mdns_process = None
    # The daemon thread dies with the process; nothing to explicitly stop.


class LocalServerHandler(BaseHTTPRequestHandler):
    """
    HTTP request handler that mirrors the Athena REST API.

    Endpoints:
      GET  /ping                         -> device identity + health
      GET  /api/v1/settings              -> list available params (getParamsAllKeys)
      GET  /api/v1/settings/values       -> get param values (?paramKeys=a,b,c)
      GET  /api/v1/settings/metadata     -> compressed settings schema + capabilities
      POST /api/v0/settings              -> save params [body: [{key,value,is_compressed}]]
      GET  /api/v1/message               -> cereal service message (?service=deviceState)
    """

    # Silence noisy request logs from BaseHTTPRequestHandler
    def log_message(self, fmt, *args):
        pass

    def _send_json(self, data: dict | list, status: int = 200) -> None:
        body = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def _send_error_json(self, status: int, detail: str) -> None:
        self._send_json({"status": status, "title": detail, "detail": detail}, status)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        qs = parse_qs(parsed.query)

        try:
            if path == "/ping":
                self._handle_ping()
            elif path == "/api/v1/settings":
                self._handle_get_settings_keys()
            elif path == "/api/v1/settings/values":
                self._handle_get_settings_values(qs)
            elif path == "/api/v1/settings/metadata":
                self._handle_get_metadata()
            elif path == "/api/v1/message":
                self._handle_get_message(qs)
            else:
                self._send_error_json(404, f"Not found: {path}")
        except Exception as e:
            cloudlog.exception(f"local_server: error handling GET {path}: {e}")
            self._send_error_json(500, str(e))

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        print(f"[local_server] POST {self.path!r} → path={path!r}", flush=True)

        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            data = json.loads(body) if body else {}

            if path == "/api/v0/settings":
                self._handle_save_settings(data)
            else:
                print(f"[local_server] 404: path {path!r} did not match /api/v0/settings", flush=True)
                self._send_error_json(404, f"Not found: {path}")
        except json.JSONDecodeError:
            self._send_error_json(400, "Invalid JSON body")
        except Exception as e:
            cloudlog.exception(f"local_server: error handling POST {path}: {e}")
            self._send_error_json(500, str(e))

    def _handle_ping(self) -> None:
        params = self.server.params
        dongle_id = _get_dongle_id(params)
        is_ready = sunnylink_ready(params)
        local_ip = _get_local_ip()

        try:
            import openpilot.common.version as version_module
            version = getattr(version_module, 'version', 'unknown')
        except Exception:
            version = "unknown"

        from openpilot.sunnypilot.sunnylink.utils import get_sunnylink_status
        is_enabled, is_registered, is_fault = get_sunnylink_status(params)

        try:
            from openpilot.common.hardware import HARDWARE
            device_type = HARDWARE.get_device_type()
        except Exception:
            device_type = "unknown"

        self._send_json({
            "dongle_id": dongle_id,
            "version": version,
            "device_type": device_type,
            "sunnylink_enabled": is_enabled,
            "sunnylink_registered": is_registered,
            "sunnylink_ready": is_ready,
            "local_ip": local_ip,
            "port": self.server.port,
        })

    def _handle_get_settings_keys(self) -> None:
        params = self.server.params
        all_keys = [k.decode("utf-8") for k in params.all_keys()]

        items = []
        for key in sorted(all_keys):
            param_type = params.get_type(key)
            type_name = _param_type_to_string(param_type)
            items.append({
                "key": key,
                "type": type_name,
            })

        self._send_json({
            "items": items,
            "count": len(items),
        })

    def _handle_get_settings_values(self, qs: dict) -> None:
        param_keys_str = qs.get("paramKeys", [""])[0]
        if not param_keys_str:
            self._send_error_json(400, "Missing required query parameter: paramKeys")
            return

        requested_keys = [k.strip() for k in param_keys_str.split(",") if k.strip()]
        if not requested_keys:
            self._send_error_json(400, "No valid paramKeys provided")
            return

        params = self.server.params
        available_keys = {k.decode("utf-8") for k in params.all_keys()}
        zero_values = {
            ParamKeyType.STRING: b"",
            ParamKeyType.BOOL: b"0",
            ParamKeyType.INT: b"0",
            ParamKeyType.FLOAT: b"0.0",
            ParamKeyType.TIME: b"",
            ParamKeyType.JSON: b"{}",
            ParamKeyType.BYTES: b"",
        }

        items = []
        for key in requested_keys:
            if key not in available_keys:
                continue

            value = get_param_as_byte(key)
            if value is None:
                value = get_param_as_byte(key, get_default=True)
            if value is None:
                param_type = params.get_type(key)
                value = zero_values.get(ParamKeyType(param_type.value), b"")

            encoded = base64.b64encode(value).decode("utf-8")
            param_type = params.get_type(key)
            type_name = _param_type_to_string(param_type)

            items.append({
                "key": key,
                "value": encoded,
                "type": type_name,
                "is_compressed": False,
            })

        self._send_json({
            "items": items,
            "count": len(items),
        })

    def _handle_get_metadata(self) -> None:
        """Return the compressed settings schema + capabilities."""
        try:
            from openpilot.sunnypilot.sunnylink.tools.generate_settings_schema import generate_schema
            from openpilot.sunnypilot.sunnylink.capabilities import generate_capabilities, CAPABILITY_LABELS
            from openpilot.sunnypilot.models.default_model import DEFAULT_MODEL

            schema = generate_schema()
            schema["capabilities"] = generate_capabilities()
            schema["capability_labels"] = CAPABILITY_LABELS
            schema["default_model"] = DEFAULT_MODEL

            raw = json.dumps(schema, separators=(",", ":")).encode("utf-8")
            params_metadata = base64.b64encode(gzip.compress(raw)).decode("utf-8")

            self._send_json({"params_metadata": params_metadata})
        except Exception as e:
            cloudlog.exception("local_server: getParamsMetadata failed")
            self._send_error_json(500, f"Failed to generate metadata: {e}")

    def _handle_get_message(self, qs: dict) -> None:
        """Return a cereal service message."""
        service = qs.get("service", [None])[0]
        if not service:
            self._send_error_json(400, "Missing required query parameter: service")
            return

        try:
            import openpilot.cereal.messaging as messaging
            # poll=service ensures SubMaster properly subscribes and waits
            # for the requested service before returning from update().
            sm = messaging.SubMaster([service], poll=service)
            # First update: up to 2s to connect and receive initial message.
            # Cereal services publish at varying rates (deviceState ~2Hz,
            # selfdriveState ~20Hz), so we may need a few cycles to catch one.
            sm.update(2000)

            if sm.valid.get(service, False):
                data = sm[service]
                result = _capnp_to_dict(data)
                self._send_json({service: result})
            else:
                self._send_error_json(
                    503,
                    f"Service '{service}' not available (device may be offroad)"
                )
        except Exception as e:
            cloudlog.exception(f"local_server: getMessage({service}) failed")
            self._send_error_json(500, f"Failed to get message for {service}: {e}")

    def _handle_save_settings(self, data: list) -> None:
        """Save params. Body: [{key, value, is_compressed?}]"""
        if not isinstance(data, list):
            self._send_error_json(400, "Body must be a JSON array of {key, value, is_compressed?}")
            return

        params_to_update: dict[str, str] = {}
        for item in data:
            key = item.get("key", "")
            value = item.get("value", "")
            is_compressed = item.get("is_compressed", False)

            if not key:
                continue
            if key in BLOCKED_PARAMS:
                cloudlog.warning(f"local_server: blocked param modification attempt: {key}")
                continue

            # Decode the base64 value and save via the same path as sunnylinkd
            save_param_from_base64_encoded_string(key, value, is_compressed)
            params_to_update[key] = value

        self._send_json({"saved": len(params_to_update), "keys": list(params_to_update.keys())})


def _param_type_to_string(param_type: ParamKeyType) -> str:
    mapping = {
        ParamKeyType.STRING: "String",
        ParamKeyType.BOOL: "Bool",
        ParamKeyType.INT: "Int",
        ParamKeyType.FLOAT: "Float",
        ParamKeyType.TIME: "Time",
        ParamKeyType.JSON: "Json",
        ParamKeyType.BYTES: "Bytes",
    }
    return mapping.get(param_type, "Unknown")


def _capnp_to_dict(msg) -> dict:
    """Convert a capnp struct to a plain dict for JSON serialization."""
    result = {}
    if msg is None:
        return result
    try:
        for field in msg.schema.fields:
            name = field.name
            proto = msg.schema.node
            # Cap'n Proto fields
            if hasattr(msg, name):
                val = getattr(msg, name)
                if hasattr(val, 'to_dict'):
                    result[name] = val.to_dict()
                elif hasattr(val, 'schema'):
                    result[name] = _capnp_to_dict(val)
                elif isinstance(val, (list, tuple)):
                    result[name] = [
                        v.to_dict() if hasattr(v, 'to_dict') else _capnp_to_dict(v) if hasattr(v, 'schema') else str(v)
                        for v in val
                    ]
                elif isinstance(val, bytes):
                    result[name] = val.hex()
                elif isinstance(val, (bool, int, float, str, type(None))):
                    result[name] = val
                else:
                    result[name] = str(val)
    except Exception:
        # Fallback: try to_dict if available
        if hasattr(msg, 'to_dict'):
            try:
                return msg.to_dict()
            except Exception:
                pass
    return result


class LocalServer(HTTPServer):
    """Threaded HTTP server with access to device Params."""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, port: int, params: Params):
        self.params = params
        self.port = port
        super().__init__(("0.0.0.0", port), LocalServerHandler)


def main(exit_event: threading.Event | None = None):
    """Run the local HTTP server. Blocks until exit_event is set."""
    params = Params()
    port = int(os.getenv("SUNNYLINK_LOCAL_PORT", str(DEFAULT_PORT)))

    if not sunnylink_ready(params):
        cloudlog.info("local_server: sunnylink not ready, waiting...")
        while not sunnylink_ready(params):
            time.sleep(5)
            if exit_event and exit_event.is_set():
                return

    dongle_id = _get_dongle_id(params)

    cloudlog.info(f"local_server: starting on port {port} for device {dongle_id}")

    # Start mDNS advertisement
    _start_mdns(dongle_id, port)

    server = LocalServer(port, params)

    def serve():
        try:
            server.serve_forever(poll_interval=0.5)
        except Exception as e:
            cloudlog.exception(f"local_server: serve_forever error: {e}")

    server_thread = threading.Thread(target=serve, name="local_server", daemon=True)
    server_thread.start()

    local_ip = _get_local_ip()
    cloudlog.info(f"local_server: listening on http://{local_ip}:{port} (dongle_id={dongle_id})")
    cloudlog.info(f"local_server: mDNS name: sunnylink-{dongle_id}.local")

    try:
        while exit_event is None or not exit_event.is_set():
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cloudlog.info("local_server: shutting down")
        _stop_mdns()
        server.shutdown()
        server_thread.join(timeout=5)


if __name__ == "__main__":
    main()
