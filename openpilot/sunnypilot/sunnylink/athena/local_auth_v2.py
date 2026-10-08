"""Sunnylink local v2 security core: keys, grants, the pending window and the pinned TLS dial.

Nothing here is registered with the RPC dispatcher: an enrollment may only be staged from a
cloud-origin payload, never from a local RPC. See sunnylink-mobile/docs/LOCAL_AUTH_V2.md.
"""
from __future__ import annotations

import base64
import hashlib
import json
import re
import secrets
import socket
import ssl
import threading
import ipaddress
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any
from urllib.parse import urlsplit

from websocket import WebSocket, create_connection
from cryptography import x509
from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, padding, rsa
from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature

VERSION = 2
AUTHENTICATE_PURPOSE = "sunnylink-local-authenticate"
PAIRING_TTL_S = 120
MAX_ENROLL_BYTES = 4096
MAX_GRANTS = 8

# The QR the app scans is a compact signed blob, not a JWT: this metadata is 494 characters of
# JSON and base64 in a JWT, which is a 77-module code, and a 77-module code cannot be read off a
# 536x240 panel. The same fields in a binary blob are ~200 characters (53 modules, 4.5 px/module
# there) with every property kept: signed by the device identity key, fresh per window, and
# carrying no bearer capability. Frame:
#
#   "SLEN" | version | algorithm | len(cloud id) | cloud id | key id (32) | session (32) | ttl
#
# and the QR is `<base64url frame>.<base64url signature over the frame bytes>`, the signature
# being raw r||s for ES256 and PKCS#1 v1.5 DER for RS256. The comma device id is deliberately
# absent: the app already gets it from the same authenticated cloud details it checks the key
# against, and a second unauthenticated copy of it is one more thing to disagree with.
QR_MAGIC = b"SLEN"
QR_VERSION = 3
QR_ES256 = 1
QR_RS256 = 2
QR_ALGORITHMS = {QR_ES256: "ES256", QR_RS256: "RS256"}
QR_KEY_ID_BYTES = 32
QR_SESSION_BYTES = 32
MAX_QR_FRAME_BYTES = 256
MAX_QR_SIGNATURE_BYTES = 512
# The only RPCs an authenticated local session may ask for. Long-running operations are
# deliberately absent.
LOCAL_METHODS = frozenset({"getParams", "getParamsAllKeys", "getParamsMetadata", "getMessage", "saveParams",
                           "updateLocalAppAlias", "unpairLocalApp"})
_TOKEN = re.compile(r"[A-Za-z0-9_-]+\Z")


class AuthorizationError(ValueError):
  """Fail closed without exposing protocol internals to an unauthenticated peer."""


class Origin(Enum):
  """Where a trust mutation came from. Local sessions may never enroll or revoke."""
  CLOUD = "cloud"
  LOCAL = "local"


def encode64(data: bytes) -> str:
  return base64.urlsafe_b64encode(data).rstrip(b"=").decode("ascii")


def decode64(value: Any, *, max_bytes: int = MAX_ENROLL_BYTES) -> bytes:
  if not isinstance(value, str) or not value or len(value) > (max_bytes * 4 + 2) // 3 or not _TOKEN.fullmatch(value):
    raise AuthorizationError("invalid encoding")
  try:
    raw = base64.b64decode(value + "=" * (-len(value) % 4), altchars=b"-_", validate=True)
  except ValueError as e:
    raise AuthorizationError("invalid encoding") from e
  if len(raw) > max_bytes or encode64(raw) != value:
    raise AuthorizationError("noncanonical encoding")
  return raw


def load_app_key(value: str) -> ec.EllipticCurvePublicKey:
  raw = decode64(value, max_bytes=128)
  try:
    key = serialization.load_der_public_key(raw)
  except ValueError as e:
    raise AuthorizationError("invalid key") from e
  if not isinstance(key, ec.EllipticCurvePublicKey) or not isinstance(key.curve, ec.SECP256R1):
    raise AuthorizationError("unsupported key")
  if key.public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo) != raw:
    raise AuthorizationError("noncanonical key")
  return key


def key_id_bytes(key: ec.EllipticCurvePublicKey | rsa.RSAPublicKey) -> bytes:
  raw = key.public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo)
  return hashlib.sha256(raw).digest()


def key_id(key: ec.EllipticCurvePublicKey | rsa.RSAPublicKey) -> str:
  return encode64(key_id_bytes(key))


@dataclass(frozen=True)
class QrPayload:
  """What the app reads out of the scanned frame. Never authority on its own: the app checks the
  key id against authenticated cloud device details before it uses any of it."""
  cloud_device_id: str
  device_key_id: bytes
  session: bytes
  ttl_s: int
  algorithm: str


def qr_frame(cloud_device_id: str, device_key_id: bytes, session: bytes, ttl_s: int, algorithm: int) -> bytes:
  if algorithm not in QR_ALGORITHMS:
    raise AuthorizationError("invalid qr algorithm")
  if len(device_key_id) != QR_KEY_ID_BYTES or len(session) != QR_SESSION_BYTES:
    raise AuthorizationError("invalid qr fields")
  if not 0 < ttl_s < 256:
    raise AuthorizationError("invalid qr ttl")
  encoded = text_field(cloud_device_id).encode("utf-8")
  if not 0 < len(encoded) < 256:
    raise AuthorizationError("invalid cloud device id")
  return QR_MAGIC + bytes([QR_VERSION, algorithm, len(encoded)]) + encoded + device_key_id + session + bytes([ttl_s])


def decode_qr(raw: str) -> QrPayload:
  """Parse a scanned QR frame. The signature is the app's to verify (it needs the device key from
  the cloud); this validates the framing so no caller can disagree with [qr_frame]."""
  parts = raw.split(".") if isinstance(raw, str) else []
  if len(parts) != 2:
    raise AuthorizationError("invalid qr")
  frame = decode64(parts[0], max_bytes=MAX_QR_FRAME_BYTES)
  decode64(parts[1], max_bytes=MAX_QR_SIGNATURE_BYTES)
  if len(frame) < len(QR_MAGIC) + 3 + 1 + QR_KEY_ID_BYTES + QR_SESSION_BYTES + 1 or not frame.startswith(QR_MAGIC):
    raise AuthorizationError("invalid qr")
  version, algorithm, id_length = frame[4], frame[5], frame[6]
  if version != QR_VERSION or algorithm not in QR_ALGORITHMS or id_length == 0 or len(frame) != 7 + id_length + QR_KEY_ID_BYTES + QR_SESSION_BYTES + 1:
    raise AuthorizationError("invalid qr")
  try:
    cloud_device_id = text_field(frame[7:7 + id_length].decode("utf-8"))
  except UnicodeDecodeError as e:
    raise AuthorizationError("invalid qr") from e
  key_start = 7 + id_length
  return QrPayload(cloud_device_id, frame[key_start:key_start + QR_KEY_ID_BYTES],
                   frame[key_start + QR_KEY_ID_BYTES:key_start + QR_KEY_ID_BYTES + QR_SESSION_BYTES],
                   frame[-1], QR_ALGORITHMS[algorithm])


def text_field(value: Any, maximum: int = 128) -> str:
  if not isinstance(value, str) or not value or len(value.encode("utf-8")) > maximum or any(ord(c) < 32 or ord(c) == 127 for c in value):
    raise AuthorizationError("invalid field")
  return value


def statement(purpose: str, *fields: str) -> bytes:
  """Length-bounded UTF-8 fields; LF is forbidden, avoiding JSON canonicalization."""
  return ("sunnylink-local-v2\n" + purpose + "\n" + "\n".join(text_field(f, 1024) for f in fields) + "\n").encode("utf-8")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
  result: dict[str, Any] = {}
  for key, value in pairs:
    if key in result:
      raise AuthorizationError("duplicate field")
    result[key] = value
  return result


def parse_object(raw: str) -> dict[str, Any]:
  if not isinstance(raw, str) or len(raw.encode("utf-8")) > MAX_ENROLL_BYTES:
    raise AuthorizationError("message too large")
  try:
    obj = json.loads(raw, object_pairs_hook=_unique_object)
  except (ValueError, UnicodeError) as e:
    raise AuthorizationError("invalid message") from e
  if not isinstance(obj, dict):
    raise AuthorizationError("invalid message")
  return obj


@dataclass(frozen=True)
class Grant:
  app_id: str
  public_key: str
  key_id: str
  app_name: str
  revision: str

  @staticmethod
  def from_dict(data: Mapping[str, Any]) -> Grant:
    if set(data) != {"app_id", "public_key", "key_id", "app_name", "revision"}:
      raise AuthorizationError("invalid grant")
    public = text_field(data["public_key"], 256)
    actual_id = key_id(load_app_key(public))
    if data["key_id"] != actual_id or data["app_id"] != actual_id:
      raise AuthorizationError("key mismatch")
    revision = text_field(data["revision"], 43)
    if len(decode64(revision, max_bytes=32)) != 32:
      raise AuthorizationError("invalid revision")
    return Grant(actual_id, public, actual_id, text_field(data["app_name"], 80), revision)

  def to_dict(self) -> dict[str, str]:
    return {"app_id": self.app_id, "public_key": self.public_key, "key_id": self.key_id,
            "app_name": self.app_name, "revision": self.revision}


@dataclass(frozen=True)
class Window:
  nonce: str
  deadline: float
  qr: str


class LocalAuthority:
  """The daemon-owned authority. Every trust mutation goes through it, and a persistence
  failure never publishes a new grant."""

  def __init__(self, cloud_device_id: str, comma_device_id: str, device_private_key: ec.EllipticCurvePrivateKey | rsa.RSAPrivateKey,
               persist: Callable[[dict[str, Any]], None], registry: Any = None, clock: Callable[[], float] = time.monotonic):
    self.cloud_device_id = text_field(cloud_device_id)
    self.comma_device_id = text_field(comma_device_id)
    self.device_key = device_private_key
    self.device_key_id = key_id(device_private_key.public_key())
    self.persist = persist
    self.clock = clock
    self.lock = threading.RLock()
    self.window: Window | None = None
    self.pending: Grant | None = None
    self.grants: dict[str, Grant] = {}
    if isinstance(registry, dict) and type(registry.get("v")) is int and registry.get("v") == VERSION and set(registry) == {"v", "grants"}:
      entries = registry["grants"]
      if isinstance(entries, list) and len(entries) <= MAX_GRANTS:
        try:
          decoded = [Grant.from_dict(entry) for entry in entries if isinstance(entry, dict)]
          if len(decoded) == len(entries) and len({g.key_id for g in decoded}) == len(decoded):
            self.grants = {g.key_id: g for g in decoded}
        except AuthorizationError:
          pass  # Corrupt/legacy storage is not authorization.

  def arm(self) -> str:
    with self.lock:
      session = secrets.token_bytes(QR_SESSION_BYTES)
      algorithm = QR_ES256 if isinstance(self.device_key, ec.EllipticCurvePrivateKey) else QR_RS256
      frame = qr_frame(self.cloud_device_id, key_id_bytes(self.device_key.public_key()), session, PAIRING_TTL_S, algorithm)
      qr = encode64(frame) + "." + encode64(self._sign_frame(frame))
      self.window = Window(encode64(session), self.clock() + PAIRING_TTL_S, qr)
      self.pending = None
      return qr

  def _sign_frame(self, frame: bytes) -> bytes:
    """ES256 signs get their raw r||s form, the compact encoding every peer can carry in 86
    characters; the isinstance is also what narrows the key type for the call."""
    if isinstance(self.device_key, ec.EllipticCurvePrivateKey):
      r, s = decode_dss_signature(self.device_key.sign(frame, ec.ECDSA(hashes.SHA256())))
      return r.to_bytes(32, "big") + s.to_bytes(32, "big")
    return self.device_key.sign(frame, padding.PKCS1v15(), hashes.SHA256())

  def cancel(self) -> None:
    with self.lock:
      self.window = None
      self.pending = None

  def _fresh(self, nonce: str) -> None:
    if self.window is None or self.clock() >= self.window.deadline or not secrets.compare_digest(self.window.nonce, nonce):
      raise AuthorizationError("enrollment unavailable")

  def enroll(self, raw: str, origin: Origin) -> Grant:
    if origin is not Origin.CLOUD:
      raise AuthorizationError("cloud authorization required")
    data = parse_object(raw)
    required = {"v", "cloud_device_id", "comma_device_id", "device_key_id", "pairing_session", "public_key", "key_id", "app_name", "signature"}
    if set(data) != required or type(data["v"]) is not int or data["v"] != VERSION:
      raise AuthorizationError("invalid enrollment")
    if (data["cloud_device_id"], data["comma_device_id"], data["device_key_id"]) != \
       (self.cloud_device_id, self.comma_device_id, self.device_key_id):
      raise AuthorizationError("wrong device")
    public = text_field(data["public_key"], 256)
    key = load_app_key(public)
    app_id = key_id(key)
    if data["key_id"] != app_id:
      raise AuthorizationError("wrong key")
    nonce = text_field(data["pairing_session"], 43)
    if len(decode64(nonce, max_bytes=32)) != 32:
      raise AuthorizationError("invalid nonce")
    name = text_field(data["app_name"], 80)
    signed = statement("enroll", self.cloud_device_id, self.comma_device_id, self.device_key_id, nonce, public, app_id, name)
    try:
      key.verify(decode64(data["signature"], max_bytes=72), signed, ec.ECDSA(hashes.SHA256()))
    except InvalidSignature as e:
      raise AuthorizationError("invalid proof") from e
    with self.lock:
      self._fresh(nonce)
      if self.pending is not None:
        if (self.pending.public_key, self.pending.app_name) != (public, name):
          raise AuthorizationError("window already bound")
        return self.pending
      if app_id not in self.grants and len(self.grants) >= MAX_GRANTS:
        raise AuthorizationError("grant limit")
      self.pending = Grant(app_id, public, app_id, name, encode64(secrets.token_bytes(32)))
      return self.pending

  def confirm(self, grant: Grant, tls_peer_key_id: str) -> Grant:
    """Called only after the transport proves the pinned TLS server key."""
    with self.lock:
      if not secrets.compare_digest(grant.key_id, tls_peer_key_id):
        raise AuthorizationError("wrong TLS peer")
      if self.pending != grant:
        raise AuthorizationError("no pending enrollment")
      if self.window is None:
        raise AuthorizationError("window closed")
      self._fresh(self.window.nonce)
      updated = self.grants | {grant.key_id: grant}
      self.persist({"v": VERSION, "grants": [g.to_dict() for g in updated.values()]})
      self.grants = updated
      self.cancel()
      return grant

  def authenticate_device(self, phone_key_id: str, challenge: str) -> dict[str, str]:
    """Proof of device identity for the phone's fresh challenge, sent only inside a TLS
    channel already pinned to `phone_key_id`."""
    if len(decode64(challenge, max_bytes=32)) != 32:
      raise AuthorizationError("invalid challenge")
    fields = (self.cloud_device_id, self.comma_device_id, self.device_key_id,
              text_field(phone_key_id, 128), text_field(challenge, 128))
    proof = statement("authenticate-device", *fields)
    if isinstance(self.device_key, ec.EllipticCurvePrivateKey):
      signature = self.device_key.sign(proof, ec.ECDSA(hashes.SHA256()))
    else:
      signature = self.device_key.sign(proof, padding.PKCS1v15(), hashes.SHA256())
    return {"cloud_device_id": self.cloud_device_id, "comma_device_id": self.comma_device_id,
            "device_key_id": self.device_key_id, "phone_key_id": phone_key_id,
            "challenge": challenge, "signature": encode64(signature)}

  def revoke_by_self_proof(self, raw: str, origin: Origin) -> str:
    """Cloud-origin self-revocation: a phone drops its OWN grant.

    The proof is signed by the very key being dropped and verified against the grant this device
    already holds, so it can only ever remove an enrolled phone. Removing a key that is not
    enrolled is not an error: there is simply nothing left to drop.
    """
    if origin is not Origin.CLOUD:
      raise AuthorizationError("cloud authorization required")
    data = parse_object(raw)
    if set(data) != {"v", "cloud_device_id", "comma_device_id", "key_id", "signature"} \
       or type(data["v"]) is not int or data["v"] != VERSION:
      raise AuthorizationError("invalid revocation")
    if (data["cloud_device_id"], data["comma_device_id"]) != (self.cloud_device_id, self.comma_device_id):
      raise AuthorizationError("wrong device")
    app_id = text_field(data["key_id"], 128)
    if len(decode64(app_id, max_bytes=32)) != 32:
      raise AuthorizationError("invalid key id")

    with self.lock:
      grant = self.grants.get(app_id)
    if grant is None:
      # Nothing enrolled under this key. `revoke` still drops a matching staged enrollment.
      self.revoke(app_id)
      return app_id

    proof = statement("revoke", self.cloud_device_id, self.comma_device_id, app_id)
    key = load_app_key(grant.public_key)
    try:
      key.verify(decode64(data["signature"], max_bytes=72), proof, ec.ECDSA(hashes.SHA256()))
    except InvalidSignature as e:
      raise AuthorizationError("invalid proof") from e
    self.revoke(app_id)
    return app_id

  def revoke_all(self) -> int:
    """Drop every grant in one durable write, leaving an open pairing window alone."""
    with self.lock:
      removed = len(self.grants)
      if removed == 0:
        return 0
      self.persist({"v": VERSION, "grants": []})
      self.grants = {}
      return removed

  def revoke(self, app_key_id: str) -> bool:
    with self.lock:
      if self.pending is not None and self.pending.key_id == app_key_id:
        self.cancel()
      if app_key_id not in self.grants:
        return False
      updated = {k: g for k, g in self.grants.items() if k != app_key_id}
      self.persist({"v": VERSION, "grants": [g.to_dict() for g in updated.values()]})
      self.grants = updated
      return True

  def session(self, grant: Grant, tls_peer_key_id: str) -> AuthorizedSession:
    with self.lock:
      if self.grants.get(grant.key_id) != grant or not secrets.compare_digest(grant.key_id, tls_peer_key_id):
        raise AuthorizationError("unauthorized peer")
      return AuthorizedSession(self, grant)


class AuthorizedSession:
  """Session-owned authorization, checked immediately before each handler executes."""

  def __init__(self, authority: LocalAuthority, grant: Grant):
    self.authority = authority
    self.grant = grant
    self.closed = False

  def close(self) -> None:
    with self.authority.lock:
      self.closed = True

  def invoke(self, method: str, handler: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
    with self.authority.lock:
      if self.closed or self.authority.grants.get(self.grant.key_id) != self.grant or method not in LOCAL_METHODS:
        raise AuthorizationError("request denied")
      # Serializes trust revocation with handler execution. Long-running operations
      # are deliberately excluded from LOCAL_METHODS.
      return handler(*args, **kwargs)


def connect_pinned(endpoint: str, expected_key_id: str, timeout_s: float = 10) -> WebSocket:
  """Pin before the HTTP upgrade, so no header or RPC leaves the device unverified.

  PKIX is replaced by an exact public-key pin for this local socket only; never reuse this
  context for cloud traffic. The pin proves private-key possession, nothing else.
  """
  parsed = urlsplit(endpoint)
  if parsed.scheme != "wss" or parsed.username is not None or parsed.password is not None \
     or parsed.query or parsed.fragment or parsed.path not in ("", "/") or parsed.hostname is None:
    raise AuthorizationError("invalid local endpoint")
  try:
    address = ipaddress.ip_address(parsed.hostname)
    port = parsed.port
  except ValueError as e:
    raise AuthorizationError("invalid local endpoint") from e
  if port is None or address.is_unspecified or address.is_multicast:
    raise AuthorizationError("invalid local endpoint")
  context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
  context.check_hostname = False
  context.verify_mode = ssl.CERT_NONE
  context.minimum_version = ssl.TLSVersion.TLSv1_3
  context.maximum_version = ssl.TLSVersion.TLSv1_3
  # New context and no supplied session prevent client resumption/early data.
  connection = socket.create_connection((str(address), port), timeout=timeout_s)
  tls: ssl.SSLSocket | None = None
  try:
    tls = context.wrap_socket(connection, server_hostname=None)
    certificate = tls.getpeercert(binary_form=True)
    if certificate is None:
      raise AuthorizationError("missing peer certificate")
    verify_tls_peer(certificate, expected_key_id)
    return create_connection(endpoint, socket=tls, enable_multithread=True, timeout=timeout_s,
                             subprotocols=["sunnylink-local-v2"])
  except Exception:
    (tls or connection).close()
    raise


def verify_tls_peer(certificate_der: bytes, expected_key_id: str) -> str:
  try:
    key = x509.load_der_x509_certificate(certificate_der).public_key()
  except ValueError as e:
    raise AuthorizationError("invalid peer certificate") from e
  if not isinstance(key, ec.EllipticCurvePublicKey) or not isinstance(key.curve, ec.SECP256R1):
    raise AuthorizationError("unsupported peer key")
  actual = key_id(key)
  if not secrets.compare_digest(actual, expected_key_id):
    raise AuthorizationError("wrong TLS peer")
  return actual
