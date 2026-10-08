import base64
import hashlib
import json
import socket
import ssl
import tempfile
import threading
from pathlib import Path
from dataclasses import replace
from datetime import UTC, datetime, timedelta
from unittest import mock

import jwt
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, rsa
from cryptography.x509.oid import NameOID

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.sunnylink.athena.local_auth_v2 import (
  AuthorizationError, Grant, LocalAuthority, Origin, connect_pinned, decode64, encode64, key_id, parse_object, statement, verify_tls_peer,
)

# The fields an enrollment signs, in the order the statement joins them.
ENROLL_FIELDS = ("cloud_device_id", "comma_device_id", "device_key_id", "pairing_session",
                 "public_key", "key_id", "app_name")


class TestLocalAuthV2(OpenpilotTestCase):
  def setup_method(self):
    self.now = 10.0
    self.device_key = ec.generate_private_key(ec.SECP256R1())
    self.app_key = ec.generate_private_key(ec.SECP256R1())
    self.writes = []
    self.authority = LocalAuthority("cloud-device", "comma-device", self.device_key, self.writes.append, clock=lambda: self.now)
    self.qr = self.authority.arm()

  def enrollment(self, app_key=None, **changes):
    app_key = app_key or self.app_key
    public = encode64(app_key.public_key().public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo))
    data = {"v": 2, "cloud_device_id": self.authority.cloud_device_id, "comma_device_id": self.authority.comma_device_id,
            "device_key_id": self.authority.device_key_id,
            "pairing_session": jwt.decode(self.qr, self.device_key.public_key(), algorithms=["ES256"])["pairing_session"],
            "public_key": public, "key_id": key_id(app_key.public_key()), "app_name": "Test phone"}
    data.update(changes)
    proof = statement("enroll", *(str(data[field]) for field in ENROLL_FIELDS))
    data["signature"] = encode64(app_key.sign(proof, ec.ECDSA(hashes.SHA256())))
    return json.dumps(data)

  def committed(self):
    grant = self.authority.enroll(self.enrollment(), Origin.CLOUD)
    return self.authority.confirm(grant, grant.key_id)

  def test_qr_is_signed_public_fresh_metadata_not_bearer(self):
    data = jwt.decode(self.qr, self.device_key.public_key(), algorithms=["ES256"])
    self.assertEqual(set(data), {"v", "purpose", "cloud_device_id", "comma_device_id", "device_key_id", "pairing_session", "ttl_s"})
    self.assertEqual(data["ttl_s"], 120)
    self.assertEqual(len(decode64(data["pairing_session"])), 32)
    self.assertNotEqual(self.qr, self.authority.arm())
    with self.assertRaises(jwt.InvalidSignatureError):
      jwt.decode(self.qr, self.app_key.public_key(), algorithms=["ES256"])

  def test_rsa_device_identity_supported(self):
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    authority = LocalAuthority("cloud", "comma", key, self.writes.append)
    payload = jwt.decode(authority.arm(), key.public_key(), algorithms=["RS256"])
    self.assertEqual(payload["device_key_id"], key_id(key.public_key()))

  def test_device_authenticates_itself_to_the_phone_challenge(self):
    grant = self.committed()
    challenge = encode64(bytes(32))
    proof = self.authority.authenticate_device(grant.key_id, challenge)
    self.assertEqual(proof["device_key_id"], self.authority.device_key_id)
    self.assertEqual(proof["phone_key_id"], grant.key_id)
    expected = statement("authenticate-device", "cloud-device", "comma-device",
                         self.authority.device_key_id, grant.key_id, challenge)
    self.device_key.public_key().verify(decode64(proof["signature"]), expected, ec.ECDSA(hashes.SHA256()))
    with self.assertRaises(AuthorizationError):
      self.authority.authenticate_device(grant.key_id, encode64(bytes(31)))

  def test_local_origin_cannot_enroll_even_with_correct_qr_and_key(self):
    with self.assertRaises(AuthorizationError):
      self.authority.enroll(self.enrollment(), Origin.LOCAL)
    self.assertEqual(self.writes, [])
    self.assertIsNone(self.authority.pending)

  def test_pending_grant_is_not_authorized_and_requires_tls_proof(self):
    grant = self.authority.enroll(self.enrollment(), Origin.CLOUD)
    self.assertEqual(self.writes, [])
    with self.assertRaises(AuthorizationError):
      self.authority.session(grant, grant.key_id)
    with self.assertRaises(AuthorizationError):
      self.authority.confirm(grant, key_id(self.device_key.public_key()))
    self.assertEqual(self.writes, [])
    self.authority.confirm(grant, grant.key_id)
    self.assertEqual(len(self.writes), 1)

  def test_same_key_retries_are_idempotent_but_competing_scanner_cannot_replace_key(self):
    grant = self.authority.enroll(self.enrollment(), Origin.CLOUD)
    self.assertEqual(grant, self.authority.enroll(self.enrollment(), Origin.CLOUD))
    with self.assertRaises(AuthorizationError):
      self.authority.enroll(self.enrollment(ec.generate_private_key(ec.SECP256R1())), Origin.CLOUD)
    self.assertEqual(self.authority.pending, grant)

  def test_enrollment_expires_before_acceptance_and_before_commit(self):
    grant = self.authority.enroll(self.enrollment(), Origin.CLOUD)
    self.now = 130.0
    for operation in (lambda: self.authority.enroll(self.enrollment(), Origin.CLOUD), lambda: self.authority.confirm(grant, grant.key_id)):
      with self.assertRaises(AuthorizationError):
        operation()
    self.assertEqual(self.writes, [])

  def test_cancel_rearm_and_restart_invalidate_old_enrollment(self):
    raw = self.enrollment()
    grant = self.authority.enroll(raw, Origin.CLOUD)
    self.authority.cancel()
    with self.assertRaises(AuthorizationError):
      self.authority.confirm(grant, grant.key_id)
    self.authority.arm()
    with self.assertRaises(AuthorizationError):
      self.authority.enroll(raw, Origin.CLOUD)
    restarted = LocalAuthority("cloud-device", "comma-device", self.device_key, self.writes.append)
    with self.assertRaises(AuthorizationError):
      restarted.enroll(raw, Origin.CLOUD)

  def test_signature_binds_all_enrollment_fields(self):
    for field, value in (("app_name", "Other phone"), ("key_id", "other"), ("device_key_id", "other"), ("comma_device_id", "other"),
                         ("cloud_device_id", "other"), ("pairing_session", encode64(b"x" * 32))):
      with self.subTest(field=field):
        data = json.loads(self.enrollment())
        data[field] = value
        with self.assertRaises(AuthorizationError):
          self.authority.enroll(json.dumps(data), Origin.CLOUD)
    self.assertEqual(self.writes, [])

  def test_bad_versions_fields_keys_and_messages_rejected(self):
    for change in ({"v": True}, {"v": 1}, {"extra": "bad"}, {"public_key": "garbage"}, {"signature": "garbage"},
                   {"app_name": "bad\nname"}, {"app_name": "x" * 81}):
      with self.subTest(change=change):
        data = json.loads(self.enrollment())
        data.update(change)
        with self.assertRaises((AuthorizationError, ValueError)):
          self.authority.enroll(json.dumps(data), Origin.CLOUD)
    for raw in ('{"v":2,"v":2}', '[]', 'null', 'x' * 4097):
      with self.assertRaises(AuthorizationError):
        parse_object(raw)

  def test_persistence_failure_never_publishes_grant(self):
    grant = self.authority.enroll(self.enrollment(), Origin.CLOUD)
    with mock.patch.object(self.authority, "persist", side_effect=OSError("disk full")):
      with self.assertRaises(OSError):
        self.authority.confirm(grant, grant.key_id)
    self.assertEqual(self.authority.grants, {})
    with self.assertRaises(AuthorizationError):
      self.authority.session(grant, grant.key_id)

  def test_grants_reload_but_legacy_corrupt_and_duplicate_registry_fail_closed(self):
    grant = self.committed()
    restarted = LocalAuthority("cloud-device", "comma-device", self.device_key, self.writes.append, self.writes[-1])
    self.assertEqual(restarted.grants, {grant.key_id: grant})
    corrupt = grant.to_dict() | {"key_id": "wrong"}
    for storage in ([{"app_id": "legacy", "endpoint": "ws://127.0.0.1"}], {"v": 1, "grants": [grant.to_dict()]},
                    {"v": 2, "grants": [corrupt]}, {"v": 2, "grants": [grant.to_dict(), grant.to_dict()]}):
      self.assertEqual(LocalAuthority("cloud-device", "comma-device", self.device_key, self.writes.append, storage).grants, {})

  def test_revocation_and_closed_session_prevent_handler_execution(self):
    grant = self.committed()
    session = self.authority.session(grant, grant.key_id)
    handler = mock.Mock(return_value="ok")
    self.assertEqual(session.invoke("getParams", handler), "ok")
    self.authority.revoke(grant.key_id)
    with self.assertRaises(AuthorizationError):
      session.invoke("saveParams", handler)
    self.assertEqual(handler.call_count, 1)
    with self.assertRaises(AuthorizationError):
      self.authority.session(grant, grant.key_id)
    with self.assertRaises(AuthorizationError):
      self.authority.enroll(self.enrollment(), Origin.CLOUD)

  def test_wrong_peer_revision_and_denied_methods_cannot_dispatch(self):
    grant = self.committed()
    handler = mock.Mock()
    with self.assertRaises(AuthorizationError):
      self.authority.session(grant, "wrong")
    with self.assertRaises(AuthorizationError):
      self.authority.session(replace(grant, revision=encode64(b"x" * 32)), grant.key_id)
    session = self.authority.session(grant, grant.key_id)
    for method in ("startLocalProxy", "pairLocalApp", "uploadFileToUrl", "startStream", "setNavDestination", "unknown"):
      with self.assertRaises(AuthorizationError):
        session.invoke(method, handler)
    session.close()
    with self.assertRaises(AuthorizationError):
      session.invoke("getMessage", handler)
    handler.assert_not_called()

  def test_same_installation_key_can_enroll_multiple_devices(self):
    original = self.committed()
    self.authority = LocalAuthority("second-cloud-device", "second-comma-device", self.device_key, self.writes.append, clock=lambda: self.now)
    self.qr = self.authority.arm()
    second = self.committed()
    self.assertEqual(original.key_id, second.key_id)
    self.assertNotEqual(original.revision, second.revision)

  def test_canonical_encoding_and_field_limits(self):
    for value in ("", "Zg==", "Zh", "a", "💥", 123, "Z g", "Zg\n"):
      with self.assertRaises(AuthorizationError):
        decode64(value)
    self.assertEqual(decode64("Zg"), b"f")
    with self.assertRaises(AuthorizationError):
      Grant.from_dict({})

  def test_pinned_connector_rejects_downgrade_and_invalid_endpoints_before_dial(self):
    with mock.patch("openpilot.sunnypilot.sunnylink.athena.local_auth_v2.socket.create_connection") as dial:
      for endpoint in ("ws://192.168.1.2:8443", "wss://host.example:8443", "wss://user:pass@192.168.1.2:8443",
                       "wss://0.0.0.0:8443", "wss://224.0.0.1:8443", "wss://192.168.1.2:8443/path",
                       "wss://192.168.1.2:8443?token=secret", "wss://192.168.1.2:99999"):
        with self.subTest(endpoint=endpoint), self.assertRaises(AuthorizationError):
          connect_pinned(endpoint, key_id(self.app_key.public_key()))
      dial.assert_not_called()

  def test_wrong_tls_pin_closes_socket_before_websocket_upgrade(self):
    base = "openpilot.sunnypilot.sunnylink.athena.local_auth_v2."
    with mock.patch(base + "socket.create_connection"), mock.patch(base + "ssl.SSLContext") as context, \
         mock.patch(base + "verify_tls_peer", side_effect=AuthorizationError("wrong key")), mock.patch(base + "create_connection") as upgrade:
      transport = context.return_value.wrap_socket.return_value
      transport.getpeercert.return_value = b"certificate"
      with self.assertRaises(AuthorizationError):
        connect_pinned("wss://192.168.1.2:8443", key_id(self.app_key.public_key()))
      transport.close.assert_called_once()
      upgrade.assert_not_called()

  def test_real_tls_connector_pins_before_http_and_requires_tls13(self):
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "test phone")])
    now = datetime.now(UTC)
    certificate = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(self.app_key.public_key())
                   .serial_number(x509.random_serial_number()).not_valid_before(now - timedelta(minutes=1))
                   .not_valid_after(now + timedelta(days=1)).sign(self.app_key, hashes.SHA256()))
    with tempfile.TemporaryDirectory() as temporary:
      cert_path = Path(temporary) / "certificate.pem"
      key_path = Path(temporary) / "private.pem"
      cert_path.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))
      key_path.write_bytes(self.app_key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                                    serialization.NoEncryption()))
      context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
      context.minimum_version = ssl.TLSVersion.TLSv1_3
      context.load_cert_chain(str(cert_path), str(key_path))
      for valid in (True, False):
        with self.subTest(valid=valid), socket.socket() as listener:
          listener.bind(("127.0.0.1", 0))
          listener.listen(1)
          listener.settimeout(5)
          received = []
          errors = []

          def serve(listener=listener, received=received, errors=errors):
            try:
              incoming, _ = listener.accept()
              with incoming, context.wrap_socket(incoming, server_side=True) as tls:
                tls.settimeout(5)
                self.assertEqual(tls.version(), "TLSv1.3")
                request = b""
                while b"\r\n\r\n" not in request:
                  piece = tls.recv(4096)
                  if not piece:
                    break
                  request += piece
                received.append(request)
                if not request:
                  return
                headers = dict(line.split(": ", 1) for line in request.decode().split("\r\n")[1:] if ": " in line)
                accept = base64.b64encode(hashlib.sha1((headers["Sec-WebSocket-Key"] +
                                                       "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
                tls.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n" +
                             f"Sec-WebSocket-Accept: {accept}\r\nSec-WebSocket-Protocol: sunnylink-local-v2\r\n\r\n").encode())
                tls.recv(4096)  # Client close frame.
            except Exception as e:
              errors.append(e)

          thread = threading.Thread(target=serve)
          thread.start()
          try:
            endpoint = f"wss://127.0.0.1:{listener.getsockname()[1]}"
            if valid:
              client = connect_pinned(endpoint, key_id(self.app_key.public_key()), timeout_s=5)
              self.assertEqual(client.sock.version(), "TLSv1.3")
              client.close(timeout=1)
            else:
              with self.assertRaises(AuthorizationError):
                connect_pinned(endpoint, key_id(self.device_key.public_key()), timeout_s=5)
          finally:
            thread.join(timeout=6)
          self.assertFalse(thread.is_alive())
          self.assertEqual(errors, [])
          self.assertEqual(len(received), 1)
          if valid:
            self.assertTrue(b"GET / HTTP/1.1" in received[0])
            self.assertNotIn(b"Authorization", received[0])
          else:
            self.assertEqual(received[0], b"")

  def test_tls_pin_checks_key_not_name(self):
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "sunnylink mobile")])
    now = datetime.now(UTC)
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(self.app_key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(now - timedelta(minutes=1)).not_valid_after(now + timedelta(days=1))
            .sign(self.app_key, hashes.SHA256()).public_bytes(serialization.Encoding.DER))
    self.assertEqual(verify_tls_peer(cert, key_id(self.app_key.public_key())), key_id(self.app_key.public_key()))
    for bad, expected in ((cert, key_id(self.device_key.public_key())), (b"garbage", key_id(self.app_key.public_key()))):
      with self.assertRaises(AuthorizationError):
        verify_tls_peer(bad, expected)
