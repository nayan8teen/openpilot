"""Tests for micd's invalid-message and stream-recovery behavior.

When the mic device can't be opened (commaai/openpilot#36088), micd used to
keep broadcasting valid=True soundPressure flatlined at zero, which made
soundd decay the alert volume to silence. These tests pin the new contract:
messages are marked invalid while the stream is down, and the stream is
re-opened automatically.
"""
import unittest
from unittest import mock

import numpy as np

import openpilot.system.micd as micd
from openpilot.common.test import OpenpilotTestCase


class TestMicdInvalidMessages(OpenpilotTestCase):
  def _make_mic(self, stream_active):
    m = micd.Mic()
    m.pm = mock.MagicMock()
    m.stream = mock.MagicMock()
    m.stream.active = stream_active
    return m

  def test_sound_pressure_message_valid_with_live_stream(self):
    m = self._make_mic(stream_active=True)
    m.update()
    msg = m.pm.send.call_args[0][1]
    assert msg.valid

  def test_sound_pressure_message_invalid_without_stream(self):
    m = self._make_mic(stream_active=False)
    m._reinit_stream = mock.MagicMock()
    m.update()
    msg = m.pm.send.call_args[0][1]
    assert not msg.valid

  def test_update_attempts_stream_recovery_when_down(self):
    m = self._make_mic(stream_active=False)
    m._reinit_stream = mock.MagicMock()
    m.update()
    m._reinit_stream.assert_called_once()

  def test_update_survives_recovery_failure(self):
    m = self._make_mic(stream_active=False)
    m._reinit_stream = mock.MagicMock(side_effect=RuntimeError("device gone"))
    m.update()  # must not raise
    m._reinit_stream.assert_called_once()

  def test_raw_audio_data_invalid_on_input_overflow(self):
    m = self._make_mic(stream_active=True)
    m.pm = mock.MagicMock()
    m.callback(np.zeros((10, 1), dtype=np.float32), 10, None, 0)
    msg = m.pm.send.call_args[0][1]
    assert msg.valid

    m.callback(np.zeros((10, 1), dtype=np.float32), 10, None, 1)  # inputOverflow
    msg = m.pm.send.call_args[0][1]
    assert not msg.valid


if __name__ == "__main__":
  unittest.main()
