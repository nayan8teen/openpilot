
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.soundd import FILTER_DT, Soundd
from openpilot.common.filter_simple import FirstOrderFilter


class TestSounddMicValidity(OpenpilotTestCase):
  """soundd must never adapt its volume from stale mic data.

  When micd can't open the mic (commaai/openpilot#36088) soundPressure
  flatlines at zero. soundd used to keep feeding that 0 dB into the ambient
  volume filter, decaying alert volume to MIN_VOLUME and effectively
  silencing safety-critical alerts.
  """

  def _make_soundd(self):
    s = Soundd.__new__(Soundd)  # skip QuietMode/params + sound file loading
    s.current_alert = 0
    s.current_volume = 0.5
    s.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)  # like soundd
    s.spl_received = False
    return s

  def _apply(self, s, valid, db=0.0):
    # exercise the real production logic (called from soundd_thread's loop)
    s.update_volume(valid, db)

  def test_valid_data_still_controls_volume(self):
    s = self._make_soundd()
    start = s.current_volume
    self._apply(s, valid=True, db=90.0)  # loud ambient -> higher volume
    assert s.spl_received
    assert s.current_volume > start

  def test_invalid_data_does_not_decay_volume(self):
    s = self._make_soundd()
    start = s.current_volume
    for _ in range(200):  # 10 s @ 20 Hz of stale zero readings
      self._apply(s, valid=False, db=0.0)
    assert not s.spl_received
    assert s.current_volume == start

  def test_volume_resumes_adapting_after_stream_recovers(self):
    s = self._make_soundd()
    self._apply(s, valid=True, db=90.0)  # loud ambient -> volume saturates at MAX
    frozen = s.current_volume
    for _ in range(100):
      self._apply(s, valid=False, db=0.0)  # mic down: volume must stay frozen
    assert s.current_volume == frozen
    for _ in range(100):
      self._apply(s, valid=True, db=20.0)  # quiet ambient: volume comes back down
    assert s.current_volume < frozen


if __name__ == "__main__":
  import unittest
  unittest.main()
