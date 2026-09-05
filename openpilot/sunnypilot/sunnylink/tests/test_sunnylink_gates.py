"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot.sunnylink.athena.local_pairing import LOCAL_APPS_KEY, LocalApp, add_local_app
from openpilot.sunnypilot.sunnylink.utils import (
  UNREGISTERED_SUNNYLINK_DONGLE_ID,
  sunnylink_need_register,
  sunnylink_ready,
)

# Params keys these tests write (restored in teardown).
_WRITTEN_KEYS = (LOCAL_APPS_KEY, "SunnylinkEnabled", "SunnylinkDongleId", "SunnylinkTempFault")


class TestSunnylinkGates(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.saved = {key: self.params.get(key) for key in _WRITTEN_KEYS}
    self.params.put_bool("SunnylinkEnabled", True, block=True)
    self.params.put_bool("SunnylinkTempFault", False, block=True)
    self.params.put("SunnylinkDongleId", UNREGISTERED_SUNNYLINK_DONGLE_ID, block=True)

  def teardown_method(self):
    for key, value in self.saved.items():
      if value is None:
        self.params.remove(key)
      else:
        self.params.put(key, value, block=True)

  def test_requires_registration_otherwise(self):
    assert not sunnylink_ready(self.params)
    assert sunnylink_need_register(self.params)

  def test_registered_device_ready(self):
    self.params.put("SunnylinkDongleId", "sunnylink-abc", block=True)
    assert sunnylink_ready(self.params)
    assert not sunnylink_need_register(self.params)

  def test_locally_paired_never_registered_device_is_ready(self):
    # The whole point of local mode: a device that never touched the cloud is
    # fully usable once an app is paired over the LAN.
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    assert sunnylink_ready(self.params)
    assert not sunnylink_need_register(self.params)

  def test_fault_blocks_everything(self):
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    self.params.put_bool("SunnylinkTempFault", True, block=True)
    assert not sunnylink_ready(self.params)
    assert not sunnylink_need_register(self.params)

  def test_disabled_blocks_everything(self):
    add_local_app(LocalApp(app_id="app-1", endpoint="ws://10.0.0.5:8443"), self.params)
    self.params.put_bool("SunnylinkEnabled", False, block=True)
    assert not sunnylink_ready(self.params)
    assert not sunnylink_need_register(self.params)
