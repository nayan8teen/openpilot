import unittest
from multiprocessing import Process
from typing import cast

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.manager.process import PythonProcess


def always_run(started, params, CP):
  return True


class FakeProc:
  def __init__(self, exitcode):
    self.exitcode = exitcode


class TestProcessSupervision(OpenpilotTestCase):
  """Dead processes that should be running used to linger in the manager state
  (shouldBeRunning=True, running=False) until the next start/stop cycle, which
  selfdrived reports as processNotRunning and blocks engagement."""

  def _make_process(self, supervised, exitcode=None):
    p = PythonProcess("test_proc", "openpilot.system.micd", always_run, supervised=supervised)
    # ty can't know FakeProc structurally satisfies the Process handle surface
    # we rely on (just .exitcode); cast like other tests in this repo
    p.proc = cast(Process, FakeProc(exitcode))
    return p

  def test_unsupervised_dead_process_is_not_reaped(self):
    p = self._make_process(supervised=False, exitcode=1)
    assert not p._reap_if_dead()
    assert p.proc is not None

  def test_supervised_dead_process_is_reaped(self):
    p = self._make_process(supervised=True, exitcode=1)
    assert p._reap_if_dead()
    assert p.proc is None

  def test_supervised_alive_process_is_left_alone(self):
    p = self._make_process(supervised=True, exitcode=None)
    assert not p._reap_if_dead()
    assert p.proc is not None

  def test_supervised_with_no_proc_starts_normally(self):
    p = self._make_process(supervised=True, exitcode=None)
    p.proc = None
    assert p._reap_if_dead()

  def test_config_flags(self):
    from openpilot.system.manager.process_config import managed_processes
    assert managed_processes['micd'].supervised
    assert managed_processes['soundd'].supervised
    assert not managed_processes['controlsd'].supervised


if __name__ == "__main__":
  unittest.main()
