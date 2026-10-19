"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.system.ui.widgets.scroller import NavScroller


class LanefulToggle(BigParamControl):
  """Tap to enable or disable laneful, like any other driving toggle."""
  def __init__(self):
    super().__init__("laneful", "LanefulEnabled",
                     description="Nudges the car toward lane center using the model's lane lines.\n" +
                                 "On an unmarked two-lane road it centers in the right half.")


class LanefulLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._toggle = LanefulToggle()
    self._scroller.add_widget(self._toggle)

  def show_event(self):
    super().show_event()
    self._toggle.refresh()
