import os
import pyray as rl

from cereal import car, custom
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog, BigConfirmationDialogV2
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.mici_setup import TermsHeader, TermsPage as SetupTermsPage
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.slider import SmallSlider
from openpilot.sunnypilot.sunnylink.backups.manager import get_local_backup_path


class LocalRestoreDialog(SetupTermsPage):
  def __init__(self):
    super().__init__(self._restore_params, self._cancel_callback, "cancel", "restore")
    self._continue_button = SmallSlider(self._continue_text, confirm_callback=self._restore_params)
    self._continue_slider = True

    self._prev_offroad_mode = ui_state.params.get("OffroadMode")
    prev_cp_bytes = ui_state.params.get("CarParamsPrevRoute")
    cp_bytes = ui_state.params.get("CarParams")

    self._old_fingerprint: str | None = None
    self._new_fingerprint: str | None = None

    if prev_cp_bytes:
      with car.CarParams.from_bytes(prev_cp_bytes) as prev_cp:
        self._old_fingerprint = prev_cp.carFingerprint

    if cp_bytes:
      with car.CarParams.from_bytes(cp_bytes) as cp:
        self._new_fingerprint = cp.carFingerprint

    cloudlog.error(f"old fingerprint: {self._old_fingerprint}, new fingerprint: {self._new_fingerprint}")

    self._switch_detected: bool = self._old_fingerprint and self._new_fingerprint and self._old_fingerprint != self._new_fingerprint

    self._title_header = TermsHeader("car switch detected!",
                                     gui_app.texture("icons_mici/setup/restore.png", 66, 60))
    self._body = UnifiedLabel(f"It looks like you've switched to \"{self._new_fingerprint}\". " +
                              "Would you like to restore params?\n\n" +
                              f"A backup for {self._old_fingerprint} will be saved.\n",
                              font_size=36,
                              text_color=rl.Color(255, 255, 255, int(255 * 0.9)),
                              font_weight=FontWeight.ROMAN)
    self._restart_header = TermsHeader("this will reset calibration & require a restart!",
                                     gui_app.texture("icons_mici/setup/warning.png", 66, 60))

  @property
  def car_switch_detected(self):
    backup_exists = False
    if self._new_fingerprint:
      backup_file = get_local_backup_path(self._new_fingerprint)
      print(f"backup file: {backup_file}")
      backup_exists = os.path.isfile(backup_file)
    return (self._old_fingerprint
            and self._new_fingerprint
            and self._old_fingerprint != self._new_fingerprint
            and backup_exists)

  @property
  def _content_height(self):
    return self._restart_header.rect.y + self._restart_header.rect.height - self._scroll_panel.get_offset() + 20

  def _render(self, _):
    super()._render(_)
    return -1

  def _render_content(self, scroll_offset):
    self._title_header.set_position(self._rect.x + 16, self._rect.y + 8 + scroll_offset)
    self._title_header.render()

    body_rect = rl.Rectangle(
      self._rect.x + 8,
      self._title_header.rect.y + self._title_header.rect.height + self.ITEM_SPACING,
      self._rect.width - 50,
      self._body.get_content_height(int(self._rect.width - 50)),
    )
    self._body.render(body_rect)

    self._restart_header.set_position(self._rect.x + 16, self._body.rect.y + self._body.rect.height + self.ITEM_SPACING)
    self._restart_header.render()

  def _restore_params(self):
    def _reset_dlg():
      self.reset()
      self._continue_button.reset()
      gui_app.set_modal_overlay(self)

    def _error():
      self.reset()
      gui_app.set_modal_overlay(BigDialog(title=tr("unable to restore"), description="try again later"))

    def _success():
      # reset calibration
      self._params.remove("CalibrationParams")
      self._params.remove("LiveTorqueParameters")
      self._params.remove("LiveParameters")
      self._params.remove("LiveParametersV2")
      self._params.remove("LiveDelay")
      # restart ui
      gui_app.request_close()

    if ui_state.engaged:
      gui_app.set_modal_overlay(BigDialog(title="",
                                          description="disengage sunnypilot to restore params",
                                          right_btn="back",
                                          right_btn_callback=_reset_dlg))
    else:
      self._prev_offroad_mode = ui_state.params.get("OffroadMode")
      ui_state.params.put("OffroadMode", True)
      ui_state.params.put("BackupManager_LocalBackup", self._old_fingerprint)
      while self._backup_status == custom.BackupManagerSP.Status.inProgress:
        continue
      if self._backup_status == custom.BackupManagerSP.Status.failed:
        _error()
      else:
        ui_state.params.put("BackupManager_LocalRestore", self._new_fingerprint)
        while self._restore_status == custom.BackupManagerSP.Status.inProgress:
          continue
        if self._restore_status == custom.BackupManagerSP.Status.failed:
          _error()
        else:
          gui_app.set_modal_overlay(BigConfirmationDialogV2(title="restart ui",
                                                            icon="icons_mici/settings/device/reboot.png",
                                                            red=True), callback=_success)

  def _cancel_callback(self):
    ui_state.params.put("OffroadMode", False if self._prev_offroad_mode is None else self._prev_offroad_mode)
    gui_app.set_modal_overlay(None)

  def _update_state(self):
    sunnylink_backup_manager = ui_state.sm["backupManagerSP"]
    self._backup_status = sunnylink_backup_manager.backupStatus
    self._restore_status = sunnylink_backup_manager.restoreStatus
