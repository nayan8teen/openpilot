import pyray as rl
from typing import Union
from openpilot.system.ui.lib.application import gui_app

from openpilot.selfdrive.ui.mici.widgets.button import BigButton, PRESSED_SCALE, LABEL_COLOR


LABEL_HORIZONTAL_PADDING = 0


class BigImageButton(BigButton):
  def __init__(self, text: str, value: str = "", icon: Union[str, rl.Texture] = "", qr_code: str = ""):
    self._qr = qr_code
    self._qr_size = 200
    self._qr_padding = 10
    super().__init__(text, value, icon)

    # Update label width to avoid overlap with QR code on the right
    available_width = self._rect.width - LABEL_HORIZONTAL_PADDING - self._qr_size - self._qr_padding - 20
    self._label.set_width(int(available_width))
    self._sub_label.set_width(int(available_width))

  def _load_images(self):
    super()._load_images()
    self._qr_texture = gui_app.texture(self._qr, self._qr_size, self._qr_size)

  def _render(self, _):
    scale = self._scale_filter.update(PRESSED_SCALE if self.is_pressed else 1.0)
    btn_y = self._rect.y + (self._rect.height * (1 - scale)) / 2

    # QR CODE -----------------------------------------------------------------
    qr_x = self._rect.x + self._rect.width - self._qr_padding - self._qr_size
    qr_y = self._rect.y + (self._rect.height - self._qr_size) / 2
    rl.draw_texture(self._qr_texture, int(qr_x), int(qr_y), rl.WHITE)

    # LABEL ------------------------------------------------------------------
    lx = self._rect.x + LABEL_HORIZONTAL_PADDING
    ly = btn_y + self._rect.height - 33

    if self.value:
      self._sub_label.set_position(lx, ly)
      ly -= self._sub_label.font_size + 9
      self._sub_label.render()

    label_color = LABEL_COLOR if self.enabled else rl.Color(255, 255, 255, int(255 * 0.35))
    self._label.set_color(label_color)
    self._label.set_position(lx, ly)
    self._label.render()
