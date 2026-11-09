"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import random
import time

import raylib_nb as rl

from openpilot.common.hardware import HARDWARE
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.system.ui.sunnypilot.widgets.jokes import JOKES
from openpilot.system.ui.widgets import Widget

# Odyssey: a joke screensaver instead of the bouncing logo. Each joke shows its setup, then the punchline, then fades
# out, and the next one appears somewhere else on the screen, so nothing stays lit in one spot (burn-in).
SETUP_S, PUNCHLINE_S, FADE_S = 4.0, 7.0, 0.6
JOKE_S = SETUP_S + PUNCHLINE_S


class ScreenSaverSP(Widget):
  def __init__(self, params: Params | None = None):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._params = params or Params()
    self._is_mici = HARDWARE.get_device_type() == 'mici' or (HARDWARE.get_device_type() == "pc" and os.getenv("BIG") != "1")

    self._setup_size = 34 if self._is_mici else 84
    self._punch_size = 40 if self._is_mici else 104
    self._order: list[int] = []
    self._joke = JOKES[0]
    self._joke_start = 0.0
    self._pos = rl.Vector2(0, 0)
    self._start_time = None
    self._dismiss = False
    self._screensaver_timeout = 300

  @property
  def is_active(self) -> bool:
    return self._start_time is not None and not self._dismiss

  @property
  def was_dismissed(self) -> bool:
    return self._dismiss

  def initialize(self):
    self._screensaver_timeout = self._params.get("ScreenSaverTimeout", return_default=True)
    if self._start_time is None:
      self._start_time = time.monotonic()
      self._next_joke()
    self._dismiss = False

  def hide_event(self):
    super().hide_event()
    self._dismiss = False
    self._start_time = None

  def _handle_mouse_release(self, mouse_pos):
    self._dismiss = True
    self._start_time = None
    gui_app.pop_widget()
    return super()._handle_mouse_release(mouse_pos)

  def _next_joke(self):
    if not self._order:  # every joke once before any repeats
      self._order = random.sample(range(len(JOKES)), len(JOKES))
    self._joke = JOKES[self._order.pop()]
    self._joke_start = time.monotonic()
    self._pos = None  # placed on the first render, once the text size is known

  def _lines(self, rect: rl.Rectangle):
    width = int(rect.width * 0.72)
    setup = wrap_text(gui_app.font(FontWeight.MEDIUM), self._joke[0], self._setup_size, width)
    punch = wrap_text(gui_app.font(FontWeight.BOLD), self._joke[1], self._punch_size, width)
    return setup, punch

  def _update_state(self):
    super()._update_state()
    if self._start_time and time.monotonic() - self._start_time > self._screensaver_timeout:
      self._dismiss = True
      self._start_time = None
    if time.monotonic() - self._joke_start > JOKE_S:
      self._next_joke()

  def _render(self, rect: rl.Rectangle):
    self.set_rect(rect)
    rl.clear_background(rl.BLACK)
    setup, punch = self._lines(rect)
    setup_h, punch_h = self._setup_size * 1.2, self._punch_size * 1.15
    gap = self._setup_size * 0.9
    block_w = max(measure_text_cached(gui_app.font(FontWeight.BOLD), ln, self._punch_size).x for ln in punch)
    block_w = max(block_w, max(measure_text_cached(gui_app.font(FontWeight.MEDIUM), ln, self._setup_size).x for ln in setup))
    block_h = len(setup) * setup_h + gap + len(punch) * punch_h
    if self._pos is None:
      margin = rect.width * 0.04
      self._pos = rl.Vector2(random.uniform(margin, max(margin, rect.width - block_w - margin)),
                             random.uniform(margin, max(margin, rect.height - block_h - margin)))

    t = time.monotonic() - self._joke_start
    fade = min(1.0, t / FADE_S, (JOKE_S - t) / FADE_S)
    punch_fade = min(1.0, max(0.0, (t - SETUP_S) / FADE_S)) * fade
    x, y = rect.x + self._pos.x, rect.y + self._pos.y
    for ln in setup:
      rl.draw_text_ex(gui_app.font(FontWeight.MEDIUM), ln, rl.Vector2(x, y), self._setup_size, 0, rl.Color(235, 235, 235, int(255 * fade)))
      y += setup_h
    y += gap
    for ln in punch:
      rl.draw_text_ex(gui_app.font(FontWeight.BOLD), ln, rl.Vector2(x, y), self._punch_size, 0, rl.Color(255, 196, 0, int(255 * punch_fade)))
      y += punch_h
    return -1
