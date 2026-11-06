"""
Steering position indicator: where the steering wheel is between full left and full right lock.

The lock-to-lock extents are learned per side from carState.steeringAngleDeg and persisted
(SteerExtentLeftDeg / SteerExtentRightDeg), so the bar's ends are this car's real full lock.
The bar uses a signed square-root scale, so the few degrees of highway steering still visibly
move the marker while full lock still lands on the ends.
"""
import math

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

FALLBACK_EXTENT_DEG = 450.0  # used for a side until it has been learned
MIN_EXTENT_DEG = 90.0        # learned values below this aren't a lock, they're just a turn
MAX_EXTENT_DEG = 1000.0      # anything past this is a sensor glitch
DWELL_FRAMES = 6             # angle must hold past the old extent this long (~0.3 s at 20 fps)
PERSIST_STEP_DEG = 2.0       # write to params only when an extent grows by at least this
PERSIST_MIN_INTERVAL_FRAMES = 20  # ~1 s at 20 fps; writes happen only when an extent grows

PARAM_LEFT = "SteerExtentLeftDeg"
PARAM_RIGHT = "SteerExtentRightDeg"


class SteerExtentLearner:
  """Running maximum of the steering angle on each side, debounced and persisted.

  Positive steeringAngleDeg is left in openpilot's convention.
  """

  def __init__(self, left: float = 0.0, right: float = 0.0):
    self.left = left if MIN_EXTENT_DEG <= left <= MAX_EXTENT_DEG else 0.0
    self.right = right if MIN_EXTENT_DEG <= right <= MAX_EXTENT_DEG else 0.0
    self._dwell = 0
    self._dwell_peak = 0.0
    self._dwell_left = True
    self._persisted = (self.left, self.right)
    self._frames_since_persist = PERSIST_MIN_INTERVAL_FRAMES

  def update(self, angle_deg: float) -> bool:
    """Feed one sample. Returns True when the extents should be written to params."""
    self._frames_since_persist += 1
    if not math.isfinite(angle_deg):
      self._dwell = 0
      return False

    magnitude = abs(angle_deg)
    is_left = angle_deg > 0
    current = self.left if is_left else self.right
    if MIN_EXTENT_DEG <= magnitude <= MAX_EXTENT_DEG and magnitude > current:
      if self._dwell == 0 or is_left != self._dwell_left:
        self._dwell = 0
        self._dwell_peak = 0.0
        self._dwell_left = is_left
      self._dwell += 1
      self._dwell_peak = max(self._dwell_peak, magnitude)
      if self._dwell >= DWELL_FRAMES:
        if is_left:
          self.left = max(self.left, self._dwell_peak)
        else:
          self.right = max(self.right, self._dwell_peak)
    else:
      self._dwell = 0
      self._dwell_peak = 0.0

    return self._should_persist()

  def _should_persist(self) -> bool:
    grew = (self.left - self._persisted[0] >= PERSIST_STEP_DEG) or (self.right - self._persisted[1] >= PERSIST_STEP_DEG)
    return grew and self._frames_since_persist >= PERSIST_MIN_INTERVAL_FRAMES

  def mark_persisted(self) -> None:
    self._persisted = (self.left, self.right)
    self._frames_since_persist = 0

  def extents(self) -> tuple[float, float]:
    return (self.left or FALLBACK_EXTENT_DEG, self.right or FALLBACK_EXTENT_DEG)

  def learned(self) -> tuple[bool, bool]:
    return (self.left > 0.0, self.right > 0.0)


def position_fraction(angle_deg: float, extent_left: float, extent_right: float) -> float:
  """Map an angle to [-1, 1] (left negative on screen), signed square-root scale, clamped."""
  if angle_deg >= 0:
    frac = -math.sqrt(min(angle_deg / max(extent_left, 1e-3), 1.0))
  else:
    frac = math.sqrt(min(-angle_deg / max(extent_right, 1e-3), 1.0))
  return frac


def draw_steering_position(rect: rl.Rectangle, angle_deg: float, extent_left: float, extent_right: float,
                           learned: tuple[bool, bool], color: rl.Color, font: rl.Font, show_text: bool = True) -> None:
  """Draw the indicator centred horizontally near the top of rect, under the current speed."""
  width = min(640.0, rect.width * 0.4)
  height = 16.0
  cx = rect.x + rect.width / 2
  y = rect.y + 345.0  # below the speed unit text, which is centred at y=290
  x0 = cx - width / 2

  track = rl.Rectangle(x0, y, width, height)
  rl.draw_rectangle_rounded(track, 1.0, 10, rl.Color(0, 0, 0, 110))
  rl.draw_rectangle_rounded_lines_ex(track, 1.0, 10, 2, rl.Color(255, 255, 255, 70))

  # reference ticks at 90 and 180 degrees each side, plus centre
  for deg in (90.0, 180.0):
    for sign in (1.0, -1.0):
      ext = extent_left if sign > 0 else extent_right
      if deg < ext:
        tx = cx + position_fraction(sign * deg, extent_left, extent_right) * width / 2
        rl.draw_line_ex(rl.Vector2(tx, y + 3), rl.Vector2(tx, y + height - 3), 2, rl.Color(255, 255, 255, 60))
  rl.draw_line_ex(rl.Vector2(cx, y - 6), rl.Vector2(cx, y + height + 6), 3, rl.Color(255, 255, 255, 150))

  # end caps: solid once that side's lock is learned, faint until then
  for side_x, is_learned in ((x0, learned[0]), (x0 + width, learned[1])):
    alpha = 200 if is_learned else 60
    rl.draw_line_ex(rl.Vector2(side_x, y - 8), rl.Vector2(side_x, y + height + 8), 4, rl.Color(255, 255, 255, alpha))

  # fill from centre to the current position, then the marker
  frac = position_fraction(angle_deg, extent_left, extent_right)
  mx = cx + frac * width / 2
  fill = rl.Rectangle(min(cx, mx), y + 3, abs(mx - cx), height - 6)
  rl.draw_rectangle_rec(fill, rl.Color(color.r, color.g, color.b, 160))
  rl.draw_circle(int(mx), int(y + height / 2), 14, color)
  rl.draw_circle_lines(int(mx), int(y + height / 2), 14, rl.Color(0, 0, 0, 160))

  if show_text:
    text = f"{abs(angle_deg):.0f}°"
    size = 34
    tw = measure_text_cached(font, text, size, 0).x
    rl.draw_text_ex(font, text, rl.Vector2(cx - tw / 2, y + height + 10), size, 0, rl.Color(255, 255, 255, 200))


class SteeringPositionIndicator(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._learner = SteerExtentLearner(self._read_param(PARAM_LEFT), self._read_param(PARAM_RIGHT))
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._angle = 0.0
    self._last_frame = -1

  def _read_param(self, key: str) -> float:
    try:
      return float(self._params.get(key, return_default=True) or 0.0)
    except (TypeError, ValueError):
      return 0.0

  def _update_state(self) -> None:
    sm = ui_state.sm
    frame = sm.recv_frame['carState']
    if frame < ui_state.started_frame or frame == self._last_frame:
      return
    self._last_frame = frame
    self._angle = sm['carState'].steeringAngleDeg
    if self._learner.update(self._angle):
      self._params.put(PARAM_LEFT, float(self._learner.left))
      self._params.put(PARAM_RIGHT, float(self._learner.right))
      self._learner.mark_persisted()

  def _render(self, rect: rl.Rectangle) -> None:
    if ui_state.status == UIStatus.ENGAGED:
      color = rl.Color(128, 216, 166, 255)
    elif ui_state.status == UIStatus.OVERRIDE or ui_state.sm['carState'].steeringPressed:
      color = rl.Color(145, 155, 149, 255)
    elif ui_state.status == UIStatus.LAT_ONLY:
      color = rl.Color(0x00, 0xC8, 0xC8, 0xFF)
    else:
      color = rl.Color(235, 235, 235, 255)
    left, right = self._learner.extents()
    draw_steering_position(rect, self._angle, left, right, self._learner.learned(), color, self._font)
