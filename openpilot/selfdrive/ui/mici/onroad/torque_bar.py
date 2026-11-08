import math
import time
from functools import wraps
from collections import OrderedDict

import numpy as np
import raylib_nb as rl
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter

# TODO: arc_bar_pts doesn't consider rounded end caps part of the angle span
TORQUE_ANGLE_SPAN = 12.7  # degrees on the default 476 px camera view; narrower views beside the navigation panel scale down

DEBUG = False


def quantized_lru_cache(maxsize=128):
  def decorator(func):
    cache = OrderedDict()
    @wraps(func)
    def wrapper(r_mid, thickness, a0_deg, a1_deg, **kwargs):
      # Quantize inputs: balanced for smoothness vs cache effectiveness
      key = (round(r_mid),
             round(thickness),           # 1px precision for smoother height transitions
             round(a0_deg * 10) / 10,    # 0.1° precision for smoother angle transitions
             round(a1_deg * 10) / 10,
             tuple(sorted(kwargs.items())))

      if key in cache:
        cache.move_to_end(key)
      else:
        if len(cache) >= maxsize:
          cache.popitem(last=False)

        result = func(r_mid, thickness, a0_deg, a1_deg, **kwargs)
        cache[key] = result
      return cache[key]
    return wrapper
  return decorator


@quantized_lru_cache(maxsize=256)
def arc_bar_pts(r_mid: float, thickness: float,
                a0_deg: float, a1_deg: float,
                *, max_points: int = 100, cap_segs: int = 10,
                cap_radius: float = 7, px_per_seg: float = 2.0) -> np.ndarray:
  """Return Nx2 np.float32 points for a single closed polygon (rounded thick arc), centered at origin."""

  def get_cap(left: bool, a_deg: float):
    # end cap at a1: center (a1), sweep a1→a1+180 (skip endpoints to avoid dupes)
    # quarter arc (outer corner) at a1 with fixed pixel radius cap_radius

    nx, ny = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))  # outward normal
    tx, ty = -ny, nx  # tangent (CCW)

    mx, my = nx * r_mid, ny * r_mid  # mid-point at a1
    if DEBUG:
      rl.draw_circle(int(mx), int(my), 4, rl.PURPLE)

    ex = mx + nx * (half - cap_radius)
    ey = my + ny * (half - cap_radius)

    if DEBUG:
      rl.draw_circle(int(ex), int(ey), 2, rl.WHITE)

    # sweep 90° in the local (t,n) frame: from outer edge toward inside
    if not left:
      alpha = np.deg2rad(np.linspace(90, 0, cap_segs + 2))[1:-1]
    else:
      alpha = np.deg2rad(np.linspace(180, 90, cap_segs + 2))[1:-1]
    cap_end = np.c_[ex + np.cos(alpha) * cap_radius * tx + np.sin(alpha) * cap_radius * nx,
                    ey + np.cos(alpha) * cap_radius * ty + np.sin(alpha) * cap_radius * ny]

    # bottom quarter (inner corner) at a1
    ex2 = mx + nx * (-half + cap_radius)
    ey2 = my + ny * (-half + cap_radius)
    if DEBUG:
      rl.draw_circle(int(ex2), int(ey2), 2, rl.WHITE)

    if not left:
      alpha2 = np.deg2rad(np.linspace(0, -90, cap_segs + 1))[:-1]  # include 0 once, exclude -90
    else:
      alpha2 = np.deg2rad(np.linspace(90 - 90 - 90, 0 - 90 - 90, cap_segs + 1))[:-1]
    cap_end_bot = np.c_[ex2 + np.cos(alpha2) * cap_radius * tx + np.sin(alpha2) * cap_radius * nx,
                        ey2 + np.cos(alpha2) * cap_radius * ty + np.sin(alpha2) * cap_radius * ny]

    # append to the top quarter
    if not left:
      cap_end = np.vstack((cap_end, cap_end_bot))
    else:
      cap_end = np.vstack((cap_end_bot, cap_end))

    return cap_end

  if a1_deg < a0_deg:
    a0_deg, a1_deg = a1_deg, a0_deg
  half = thickness * 0.5

  cap_radius = min(cap_radius, half)

  span = max(1e-3, a1_deg - a0_deg)

  # pick arc segment count from arc length, clamp to shader points[] budget
  arc_len = r_mid * math.radians(span)
  arc_segs = max(6, int(arc_len / px_per_seg))
  max_arc = (max_points - (4 * cap_segs + 3)) // 2
  arc_segs = max(6, min(arc_segs, max_arc))

  # outer arc a0→a1
  ang_o = np.deg2rad(np.linspace(a0_deg, a1_deg, arc_segs + 1))
  outer = np.c_[np.cos(ang_o) * (r_mid + half),
                np.sin(ang_o) * (r_mid + half)]

  # end cap at a1
  cap_end = get_cap(False, a1_deg)

  # inner arc a1→a0
  ang_i = np.deg2rad(np.linspace(a1_deg, a0_deg, arc_segs + 1))
  inner = np.c_[np.cos(ang_i) * (r_mid - half),
                np.sin(ang_i) * (r_mid - half)]

  # start cap at a0
  cap_start = get_cap(True, a0_deg)

  pts = np.vstack((outer, cap_end, inner, cap_start, outer[:1])).astype(np.float32)

  # Rotate to start from middle of cap for proper triangulation
  pts = np.roll(pts, cap_segs, axis=0)

  if DEBUG:
    n = len(pts)
    idx = int(time.monotonic() * 12) % max(1, n)  # speed: 12 pts/sec
    for i, (x, y) in enumerate(pts):
      j = (i - idx) % n  # rotate the gradient
      t = j / n
      color = rl.Color(255, int(255 * (1 - t)), int(255 * t), 255)
      rl.draw_circle(int(x), int(y), 2, color)

  return pts


DEFAULT_MAX_LAT_ACCEL = 3.0  # m/s^2

# Meter style: zones along each half of the arc, as fractions of the end stop, like the colour bands on a VU meter.
METER_ZONES = (
  (0.0, 0.6, (255, 255, 255), 0.22),  # (start, end, colour, alpha of the unlit track)
  (0.6, 0.85, (255, 200, 0), 0.28),
  (0.85, 1.0, (255, 60, 0), 0.32),
)


class TorqueBar(Widget):
  def __init__(self, scale: float = 1.0, always: bool = False,
               fixed_height: float | None = None, center_dot: bool = True, meter: bool = False):
    super().__init__()
    self._scale = scale
    self._always = always
    # fixed_height (unscaled px): keep the arc one thickness instead of swelling as torque rises past 50%
    self._fixed_height = fixed_height
    self._center_dot = center_dot
    # meter: draw the arc as separate white, yellow and red zones with square seams; the unlit track shows
    # where each zone starts, and the fill runs through each zone in its colour to the actual value.
    self._meter = meter
    self._at_limit = False  # commanded effort is pinned at 100%, the state that leads to the take-over alert
    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._torque_line_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def _update_state(self):
    # torque line
    if ui_state.sm['controlsState'].lateralControlState.which() in ('angleState', 'curvatureState'):
      controls_state = ui_state.sm['controlsState']
      car_state = ui_state.sm['carState']
      vehicle_parameters = ui_state.sm['vehicleParameters']
      car_control = ui_state.sm['carControl']

      # Include lateral accel error in estimated torque utilization
      actual_lateral_accel = controls_state.curvature * car_state.vEgo ** 2
      desired_lateral_accel = controls_state.desiredCurvature * car_state.vEgo ** 2
      accel_diff = (desired_lateral_accel - actual_lateral_accel)

      # Include road roll in estimated torque utilization
      # Roll is less accurate near standstill, so reduce its effect at low speed
      roll_compensation = vehicle_parameters.roll * ACCELERATION_DUE_TO_GRAVITY * np.interp(car_state.vEgo, [5, 15], [0.0, 1.0])
      lateral_acceleration = actual_lateral_accel - roll_compensation
      max_lateral_acceleration = ui_state.CP.maxLateralAccel if ui_state.CP else DEFAULT_MAX_LAT_ACCEL

      if not car_control.latActive:
        raw = 0.0
      else:
        raw = (lateral_acceleration + accel_diff) / max_lateral_acceleration
    else:
      raw = -ui_state.sm['carOutput'].actuatorsOutput.torque
    self._at_limit = abs(raw) >= 0.999
    self._torque_filter.update(float(np.clip(raw, -1, 1)))

  def _render(self, rect: rl.Rectangle) -> None:
    # adjust y pos with torque
    if self._fixed_height is not None:
      torque_line_offset = 22 * self._scale
      torque_line_height = self._fixed_height * self._scale
    else:
      torque_line_offset = np.interp(abs(self._torque_filter.x), [0.5, 1], [22 * self._scale, 26 * self._scale])
      torque_line_height = np.interp(abs(self._torque_filter.x), [0.5, 1], [14 * self._scale, 56 * self._scale])

    # animate alpha and angle span
    self._torque_line_alpha_filter.update(ui_state.status not in (UIStatus.DISENGAGED, UIStatus.LONG_ONLY))

    torque_line_bg_alpha = np.interp(abs(self._torque_filter.x), [0.5, 1.0], [0.25, 0.5])
    torque_line_bg_color = rl.Color(255, 255, 255, int(255 * torque_line_bg_alpha * self._torque_line_alpha_filter.x))
    if ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      torque_line_bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * self._torque_line_alpha_filter.x))

    # draw curved line polygon torque bar
    torque_line_radius = 1200 * self._scale
    top_angle = -90
    torque_bg_angle_span = self._torque_line_alpha_filter.x * TORQUE_ANGLE_SPAN * min(1.0, rect.width / (476 * self._scale))
    torque_start_angle = top_angle - torque_bg_angle_span / 2
    torque_end_angle = top_angle + torque_bg_angle_span / 2
    # centerline radius & center (you already have these values)
    mid_r = torque_line_radius + torque_line_height / 2

    cx = rect.x + rect.width / 2 + 8  # offset 8px to right of camera feed
    cy = rect.y + rect.height + torque_line_radius - torque_line_offset
    offset = np.array([cx, cy], dtype=np.float32)

    if self._meter:
      engaged = ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY)
      self._draw_meter(rect, offset, mid_r, torque_line_height, top_angle, torque_bg_angle_span / 2, engaged)
      return

    # draw bg torque indicator line
    bg_pts = arc_bar_pts(mid_r, torque_line_height, torque_start_angle, torque_end_angle, cap_radius=7 * self._scale) + offset
    draw_polygon(rect, bg_pts, color=torque_line_bg_color)

    # draw torque indicator line
    a0s = top_angle
    a1s = a0s + torque_bg_angle_span / 2 * self._torque_filter.x
    sl_pts = arc_bar_pts(mid_r, torque_line_height, a0s, a1s, cap_radius=7 * self._scale) + offset

    # Colour by position along the arc, not by overall effort: the normal range near the centre
    # stays white and only the part of the bar that reaches out past it warms to yellow, then a
    # deep orange at the end stop, so the darkest colour is always at the tip.
    tip_x = min(bg_pts[:, 0]) if self._torque_filter.x < 0 else max(bg_pts[:, 0])
    alpha = self._torque_line_alpha_filter.x
    colors = [
      rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),  # centre: normal effort
      rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),
      rl.Color(255, 200, 0, int(255 * alpha)),  # yellow
      rl.Color(255, 90, 0, int(255 * alpha)),  # deep orange at the end stop
    ]
    stops = [0.0, 0.5, 0.75, 1.0]

    if ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      colors = [rl.Color(255, 255, 255, int(255 * 0.35 * alpha))] * 2
      stops = [0.0, 1.0]

    # The shader puts colors[0] at `end` and the last colour at `start`, and it measures in screen
    # pixels from origin_rect's corner, so pass the tip as start and the centre as end, both
    # relative to rect.
    gradient = Gradient(
      start=((tip_x - rect.x) / rect.width, 0),
      end=((cx - rect.x) / rect.width, 0),
      colors=colors,
      stops=stops,
    )

    draw_polygon(rect, sl_pts, gradient=gradient)

    # draw center torque bar dot
    if self._center_dot and abs(self._torque_filter.x) < 0.5:
      dot_y = self._rect.y + self._rect.height - torque_line_offset - torque_line_height / 2
      rl.draw_circle(int(cx), int(dot_y), (10 // 2 * self._scale),
                     rl.Color(182, 182, 182, int(255 * 0.9 * self._torque_line_alpha_filter.x)))

  def _draw_meter(self, rect: rl.Rectangle, offset: np.ndarray, mid_r: float, height: float,
                  top_angle: float, half_span: float, engaged: bool) -> None:
    alpha = self._torque_line_alpha_filter.x
    if half_span <= 0:
      return
    x = float(self._torque_filter.x)
    level = abs(x)
    side = 1 if x >= 0 else -1
    span_px = np.deg2rad(half_span) * mid_r  # arc length of one half, in px
    g = 1 * self._scale / span_px  # half the gap between zones, as a fraction of the half span
    half = height / 2
    corner = min(7 * self._scale, half)  # radius of the rounded corners at the two outer stops

    def ribbon(f0: float, f1: float, t0: float | None = None, t1: float | None = None) -> np.ndarray:
      """Points for the zone shape between signed fractions f0..f1 of the half span (negative = left),
      drawn only from t0 to t1 within it. Corners are rounded only where the shape meets an outer stop,
      and a part cut short inside a rounded end follows that rounding. Returns [outer..., inner reversed...]
      as draw_polygon expects."""
      f0, f1 = sorted((f0, f1))
      t0, t1 = sorted((f0 if t0 is None else max(f0, t0), f1 if t1 is None else min(f1, t1)))
      if (t1 - t0) * span_px < 1:
        return np.empty((0, 2), dtype=np.float32)
      s = np.linspace(t0, t1, max(8, int((t1 - t0) * span_px / 3)) + 1) * span_px  # arc length from the centre, px
      inset = np.zeros_like(s)
      for stop, rounded in ((f0 * span_px, f0 <= -1), (f1 * span_px, f1 >= 1)):
        if rounded:
          d = np.clip(corner - np.abs(s - stop), 0, corner)  # how far into the corner each sample is
          inset = np.maximum(inset, corner - np.sqrt(corner ** 2 - d ** 2))
      ang = np.deg2rad(top_angle + half_span * s / span_px)
      r_out, r_in = mid_r + half - inset, mid_r - half + inset
      outer = np.c_[np.cos(ang) * r_out, np.sin(ang) * r_out]
      inner = np.c_[np.cos(ang) * r_in, np.sin(ang) * r_in][::-1]
      return (np.vstack((outer, inner)) + offset).astype(np.float32)

    def fill(pts: np.ndarray, color: tuple[int, int, int], a: float):
      if len(pts):
        draw_polygon(rect, pts, color=rl.Color(*color, int(255 * a * alpha)))

    def bounds(lo: float, hi: float):
      # inner edges sit half a gap from each zone boundary; the centre and the outer stops get no gap
      return (lo + g if lo > 0 else 0.0), (hi - g if hi < 1 else 1.0)

    # unlit track: one white piece across the middle, then the yellow and red zones on each side
    for lo, hi, color, track_a in METER_ZONES:
      start, end = bounds(lo, hi)
      if lo == 0:
        fill(ribbon(-end, end), color, track_a)
      else:
        fill(ribbon(-end, -start), color, track_a)
        fill(ribbon(start, end), color, track_a)

    # lit part: each zone's own shape, filled from the centre to the actual value, in that zone's colour
    for lo, hi, color, _ in METER_ZONES:
      start, end = bounds(lo, hi)
      if level <= start:
        break
      zone = (-end, -start) if side < 0 else (start, end)
      cut = (-level, 0.0) if side < 0 else (0.0, level)
      fill(ribbon(*zone, *cut), color if engaged else (255, 255, 255), 0.9 if engaged else 0.35)

    # pinned at 100%: outline the whole bar in red
    at_limit = self._at_limit
    if at_limit and engaged:
      pts = ribbon(-1.0, 1.0)
      loop = [rl.Vector2(float(px), float(py)) for px, py in np.vstack((pts, pts[:1]))]
      rl.draw_spline_linear(loop, len(loop), 2 * self._scale, rl.Color(255, 40, 0, int(255 * alpha)))
