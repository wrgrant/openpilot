"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntEnum

import raylib_nb as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui.elements import (
  SteeringTorquePercentElement, CurveSpeedElement,
  UiElement, RelDistElement, RelSpeedElement, SteeringAngleElement,
  DesiredLateralAccelElement, ActualLateralAccelElement, DesiredSteeringAngleElement,
  AEgoElement, LeadSpeedElement, FrictionCoefficientElement, LatAccelFactorElement,
  SteeringTorqueEpsElement, BearingDegElement, AltitudeElement, DesiredSteeringPIDElement
)
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


RIGHT_COLUMN_WIDTH = 236  # wide enough for a label and its unit side by side


def _digit_advance(font: rl.Font, size: int) -> float:
  return max(measure_text_cached(font, d, size, 0).x for d in "0123456789")


def draw_tabular_right(font: rl.Font, text: str, right_x: float, y: float, size: int, color: rl.Color) -> None:
  """Draw text right-aligned at right_x, giving every digit the same width (tabular figures)."""
  digit_w = _digit_advance(font, size)
  x = right_x
  for ch in reversed(text):
    ch_w = measure_text_cached(font, ch, size, 0).x
    cell_w = digit_w if ch.isdigit() else ch_w
    x -= cell_w
    rl.draw_text_ex(font, ch, rl.Vector2(x + (cell_w - ch_w) / 2, y), size, 0, color)


def draw_unit(font: rl.Font, unit: str, right_x: float, y: float, size: int, color: rl.Color) -> None:
  """Draw a unit right-aligned at right_x; a trailing ^2 becomes a raised small 2."""
  base, sup = (unit[:-2], unit[-1]) if unit.endswith("^2") else (unit, "")
  sup_size = int(size * 0.65)
  sup_w = measure_text_cached(font, sup, sup_size, 0).x if sup else 0
  base_w = measure_text_cached(font, base, size, 0).x
  rl.draw_text_ex(font, base, rl.Vector2(right_x - sup_w - base_w, y), size, 0, color)
  if sup:
    rl.draw_text_ex(font, sup, rl.Vector2(right_x - sup_w, y - size * 0.15), sup_size, 0, color)


def get_bottom_dev_ui_offset():
  if ui_state.developer_ui in (DeveloperUiState.BOTTOM, DeveloperUiState.BOTH):
    return 60
  return 0


class DeveloperUiState(IntEnum):
  OFF = 0
  BOTTOM = 1
  RIGHT = 2
  BOTH = 3


class DeveloperUiRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self.dev_ui_mode = DeveloperUiState.OFF

    self.rel_dist_elem = RelDistElement()
    self.rel_speed_elem = RelSpeedElement()
    self.steering_angle_elem = SteeringAngleElement()
    self.desired_lat_accel_elem = DesiredLateralAccelElement()
    self.actual_lat_accel_elem = ActualLateralAccelElement()
    self.desired_steer_elem = DesiredSteeringAngleElement()
    self.desired_pid_steer_elem = DesiredSteeringPIDElement()
    self.a_ego_elem = AEgoElement()
    self.lead_speed_elem = LeadSpeedElement()
    self.friction_elem = FrictionCoefficientElement()
    self.lat_accel_factor_elem = LatAccelFactorElement()
    self.steering_torque_elem = SteeringTorqueEpsElement()
    self.bearing_elem = BearingDegElement()
    self.altitude_elem = AltitudeElement()
    self.steer_torque_pct_elem = SteeringTorquePercentElement()
    self.curve_speed_elem = CurveSpeedElement()

  def _update_state(self) -> None:
    self.dev_ui_mode = ui_state.developer_ui

  def _render(self, rect: rl.Rectangle) -> None:
    if self.dev_ui_mode == DeveloperUiState.OFF:
      return

    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      return

    if self.dev_ui_mode == DeveloperUiState.BOTTOM:
      self._draw_bottom_dev_ui(rect)
    elif self.dev_ui_mode == DeveloperUiState.RIGHT:
      self._draw_right_dev_ui(rect)
    elif self.dev_ui_mode == DeveloperUiState.BOTH:
      self._draw_right_dev_ui(rect)
      self._draw_bottom_dev_ui(rect)

  def _draw_right_dev_ui(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    controls_state = sm['controlsState']

    UI_BORDER_SIZE = 20
    container_width = RIGHT_COLUMN_WIDTH
    x = int(rect.x + rect.width - container_width - UI_BORDER_SIZE * 2)
    y = int(rect.y + UI_BORDER_SIZE * 1.5)

    # sunnypilot Odyssey: lead distance/speed are on the chevron, so this column shows steering torque and
    # the curve speed ahead in their place
    elements = [
      self.curve_speed_elem.update(sm, ui_state.is_metric),
      self.steer_torque_pct_elem.update(sm, ui_state.is_metric),
      self.steering_angle_elem.update(sm, ui_state.is_metric),
    ]
    if controls_state.lateralControlState.which() == 'torqueState':
      elements.append(self.desired_lat_accel_elem.update(sm, ui_state.is_metric))
    elif controls_state.lateralControlState.which() == 'angleState':
      elements.append(self.desired_steer_elem.update(sm, ui_state.is_metric))
    elif controls_state.lateralControlState.which() == 'pidState':
      elements.append(self.desired_pid_steer_elem.update(sm, ui_state.is_metric))

    elements.append(self.actual_lat_accel_elem.update(sm, ui_state.is_metric))

    current_y = y
    for element in elements:
      current_y += self._draw_right_dev_ui_element(x, current_y, element)

  def _draw_right_dev_ui_element(self, x: int, y: int, element: UiElement) -> int:
    # sunnypilot Odyssey: label and unit read upright on one line, and the value is right-aligned with
    # fixed-width digits, so the decimal point and last digit stay put as numbers change
    y += 230
    container_width = RIGHT_COLUMN_WIDTH
    label_size = 26
    value_size = 60
    label_color = rl.Color(255, 255, 255, 230)
    unit_color = rl.Color(255, 255, 255, 150)

    rl.draw_text_ex(self._font_bold, element.label, rl.Vector2(x, y), label_size, 0, label_color)
    if element.unit:
      draw_unit(self._font_semi_bold, element.unit, x + container_width, y, label_size, unit_color)

    draw_tabular_right(self._font_bold, element.value, x + container_width, y + 34, value_size, element.color)
    return 130

  def _draw_bottom_dev_ui(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    bar_height = 61
    y = int(rect.y + rect.height - bar_height)

    rl.draw_rectangle(int(rect.x), y, int(rect.width), bar_height,
                      rl.Color(0, 0, 0, 100))

    elements = [
      self.a_ego_elem.update(sm, ui_state.is_metric),
      self.lead_speed_elem.update(sm, ui_state.is_metric),
    ]

    # Add torque-specific elements if using torque control
    if sm['controlsState'].lateralControlState.which() == 'torqueState':
      override_active = ui_state.enforce_torque_control and ui_state.custom_torque_params and ui_state.torque_override_enabled
      if sm.valid['lateralTorqueParameters'] or override_active:
        elements.extend([
          self.friction_elem.update(sm, ui_state.is_metric),
          self.lat_accel_factor_elem.update(sm, ui_state.is_metric),
        ])
    else:
      # Non-torque: show steering torque and GPS data
      elements.append(self.steering_torque_elem.update(sm, ui_state.is_metric))

      if sm.valid['gpsLocationExternal'] or sm.valid['gpsLocation']:
        elements.append(self.bearing_elem.update(sm, ui_state.is_metric))

    # Add altitude if GPS available
    if sm.valid['gpsLocationExternal'] or sm.valid['gpsLocation']:
      elements.append(self.altitude_elem.update(sm, ui_state.is_metric))

    if not elements:
      return

    font_size = 38
    element_widths = []
    for element in elements:
      element.measure(self._font_bold, font_size)
      element_widths.append(element.total_width)

    total_element_width = sum(element_widths)
    num_gaps = len(elements) + 1
    available_width = rect.width
    gap_width = (available_width - total_element_width) / num_gaps

    center_y = y + bar_height // 2
    current_x = rect.x + gap_width

    for i, element in enumerate(elements):
      element_center_x = int(current_x + element_widths[i] / 2)
      self._draw_bottom_dev_ui_element(element_center_x, center_y, element)
      current_x += element_widths[i] + gap_width

  def _draw_bottom_dev_ui_element(self, center_x: int, y: int, element: UiElement) -> None:
    font_size = 38
    start_x = center_x - element.total_width / 2

    rl.draw_text_ex(self._font_bold, element.label_text, rl.Vector2(start_x, y - font_size // 2), font_size, 0, rl.WHITE)
    rl.draw_text_ex(self._font_bold, element.val_text, rl.Vector2(start_x + element.label_width, y - font_size // 2), font_size, 0, element.color)

    if element.unit:
      rl.draw_text_ex(self._font_bold, element.unit_text, rl.Vector2(start_x + element.label_width + element.val_width, y - font_size // 2),
                      font_size, 0, rl.WHITE)
