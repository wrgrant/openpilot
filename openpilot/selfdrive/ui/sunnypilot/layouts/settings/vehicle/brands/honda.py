"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.odyssey_ab_panel import reset_totals
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import EXPERIMENTS
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, button_item_sp
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from opendbc.car.honda.values import HondaFlags

NIDEC_ONLY = tr_noop("Available on Honda Nidec vehicles only.")
OFFROAD_ONLY = tr_noop("Change this with the vehicle off. It takes effect on the next drive.")

DESCRIPTIONS = {
  'enforce_stock_longitudinal': tr_noop(
    'sunnypilot will not take over control of gas and brakes. Factory Honda Nidec longitudinal control (ACC) will be used, ' +
    'and sunnypilot will only steer.'
  ),
}


class HondaSettings(BrandSettings):
  def __init__(self):
    super().__init__()

    self.enforce_stock_longitudinal = toggle_item_sp(
      lambda: tr("Nidec: Enforce Factory Longitudinal Control"),
      description=lambda: tr(DESCRIPTIONS["enforce_stock_longitudinal"]),
      initial_state=ui_state.params.get_bool("HondaEnforceStockLongitudinal"),
      callback=self._on_enable_enforce_stock_longitudinal,
      enabled=lambda: ui_state.is_offroad(),
    )

    names = ["", *EXPERIMENTS]
    self.ab_experiment = multiple_button_item_sp("Odyssey A/B experiment", "Select while parked; lowspeed also needs SteerLowSpeedAngle, models need NNLC.",
                                               ["None", *(self.LABELS.get(e, e) for e in EXPERIMENTS)], button_width=190,
                                               callback=lambda i: self._select_experiment(names[i]), inline=False)  # seven buttons must fit the page
    self.ab_manual = toggle_item_sp("Driver selects A/B arm", initial_state=ui_state.params.get_bool("OdysseyABManual"),
                                   callback=lambda v: ui_state.params.put_bool("OdysseyABManual", v))
    self.ab_reset = button_item_sp("Odyssey A/B totals", "Reset", callback=reset_totals)
    self.pulse_reset = button_item_sp("Pulse shots", "Reset", callback=self._reset_pulse)
    self.items = [self.enforce_stock_longitudinal, self.ab_experiment, self.ab_manual, self.ab_reset, self.pulse_reset]

  LABELS = {"nnlc_model": "model", "lowspeed": "low speed"}

  @staticmethod
  def _reset_pulse():
    def confirmed(result):
      if result == DialogResult.CONFIRM:
        ui_state.params.put_bool("OdysseyPulseReset", True)
    gui_app.push_widget(ConfirmDialog("Reset completed pulse shots in both speed bands?", "Reset", callback=confirmed))

  @staticmethod
  def _select_experiment(name):
    ui_state.params.put("OdysseyABArm", EXPERIMENTS[name][0] if name and EXPERIMENTS[name] else "")
    ui_state.params.put("OdysseyABExperiment", name)
    if name in ("pulse", "feel"):
      ui_state.params.put_bool("OdysseyPulsePaused", True)  # both start paused; feel shares the pulse test's pause


  def _on_enable_enforce_stock_longitudinal(self, state: bool):
    if state:
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          ui_state.params.put_bool("HondaEnforceStockLongitudinal", True)
        else:
          self.enforce_stock_longitudinal.action_item.set_state(False)

      content = (f"<h1>{self.enforce_stock_longitudinal.title}</h1><br>" +
                 f"<p>{self.enforce_stock_longitudinal.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      ui_state.params.put_bool("HondaEnforceStockLongitudinal", False)

  def update_settings(self):
    for item in (self.ab_experiment, self.ab_manual, self.ab_reset, self.pulse_reset):
      item.action_item.set_enabled(ui_state.is_offroad())
    names = ["", *EXPERIMENTS]
    current = ui_state.params.get("OdysseyABExperiment") or ""
    self.ab_experiment.action_item.set_selected_button(names.index(current) if current in names else 0)
    bosch = ui_state.CP is not None and bool(ui_state.CP.flags & HondaFlags.BOSCH)
    self.enforce_stock_longitudinal.action_item.set_enabled(ui_state.is_offroad() and not bosch)

    note = NIDEC_ONLY if bosch else OFFROAD_ONLY
    new_desc = "<b>" + tr(note) + "</b>\n\n" + tr(DESCRIPTIONS["enforce_stock_longitudinal"])
    if self.enforce_stock_longitudinal.description != new_desc:
      self.enforce_stock_longitudinal.set_description(new_desc)
