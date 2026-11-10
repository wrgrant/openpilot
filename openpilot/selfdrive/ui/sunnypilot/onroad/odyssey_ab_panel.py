"""Odyssey A/B panel: which arm is running, how much good data each arm has, and a tap to switch arms.
It shows the arm controls reports back, never the arm the tap asked for, so the label is always what is steering."""
import json
import time

import raylib_nb as rl
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import EXPERIMENTS, TARGET_STRAIGHT, TARGET_CURVE, mem_params, validate_totals
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog

# Sized to read at a glance from the driver's seat on the 2160x1080 screen: the arm is the largest thing after the speed
PANEL_X, PANEL_Y, PANEL_W = 22, 290, 640
PAD, HEADER, ARM, REASON, ROW_NAME, ROW_VALUE = 22, 36, 120, 42, 38, 30
ROWS_Y, ROW_H, BAR_W = 250, 98, 280


def next_arm(params):
  e = params.get("OdysseyABExperiment") or ""
  if e == "pulse":
    params.put_bool("OdysseyPulsePaused", not params.get_bool("OdysseyPulsePaused"))
    return
  if e not in EXPERIMENTS or not params.get_bool("OdysseyABManual"):
    return
  arms = EXPERIMENTS[e]
  arm = params.get("OdysseyABArm")
  params.put("OdysseyABArm", arms[(arms.index(arm) + 1) % len(arms)] if arm in arms else arms[0])


def reset_totals():
  def confirmed(result):
    if result == DialogResult.CONFIRM:
      ui_state.params.put("OdysseyABTotals", "{}")
      ui_state.params.put_bool("OdysseyABReset", True)
  gui_app.push_widget(ConfirmDialog("Reset straight and curve totals for every experiment?", "Reset", callback=confirmed))


def panel_rect(content: rl.Rectangle, experiment: str) -> rl.Rectangle:
  rows = 2 if experiment == "pulse" else len(EXPERIMENTS.get(experiment, ()))
  return rl.Rectangle(content.x + PANEL_X, content.y + PANEL_Y, PANEL_W, ROWS_Y + rows * ROW_H + 2 * PAD)


class OdysseyABPanel(Widget):
  def __init__(self):
    super().__init__()
    self.font = gui_app.font(FontWeight.BOLD)
    self.mem = mem_params(ui_state.params)
    self.e, self.manual, self.status, self.totals = "", False, {}, validate_totals("{}")
    self.last_read = 0.
    self.shown: rl.Rectangle | None = None  # where the panel was last drawn; None while hidden
    self.broken = False
    self.set_click_callback(lambda: next_arm(ui_state.params))  # totals are reset in settings, parked: no dialogs while driving

  def contains(self, pos) -> bool:
    """True if a touch at pos lands on the visible panel, so the road view leaves that touch to the panel."""
    return self.shown is not None and rl.check_collision_point_rec(rl.Vector2(pos.x, pos.y), self.shown)

  def render_in(self, content: rl.Rectangle) -> None:
    """Draw inside the HUD's content rect. A fault in this panel hides it; it must never take the screen down."""
    if self.broken:
      return
    try:
      self.render(panel_rect(content, self.e))
    except Exception:
      self.broken, self.shown = True, None
      cloudlog.exception("odyssey_ab panel disabled")

  def _update_state(self):
    if time.monotonic() - self.last_read < .1:
      return
    self.last_read = time.monotonic()
    p = ui_state.params
    self.e = p.get("OdysseyABExperiment") or ""
    self.manual = p.get_bool("OdysseyABManual")
    try:
      self.status = json.loads(self.mem.get("OdysseyABStatus") or "{}")
    except (ValueError, TypeError):
      self.status = {}
    live = self.status.get("totals") if self.status.get("experiment") == self.e else None
    self.totals = validate_totals(json.dumps({self.e: live}) if live else p.get("OdysseyABTotals"))

  def _render(self, rect):
    if self.e not in EXPERIMENTS:
      self.shown = None
      return
    self.shown = rect
    rl.draw_rectangle_rounded(rect, .06, 8, rl.Color(0, 0, 0, 205))
    x, y = rect.x + PAD, rect.y + PAD

    def text(s, dx, dy, size, color=rl.WHITE):
      rl.draw_text_ex(self.font, s, rl.Vector2(x + dx, y + dy), size, 0, color)

    if self.e == "pulse":
      live = self.status.get("experiment") == "pulse" and time.monotonic() - self.status.get("mono_s", 0) < 2
      status = self.status if live else {}
      text("pulse test • tap to pause / resume", 0, 0, HEADER, rl.LIGHTGRAY)
      state = {"wait": "waiting", "recover": "re-centring"}.get(status.get("state"), str(status.get("state", "waiting")))
      headline = "paused" if status.get("paused") else state
      reason = str(status.get("reason", "waiting for controls"))
      text(headline, 0, 50, 86)
      if reason != headline:  # say why only when it adds something
        text(reason, 0, 170, 36, rl.YELLOW)
      text(f"{status.get('finished', 0)} / {status.get('total', 336)} shots • {status.get('band', '')} mph", 0, 250, 38)
      text(str(status.get("last_result", "")), 0, 330, 30)
      return
    text(f"{self.e} test" + ("  •  tap for next" if self.manual else "  •  by the clock"), 0, 0, HEADER, rl.LIGHTGRAY)
    live = self.status.get("experiment") == self.e and time.monotonic() - self.status.get("mono_s", 0) < 2
    good = bool(live and self.status.get("good"))
    text(str(self.status.get("arm", "")) if live else "waiting", 0, 44, ARM)
    text(str(self.status.get("reason", "")) if live else "waiting for controls", 0, 178, REASON, rl.GREEN if good else rl.YELLOW)
    totals = self.totals[self.e]
    need = min(totals, key=lambda a: min(totals[a]["straight"] / TARGET_STRAIGHT, 1) + min(totals[a]["curve"] / TARGET_CURVE, 1))
    for i, arm in enumerate(EXPERIMENTS[self.e]):
      dy = ROWS_Y + i * ROW_H
      text(arm + ("   needs data" if arm == need else ""), 0, dy, ROW_NAME, rl.WHITE if arm == need else rl.LIGHTGRAY)
      for j, (kind, label, target) in enumerate((("straight", "straight", TARGET_STRAIGHT), ("curve", "curves", TARGET_CURVE))):
        dx = j * (BAR_W + 36)
        val = totals[arm][kind]
        text(f"{label} {val:.0f}/{target} s", dx, dy + 42, ROW_VALUE)
        rl.draw_rectangle(int(x + dx), int(y + dy + 78), BAR_W, 9, rl.DARKGRAY)
        rl.draw_rectangle(int(x + dx), int(y + dy + 78), int(BAR_W * min(1, val / target)), 9, rl.GREEN)
