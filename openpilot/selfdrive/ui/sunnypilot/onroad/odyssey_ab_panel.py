"""Odyssey test panel: which test and arm is running, how much good data there is, and three touch zones.
  top strip    switch to the next test (pulse, bump, lane), so one trip can run several without parking
  middle       next arm, or pause / resume the pulse test
  bottom strip (pulse test) whether the wheel torque sensor may stop a shot
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
PANEL_X, PANEL_Y, PANEL_W = 22, 250, 640
PAD, HEADER, ARM, REASON, ROW_NAME, ROW_VALUE = 22, 32, 110, 40, 36, 32
HEAD_H, FOOT_H = 70, 120  # the touch strips: tall enough to hit without looking closely
# one line per arm: the panel has to end above the driver-monitoring icon at the bottom left (about 430 px below its top)
ROWS_Y, ROW_H, COL_STRAIGHT, COL_CURVES = HEAD_H + 226, 44, 190, 420
PULSE_H = HEAD_H + 270 + FOOT_H
ONROAD_TESTS = ("pulse", "bump", "lane")  # the tests the top strip cycles through; the model test needs its models loaded at start


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


def next_test(params):
  """Top strip: the next of ONROAD_TESTS. The pulse test arrives paused; the others arrive in manual mode on their off arm."""
  e = params.get("OdysseyABExperiment") or ""
  test = ONROAD_TESTS[(ONROAD_TESTS.index(e) + 1) % len(ONROAD_TESTS)] if e in ONROAD_TESTS else ONROAD_TESTS[0]
  params.put("OdysseyABArm", EXPERIMENTS[test][0] if EXPERIMENTS[test] else "")
  if test == "pulse":
    params.put_bool("OdysseyPulsePaused", True)
  else:
    params.put_bool("OdysseyABManual", True)
  params.put("OdysseyABExperiment", test)


def toggle_wheel_stop(params):
  """Bottom strip of the pulse test: let the wheel torque sensor stop shots, or ignore it."""
  params.put_bool("OdysseyPulseIgnoreWheel", not params.get_bool("OdysseyPulseIgnoreWheel"))


def reset_totals():
  def confirmed(result):
    if result == DialogResult.CONFIRM:
      ui_state.params.put("OdysseyABTotals", "{}")
      ui_state.params.put_bool("OdysseyABReset", True)
  gui_app.push_widget(ConfirmDialog("Reset straight and curve totals for every experiment?", "Reset", callback=confirmed))


def panel_rect(content: rl.Rectangle, experiment: str) -> rl.Rectangle:
  height = PULSE_H if experiment == "pulse" else ROWS_Y + len(EXPERIMENTS.get(experiment, ())) * ROW_H + 2 * PAD
  return rl.Rectangle(content.x + PANEL_X, content.y + PANEL_Y, PANEL_W, height)


class OdysseyABPanel(Widget):
  def __init__(self):
    super().__init__()
    self.font = gui_app.font(FontWeight.BOLD)
    self.mem = mem_params(ui_state.params)
    self.e, self.manual, self.status, self.totals = "", False, {}, validate_totals("{}")
    self.last_read = 0.
    self.status_seen = -1e9
    self.drawn: list[str] = []
    self.shown: rl.Rectangle | None = None  # where the panel was last drawn; None while hidden
    self.broken = False

  def zone(self, pos) -> str:
    """Which touch strip a point is in: 'test', 'wheel' or 'main'."""
    if self.shown is None:
      return ""
    if pos.y < self.shown.y + HEAD_H:
      return "test"
    return "wheel" if self.e == "pulse" and pos.y >= self.shown.y + self.shown.height - FOOT_H else "main"

  def _handle_mouse_release(self, mouse_pos, /) -> None:
    # totals are reset in settings, parked: no dialogs while driving
    action = {"test": next_test, "wheel": toggle_wheel_stop, "main": next_arm}.get(self.zone(mouse_pos))
    if action is not None:
      action(ui_state.params)

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
      status = json.loads(self.mem.get("OdysseyABStatus") or "{}")
    except (ValueError, TypeError):
      status = {}
    if status != self.status:
      # controls rewrites its status at least once a second. Judge freshness by when a new one arrived on this
      # process's clock: its own stamp may be on another clock (the pulse test stamps with message time).
      self.status, self.status_seen = status, time.monotonic()
    live = self.status.get("totals") if self.status.get("experiment") == self.e else None
    self.totals = validate_totals(json.dumps({self.e: live}) if live else p.get("OdysseyABTotals"))

  def _render(self, rect):
    if self.e not in EXPERIMENTS:
      self.shown = None
      return
    self.shown = rect
    rl.draw_rectangle_rounded(rect, .06, 8, rl.Color(0, 0, 0, 205))
    x, y = rect.x + PAD, rect.y + PAD

    self.drawn = []  # what the panel says this frame, for the closed-loop screen check (tools/odyssey_sim/replay/ui_check.py)

    def text(s, dx, dy, size, color=rl.WHITE):
      self.drawn.append(s)
      rl.draw_text_ex(self.font, s, rl.Vector2(x + dx, y + dy), size, 0, color)

    text(f"{self.e} test   •   tap here to change test", 0, 0, HEADER, rl.LIGHTGRAY)
    rl.draw_rectangle(int(rect.x), int(rect.y + HEAD_H), int(rect.width), 2, rl.GRAY)
    if self.e == "pulse":
      live = self.status.get("experiment") == "pulse" and time.monotonic() - self.status_seen < 2
      status = self.status if live else {}
      state = {"wait": "waiting", "recover": "re-centring", "lead-in": "pulsing", "pulse": "pulsing",
               "tail": "pulsing"}.get(status.get("state"), str(status.get("state", "waiting")))
      headline = "paused" if status.get("paused") else state
      reason = str(status.get("reason", "waiting for controls"))
      text(headline, 0, HEAD_H - 4, 86)
      text("tap to start" if status.get("paused") else reason if reason != headline else "tap to pause", 0, HEAD_H + 96, 36, rl.YELLOW)
      text(f"{status.get('finished', 0)} of {status.get('total', 0)} shots  •  {status.get('band', '')} mph", 0, HEAD_H + 146, 38)
      text(str(status.get("last_result", "")), 0, HEAD_H + 198, 30)
      foot = rect.height - FOOT_H - PAD
      rl.draw_rectangle(int(rect.x), int(rect.y + rect.height - FOOT_H), int(rect.width), 2, rl.GRAY)
      ignored = bool(status.get("ignore_wheel")) if live else ui_state.params.get_bool("OdysseyPulseIgnoreWheel")
      wheel, limit = status.get("wheel", 0), status.get("wheel_limit", 800)
      stopped = status.get("aborts", {}).get("you're steering", 0)
      text(f"wheel sensor {wheel:.0f}, stops a shot at {limit:.0f}" + (f"  (stopped {stopped})" if stopped else ""), 0, foot + 14, 30,
           rl.RED if wheel >= limit else rl.WHITE)
      text("IGNORED  •  tap here to obey it" if ignored else "obeyed  •  tap here to ignore it", 0, foot + 58, 36,
           rl.YELLOW if ignored else rl.LIGHTGRAY)
      return
    text("tap for the next arm" if self.manual else "arms switch by the clock", 0, HEAD_H, 30, rl.LIGHTGRAY)
    live = self.status.get("experiment") == self.e and time.monotonic() - self.status_seen < 2
    good = bool(live and self.status.get("good"))
    text(str(self.status.get("arm", "")) if live else "waiting", 0, HEAD_H + 26, ARM)
    text(str(self.status.get("reason", "")) if live else "waiting for controls", 0, HEAD_H + 142, REASON, rl.GREEN if good else rl.YELLOW)
    totals = self.totals[self.e]
    need = min(totals, key=lambda a: min(totals[a]["straight"] / TARGET_STRAIGHT, 1) + min(totals[a]["curve"] / TARGET_CURVE, 1))
    text("straight", COL_STRAIGHT, ROWS_Y - 34, 26, rl.GRAY)
    text("curves", COL_CURVES, ROWS_Y - 34, 26, rl.GRAY)
    for i, arm in enumerate(EXPERIMENTS[self.e]):
      dy = ROWS_Y + i * ROW_H
      color = rl.WHITE if arm == need else rl.LIGHTGRAY  # the brightest row is the arm that most needs data
      text(arm, 0, dy, ROW_NAME, color)
      for dx, kind, target in ((COL_STRAIGHT, "straight", TARGET_STRAIGHT), (COL_CURVES, "curve", TARGET_CURVE)):
        val = totals[arm][kind]
        text(f"{val:.0f} / {target} s", dx, dy + 4, ROW_VALUE, rl.GREEN if val >= target else color)
