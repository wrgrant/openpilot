"""Odyssey test panel: which test and arm is running, how much good data there is, and touch controls.
  top strip    switch to the next test (pulse, bump, lane), so one trip can run several without parking
  middle       next arm, or fire the next pulse shot (status heading pauses / resumes)
  bottom strip (pulse test) whether the wheel torque sensor may stop a shot
  feel test    status strip pauses / resumes; below it a 2 x 3 grid of 360 x 200 px zones: ratings 0-4 and "felt that"
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

# Left column on the 2160x1080 layout; cruise moves beside it, speed and the road ahead stay visible.
PANEL_X, PANEL_Y, PANEL_W = 22, 10, 720
PAD, HEADER, ARM, REASON, ROW_NAME, ROW_VALUE = 24, 46, 120, 46, 44, 40
HEAD_H, PAUSE_H, FOOT_H = 200, 200, 200
ROWS_Y, ROW_H, COL_STRAIGHT, COL_CURVES = HEAD_H + 340, 70, 230, 460
PULSE_H = 1000  # fire strip gets 400 px; the other three tap targets get 200 px each
NOTE_CHARS, NOTE_COLOR = 34, rl.Color(120, 200, 255, 255)  # a drive companion's note (OdysseyAgentNote): two short lines
ONROAD_TESTS = ("pulse", "bump", "lane", "feel")  # the tests the top strip cycles through; the model test needs its models loaded at start


FEEL_ROW_H = 150  # four rows under the status strip: 0|1, 2|3, 4|felt that, then redo across the full width
FEEL_LABELS = ("didn't notice", "noticed", "mildly annoying", "annoying", "bad")
FEEL_ACK_S = 1.5  # how long a tap's acknowledgement stays on the zone
FEEL_STATE = {"wait": "waiting", "stimulus": "running", "rate": "rate it", "rest": "rated", "done": "plan done"}


def feel_zone(dx, dy, width) -> str:
  """Feel test grid zone at (dx, dy) below the status strip: 'rate0'..'rate4', 'felt' or 'redo'."""
  row = min(3, max(0, int(dy // FEEL_ROW_H)))
  if row == 3:
    return "redo"
  index = row * 2 + (1 if dx >= width / 2 else 0)
  return "felt" if index == 5 else f"rate{index}"


def feel_tap(zone):
  """Send a feel rating or a 'felt that' mark to controls (odyssey_feel.post_input, a file in /dev/shm)."""
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_feel import post_input
  if zone in ("felt", "redo"):
    post_input(zone)
  elif zone.startswith("rate"):
    post_input("rate", int(zone[4:]))


def next_arm(params):
  e = params.get("OdysseyABExperiment") or ""
  if e == "pulse":
    if not params.get_bool("OdysseyPulsePaused"):
      mem_params(params).put("OdysseyPulseFire", str(time.monotonic()))
    return
  if e not in EXPERIMENTS or not params.get_bool("OdysseyABManual"):
    return
  arms = EXPERIMENTS[e]
  arm = params.get("OdysseyABArm")
  params.put("OdysseyABArm", arms[(arms.index(arm) + 1) % len(arms)] if arm in arms else arms[0])


def toggle_pulse(params):
  params.put_bool("OdysseyPulsePaused", not params.get_bool("OdysseyPulsePaused"))


def next_test(params):
  """Top strip: the next of ONROAD_TESTS. The pulse test arrives paused; the others arrive in manual mode on their off arm."""
  e = params.get("OdysseyABExperiment") or ""
  test = ONROAD_TESTS[(ONROAD_TESTS.index(e) + 1) % len(ONROAD_TESTS)] if e in ONROAD_TESTS else ONROAD_TESTS[0]
  params.put("OdysseyABArm", EXPERIMENTS[test][0] if EXPERIMENTS[test] else "")
  if test in ("pulse", "feel"):
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
  height = PULSE_H if experiment in ("pulse", "feel") else ROWS_Y + len(EXPERIMENTS.get(experiment, ())) * ROW_H + 2 * PAD
  return rl.Rectangle(content.x + PANEL_X, content.y + PANEL_Y, PANEL_W, height)


def note_lines(raw) -> list[str]:
  """Up to two lines of NOTE_CHARS from the companion's note; anything longer is cut, never wrapped over the panel."""
  lines = [line.strip()[:NOTE_CHARS] for line in str(raw or "").splitlines() if line.strip()]
  return lines[:2]


class OdysseyABPanel(Widget):
  def __init__(self):
    super().__init__()
    self.font = gui_app.font(FontWeight.BOLD)
    self.mem = mem_params(ui_state.params)
    self.e, self.manual, self.status, self.totals = "", False, {}, validate_totals("{}")
    self.last_read = 0.
    self.status_seen = -1e9
    self.drawn: list[str] = []
    self.note: list[str] = []
    self.shown: rl.Rectangle | None = None  # where the panel was last drawn; None while hidden
    self.broken = False
    self.feel_ack: tuple[str, float] = ("", -1e9)  # the last feel zone tapped, and when, for the acknowledgement

  def zone(self, pos) -> str:
    """Which touch control a point is in: 'test', 'pause', 'wheel', 'main', or in the feel test 'rate0'-'rate4' / 'felt'."""
    if self.shown is None:
      return ""
    if pos.y < self.shown.y + HEAD_H:
      return "test"
    if self.e == "feel":
      dy = pos.y - self.shown.y - HEAD_H - PAUSE_H
      return "pause" if dy < 0 else feel_zone(pos.x - self.shown.x, dy, self.shown.width)
    if self.e == "pulse" and pos.y < self.shown.y + HEAD_H + PAUSE_H:
      return "pause"
    return "wheel" if self.e == "pulse" and pos.y >= self.shown.y + self.shown.height - FOOT_H else "main"

  def _handle_mouse_release(self, mouse_pos, /) -> None:
    # totals are reset in settings, parked: no dialogs while driving
    zone = self.zone(mouse_pos)
    if zone.startswith("rate") or zone in ("felt", "redo"):
      try:  # a feel tap must never take the screen down
        feel_tap(zone)
        self.feel_ack = (zone, time.monotonic())
      except Exception:
        cloudlog.exception("odyssey_feel tap failed")
      return
    action = {"test": next_test, "wheel": toggle_wheel_stop, "main": next_arm, "pause": toggle_pulse}.get(zone)
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
      self._update_state()
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
    self.note = note_lines(self.mem.get("OdysseyAgentNote"))
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
    rl.draw_rectangle_rounded(rect, .06, 8, rl.Color(0, 0, 0, 240))
    x, y = rect.x + PAD, rect.y + PAD

    self.drawn = []  # what the panel says this frame, for the closed-loop screen check (tools/odyssey_sim/replay/ui_check.py)

    def text(s, dx, dy, size, color=rl.WHITE):
      self.drawn.append(s)
      rl.draw_text_ex(self.font, s, rl.Vector2(x + dx, y + dy), size, 0, color)

    text(f"{self.e} test", 0, 20, 64)
    text("tap here to change test", 0, 106, HEADER, rl.LIGHTGRAY)
    rl.draw_rectangle(int(rect.x), int(rect.y + HEAD_H), int(rect.width), 2, rl.GRAY)
    if self.e == "feel":
      self._render_feel(rect, text)
      return
    if self.e == "pulse":
      live = self.status.get("experiment") == "pulse" and time.monotonic() - self.status_seen < 2
      status = self.status if live else {}
      state = {"wait": "waiting", "recover": "re-centring", "lead-in": "pulsing", "pulse": "pulsing",
               "tail": "pulsing"}.get(status.get("state"), str(status.get("state", "waiting")))
      headline = "paused" if status.get("paused") else state
      reason = str(status.get("reason", "waiting for controls"))
      text(headline, 0, HEAD_H + 2, 86)
      text("tap to start" if status.get("paused") else "tap to pause", 0, HEAD_H + 100, 46, rl.LIGHTGRAY)
      hint = "paused" if status.get("paused") else f"Tap to fire: {status.get('next_shot', 'waiting')}"
      if not status.get("paused"):
        if status.get("series_total"):
          hint = f"burst {status['series_index']} of {status['series_total']}"
        elif status.get("mode") == "burst":
          hint = f"Tap for burst of {status.get('burst', 5)}"
        elif status.get("mode") == "auto":
          hint = "auto • tap to fire"
      fire_y = HEAD_H + PAUSE_H
      rl.draw_rectangle(int(rect.x), int(rect.y + fire_y), int(rect.width), 2, rl.GRAY)
      text(hint, 0, fire_y + 14, 52, rl.YELLOW)
      text(reason, 0, fire_y + 84, 40, rl.YELLOW)
      shots = f"{status.get('finished', 0)} of {status.get('total', 0)} shots"
      text(shots + (f"  +  {status.get('queue')} queued" if status.get("queue") else f"  •  {status.get('band', '')} mph"), 0, fire_y + 148, 40)
      if self.note:  # the companion's note takes the place of the last shot's result
        for i, line in enumerate(self.note):
          text(line, 0, fire_y + 216 + 46 * i, 36, NOTE_COLOR)
      else:
        text(str(status.get("last_result", "")), 0, fire_y + 230, 36)
      foot = rect.height - FOOT_H - PAD
      rl.draw_rectangle(int(rect.x), int(rect.y + rect.height - FOOT_H), int(rect.width), 2, rl.GRAY)
      ignored = bool(status.get("ignore_wheel")) if live else ui_state.params.get_bool("OdysseyPulseIgnoreWheel")
      wheel, limit = status.get("wheel", 0), status.get("wheel_limit", 800)
      stopped = status.get("aborts", {}).get("you're steering", 0)
      text(f"wheel {wheel:.0f} / stop {limit:.0f}" + (f"  ({stopped} stops)" if stopped else ""), 0, foot + 30, 42,
           rl.RED if wheel >= limit else rl.WHITE)
      text("IGNORED  •  tap here to obey it" if ignored else "obeyed  •  tap here to ignore it", 0, foot + 106, 40,
           rl.YELLOW if ignored else rl.LIGHTGRAY)
      return
    if self.note:
      text(self.note[0], 0, HEAD_H, 30, NOTE_COLOR)  # one line fits here; the pulse panel shows both
    else:
      text("tap for the next arm" if self.manual else "arms switch by the clock", 0, HEAD_H, 30, rl.LIGHTGRAY)
    live = self.status.get("experiment") == self.e and time.monotonic() - self.status_seen < 2
    good = bool(live and self.status.get("good"))
    text(str(self.status.get("arm", "")) if live else "waiting", 0, HEAD_H + 70, ARM)
    text(str(self.status.get("reason", "")) if live else "waiting for controls", 0, HEAD_H + 220, REASON, rl.GREEN if good else rl.YELLOW)
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

  def _render_feel(self, rect, text):
    """Blind: trial number and state only, never the stimulus. Rating zones light up when a rating is asked for."""
    live = self.status.get("experiment") == "feel" and time.monotonic() - self.status_seen < 2
    status = self.status if live else {}
    paused = bool(status.get("paused")) if live else ui_state.params.get_bool("OdysseyPulsePaused")
    state = str(status.get("state", ""))
    asking = live and state in ("rate", "rest")  # a rating is taken even while paused
    if paused:
      headline = "paused"
    elif not live:
      headline = "waiting for controls"
    elif state in ("stimulus", "rate", "rest"):
      headline = f"trial {status.get('trial', '')}  {FEEL_STATE[state]}"
    else:
      headline = FEEL_STATE.get(state, state)
    text(headline, 0, HEAD_H + 2, 72, rl.YELLOW if state == "rate" and not paused else rl.WHITE)
    detail = "tap to start" if paused else {"stimulus": "tap to pause", "rate": "how did that feel?",
                                            "rest": "tap another to change"}.get(state, f"{status.get('reason', '')}  •  tap to pause")
    text(f"{detail}   ({status.get('rated', 0)} of {status.get('total', 0)} rated)" if live else detail, 0, HEAD_H + 100, 40,
         rl.LIGHTGRAY)
    top = rect.y + HEAD_H + PAUSE_H
    half = rect.width / 2
    ack_zone, ack_t = self.feel_ack
    acked = time.monotonic() - ack_t < FEEL_ACK_S
    chosen = status.get("rating") if asking else None
    for index in range(6):
      zx, zy = rect.x + (index % 2) * half, top + (index // 2) * FEEL_ROW_H
      zone = "felt" if index == 5 else f"rate{index}"
      cell = rl.Rectangle(zx + 6, zy + 6, half - 12, FEEL_ROW_H - 12)
      if zone == "felt":
        fill = rl.Color(40, 120, 160, 255) if acked and ack_zone == "felt" else rl.Color(20, 70, 95, 255)
        number, label, color = "felt", "that  •  any time", NOTE_COLOR
      else:
        fill = rl.Color(30, 120, 60, 255) if chosen == index else rl.Color(60, 60, 60, 255) if asking else rl.Color(25, 25, 25, 255)
        label, color, number = FEEL_LABELS[index], rl.WHITE if asking else rl.GRAY, str(index)
      if acked and ack_zone == zone:
        fill = rl.Color(min(fill.r + 60, 255), min(fill.g + 60, 255), min(fill.b + 60, 255), 255)
      rl.draw_rectangle_rounded(cell, .12, 8, fill)
      dx, dy = zx - rect.x - PAD + 22, zy - rect.y - PAD + 14
      text(number, dx, dy, 80, color)
      text(label, dx, dy + 92, 36, color)
    zy = top + 3 * FEEL_ROW_H
    fill = rl.Color(170, 90, 20, 255) if acked and ack_zone == "redo" else rl.Color(110, 55, 10, 255)
    rl.draw_rectangle_rounded(rl.Rectangle(rect.x + 6, zy + 6, rect.width - 12, FEEL_ROW_H - 12), .12, 8, fill)
    text("redo trial", 22 - PAD + 0, zy - rect.y - PAD + 30, 64, rl.WHITE)
    text("truck / gust spoiled it", 22 - PAD + 330, zy - rect.y - PAD + 50, 36, rl.LIGHTGRAY)
