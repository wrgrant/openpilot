#!/usr/bin/env python3
"""Run the real screen and the real controls process on a recorded drive, tap the Odyssey panel, and check what happens.

comma's replay tool plays a recorded minute (messages and camera video) as if the van were driving it. The fork's real
controlsd steers on it, the real main screen draws it in the large layout, and this script injects touches into the
screen the way openpilot's own UI test does (selfdrive/ui/tests/diff/replay.py). It then checks, for each touch strip of
the panel, that the setting changed, that controls acted on it, and that the touch did not also open the sidebar.
The whole run is recorded to a video, with a still at every step.

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_sim/replay/ui_check.py ~/comma-logs/<dongle>_<route> --segment 10 --out <dir>

The recorded van does not react to anything controls sends, so this proves the screen, the touches and the software
loop, not the van's response. It needs the Mac's display awake (a window is created, hidden).
"""
import argparse
import json
import os
import pty
import subprocess
import sys
import threading
import time
from pathlib import Path

FPS = 20
TAP_FRAMES = 3


def drain(fd: int) -> None:
  try:
    while os.read(fd, 65536):
      pass
  except OSError:
    pass


def car_params(segment: Path):
  """The van's own CarParams and CarParamsSP from the log, with the NNLC model path made local (as controlsd_check)."""
  from openpilot.common.basedir import BASEDIR
  from openpilot.tools.lib.logreader import LogReader
  cp = sp = None
  for m in LogReader(str(segment / "rlog.zst")):
    if m.which() == "carParams" and cp is None:
      cp = m.carParams.as_builder()
    elif m.which() == "carParamsSP" and sp is None:
      sp = m.carParamsSP.as_builder()
    if cp is not None and sp is not None:
      break
  if cp is None or sp is None:
    sys.exit(f"{segment} has no carParams / carParamsSP: use a segment from a drive on this fork")
  sp.neuralNetworkLateralControl.model.path = BASEDIR + sp.neuralNetworkLateralControl.model.path.split('/data/openpilot', 1)[1]
  return cp.to_bytes(), sp.to_bytes()


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route_dir")
  ap.add_argument("--segment", type=int, default=0)
  ap.add_argument("--out", default="ui_check")
  ap.add_argument("--windowed", action="store_true", help="show the window instead of hiding it")
  ap.add_argument("--soak", type=float, default=0., help="after the taps, leave the pulse test running this many seconds and report "
                  + "shots, stops by reason and the largest wheel-sensor reading (for stretches where the wheel was being held or shaken)")
  args = ap.parse_args()
  route_dir = Path(args.route_dir).expanduser().resolve()
  dongle, route = route_dir.name.split("_", 1)
  out = Path(args.out).expanduser().resolve()
  out.mkdir(parents=True, exist_ok=True)
  # the screen reads these when it is imported
  os.environ.update(BIG="1", RECORD="1", RECORD_OUTPUT=str(out / "screen.mp4"), RECORD_QUALITY="23")

  from openpilot.common.basedir import BASEDIR
  from openpilot.common.prefix import OpenpilotPrefix
  with OpenpilotPrefix() as prefix:
    from openpilot.common.params import Params
    from openpilot.selfdrive.ui.tests.diff.replay import setup_state
    setup_state()
    params = Params()
    cp, sp = car_params(route_dir / f"{route}--{args.segment}")
    params.put("CarParams", cp, block=True)
    params.put("CarParamsSP", sp, block=True)
    for key in ("NeuralNetworkLateralControl", "LagdToggle", "SteerLowSpeedAngle", "TorqueBar", "OdysseyPulsePaused"):
      params.put_bool(key, True, block=True)
    params.put("DevUIInfo", 3, block=True)
    params.put("OdysseyABExperiment", "pulse", block=True)

    # replay shells out to python for its file reader: it has to find this repo's interpreter, not the system one
    env = {**os.environ, "OPENPILOT_PREFIX": prefix.prefix, "PYTHONPATH": BASEDIR, "TERM": "xterm-256color",
           "PATH": os.path.dirname(sys.executable) + os.pathsep + os.environ.get("PATH", "")}
    log = open(out / "controlsd.log", "w")
    # replay draws a console screen and exits without a terminal: give it a pseudo-terminal and throw its drawing away
    master, slave = pty.openpty()
    threading.Thread(target=drain, args=(master,), daemon=True).start()
    procs = [
      # the recorded van's own carControl and controlsState are held back: this run's controlsd publishes them
      subprocess.Popen([os.path.join(BASEDIR, "openpilot/tools/replay/replay"), f"{dongle}/{route}", "--data_dir", str(route_dir),
                        "-s", str(60 * args.segment), "--ecam", "-b", "carControl,controlsState,carControlSP"],
                       env=env, stdin=slave, stdout=slave, stderr=slave, start_new_session=True, cwd=os.path.join(BASEDIR, "openpilot/tools/replay")),
      subprocess.Popen([sys.executable, "-m", "openpilot.selfdrive.controls.controlsd"], env=env, cwd=BASEDIR,
                       stdout=log, stderr=subprocess.STDOUT),
    ]
    try:
      results = drive_screen(params, out, procs, hidden=not args.windowed, soak=args.soak)
    finally:
      for p in procs:
        p.terminate()
      for p in procs:
        try:
          p.wait(timeout=5)
        except subprocess.TimeoutExpired:
          p.kill()
  (out / "results.json").write_text(json.dumps(results, indent=2))
  width = max(len(r["step"]) for r in results)
  for r in results:
    print(f"{'ok  ' if r['ok'] else 'FAIL'} {r['step']:<{width}}  {r['detail']}")
  print(f"video: {out / 'screen.mp4'}   stills: {out}/NN_*.png")
  sys.exit(0 if all(r["ok"] for r in results) else 1)


def drive_screen(params, out: Path, procs, hidden: bool, soak: float = 0.) -> list[dict]:
  import raylib_nb as rl
  from PIL import Image
  if hidden:
    rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  from openpilot.selfdrive.ui.ui_state import ui_state, device
  from openpilot.system.ui.lib.application import MouseEvent, MousePos, gui_app
  from openpilot.selfdrive.ui.layouts.main import MainLayout, MainState
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import mem_params

  gui_app.init_window("Odyssey screen check", fps=FPS)
  layout = MainLayout()
  device.set_override_interactive_timeout(99999)
  road = layout._layouts[MainState.ONROAD]
  panel = road._hud_renderer.odyssey_ab_panel
  mem = mem_params(params)
  results: list[dict] = []
  frame = 0
  pending: list[tuple[int, MouseEvent]] = []

  def status() -> dict:
    try:
      return json.loads(mem.get("OdysseyABStatus") or "{}")
    except (ValueError, TypeError):
      return {}

  def still(name: str) -> None:
    image = rl.load_image_from_texture(gui_app._render_texture.texture)
    size = (image.width, image.height)
    Image.frombytes("RGBA", size, image.to_bytes(size[0] * size[1] * 4)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).convert("RGB").save(
      out / f"{len(results):02d}_{name}.png")
    rl.unload_image(image)

  def record(step: str, ok: bool, detail: str) -> None:
    still(step.replace(" ", "_").replace(":", ""))
    results.append({"step": step, "ok": bool(ok), "detail": detail, "frame": frame, "status": status()})

  def tap(x: float, y: float) -> None:
    pending.append((frame + 1, MouseEvent(MousePos(x, y), 0, True, False, False, time.monotonic())))
    pending.append((frame + 1 + TAP_FRAMES, MouseEvent(MousePos(x, y), 0, False, True, False, time.monotonic())))

  def wait(cond, seconds: float):
    """Run frames until cond() or the time is up; a generator, driven by the render loop."""
    end = time.monotonic() + seconds
    while not cond() and time.monotonic() < end:
      yield
    return bool(cond())

  def strip(which: str) -> tuple[float, float]:
    r = panel.shown
    return r.x + r.width / 2, {"top": r.y + 30, "middle": r.y + r.height / 2 - 20, "bottom": r.y + r.height - 50}[which]

  def script():
    ok = yield from wait(lambda: ui_state.started and panel.shown is not None, 40)
    yield from wait(lambda: False, 2)
    record("the panel shows on the road view", ok, f"started={ui_state.started}, panel at {None if panel.shown is None else (panel.shown.x, panel.shown.y)}")
    if not ok:
      return
    # a touch on the road, away from the panel, opens the sidebar: proves the injected touches are real ones
    tap(layout._content_rect.x + layout._content_rect.width * .7, 400)
    opened = yield from wait(lambda: layout._sidebar.is_visible, 2)
    record("a touch on the road opens the sidebar", opened, f"sidebar visible={layout._sidebar.is_visible}")
    tap(layout._content_rect.x + layout._content_rect.width * .7, 400)
    yield from wait(lambda: not layout._sidebar.is_visible, 2)

    yield from wait(lambda: status().get("experiment") == "pulse", 10)
    s = status()
    yield from wait(lambda: "paused" in panel.drawn, 2)
    record("controls reports the pulse test, paused, and the panel says so",
           s.get("paused") is True and "paused" in panel.drawn and "tap to start" in panel.drawn,
           f"status {s.get('state')}, paused={s.get('paused')}, panel says {panel.drawn[1:3]}")

    tap(*strip("middle"))
    ok = yield from wait(lambda: not params.get_bool("OdysseyPulsePaused") and status().get("paused") is False, 3)
    record("middle strip starts the test", ok and not layout._sidebar.is_visible,
           f"paused param={params.get_bool('OdysseyPulsePaused')}, controls paused={status().get('paused')}, sidebar={layout._sidebar.is_visible}")

    seen = set()

    def shot_done():
      seen.add(status().get("state"))
      return status().get("finished", 0) >= 1
    ok = yield from wait(shot_done, 45)
    s = status()
    count = f"{s.get('finished')} of {s.get('total')} shots"
    yield from wait(lambda: any(count in line for line in panel.drawn), 2)
    # with --soak the road is whatever was asked for and may not allow a shot this soon: then this step only reports
    record("controls runs a shot on the recorded road and the panel counts it", (ok or soak > 0) and any(count in line for line in panel.drawn),
           f"states seen {sorted(x for x in seen if x)}, last: {s.get('last_result')}, panel says {[line for line in panel.drawn if 'shots' in line]}")

    # what a drive companion does from outside: queue an extra shot and leave a note
    note = "3 of 26 good. Wheel moves at 5%.\nNeed: 2 sweeps, ramp left."
    params.put("OdysseyPulseQueue", json.dumps([{"kind": "single", "height": .07, "duration": .1, "direction": 1}]), block=True)
    mem.put("OdysseyAgentNote", note, block=True)
    ok = yield from wait(lambda: str(status().get("last_result", "")).startswith("single 7%"), 45)
    shown = all(line in panel.drawn for line in note.splitlines())
    record("a queued shot runs before the plan and the note shows on the panel", (ok or soak > 0) and shown,
           f"last shot: {status().get('last_result')}, queue left {status().get('queue')}, panel says {[line for line in panel.drawn if line in note]}")

    tap(*strip("bottom"))
    ok = yield from wait(lambda: params.get_bool("OdysseyPulseIgnoreWheel") and status().get("ignore_wheel") is True, 3)
    record("bottom strip makes controls ignore the wheel sensor", ok and not params.get_bool("OdysseyPulsePaused"),
           f"param={params.get_bool('OdysseyPulseIgnoreWheel')}, controls={status().get('ignore_wheel')}, wheel sensor reading {status().get('wheel')}")
    tap(*strip("bottom"))
    ok = yield from wait(lambda: not params.get_bool("OdysseyPulseIgnoreWheel") and status().get("ignore_wheel") is False, 3)
    record("bottom strip again obeys it", ok, f"param={params.get_bool('OdysseyPulseIgnoreWheel')}, controls={status().get('ignore_wheel')}")

    tap(*strip("top"))
    ok = yield from wait(lambda: params.get("OdysseyABExperiment") == "bump" and status().get("experiment") == "bump", 4)
    record("top strip switches to the bump test", ok,
           f"experiment param={params.get('OdysseyABExperiment')}, controls arm={status().get('arm')}, reason={status().get('reason')}")
    tap(*strip("middle"))
    ok = yield from wait(lambda: status().get("arm") == "kick", 3)
    yield from wait(lambda: "kick" in panel.drawn, 2)
    record("middle strip moves to the kick arm, controls confirms it, the panel shows it", ok and "kick" in panel.drawn and not layout._sidebar.is_visible,
           f"arm param={params.get('OdysseyABArm')}, controls arm={status().get('arm')}, panel says {panel.drawn[2:4]}, sidebar={layout._sidebar.is_visible}")
    ok = yield from wait(lambda: status().get("good") is True, 12)
    record("the kick arm counts good seconds", ok, f"reason={status().get('reason')}, totals={status().get('totals', {}).get('kick')}")
    tap(*strip("top"))
    ok = yield from wait(lambda: status().get("experiment") == "lane", 4)
    record("top strip switches to the lane test", ok, f"experiment={status().get('experiment')}, arm={status().get('arm')}")
    tap(*strip("top"))
    ok = yield from wait(lambda: status().get("experiment") == "pulse" and status().get("paused") is True, 4)
    record("top strip returns to the pulse test, paused", ok, f"experiment={status().get('experiment')}, paused={status().get('paused')}")
    if soak > 0:
      tap(*strip("middle"))
      yield from wait(lambda: status().get("paused") is False, 3)
      peak, reasons, red = 0, {}, False
      end = time.monotonic() + soak

      def watching():
        nonlocal peak, red
        st = status()
        peak = max(peak, st.get("wheel", 0))
        reasons[st.get("reason", "")] = reasons.get(st.get("reason", ""), 0) + 1
        red = red or (st.get("wheel", 0) >= st.get("wheel_limit", 1e9) and any("wheel sensor" in line for line in panel.drawn))
        return time.monotonic() >= end
      yield from wait(watching, soak + 1)
      st, frames = status(), max(sum(reasons.values()), 1)
      share = ", ".join(f"{k} {100 * v / frames:.0f}%" for k, v in sorted(reasons.items(), key=lambda kv: -kv[1])[:6])
      record(f"{soak:.0f} s with the test running", not panel.broken,
             f"shots finished {st.get('finished')}, stopped {st.get('aborts')}, largest wheel-sensor reading {peak} (limit {st.get('wheel_limit')}), "
             + f"panel showed the reading over the limit: {red}; time by state: {share}")
    record("both helper processes are still running", all(p.poll() is None for p in procs) and not panel.broken,
           f"exit codes {[p.poll() for p in procs]}, panel fault {panel.broken}")

  runner = script()
  try:
    for _ in gui_app.render():
      while pending and pending[0][0] <= frame:
        with gui_app._mouse._lock:
          gui_app._mouse._events.append(pending.pop(0)[1])
      ui_state.update()
      frame += 1
      try:
        next(runner)
      except StopIteration:
        break
  finally:
    gui_app.close()
  return results


if __name__ == "__main__":
  main()
