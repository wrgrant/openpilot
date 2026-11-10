#!/usr/bin/env python3
"""Run the fork's real controlsd process over a recorded minute, with the van's car parameters and a chosen set of
params, before installing a controls change. harness.py replays the lateral controller class alone; this runs
everything controlsd does around it (the lane correction, OdysseySteer, the A/B switches).

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_sim/replay/controlsd_check.py <segment rlog.zst> \
      '{"NnlcModelAB": true, "OdysseyLaneCenterAB": true, "SteerLowSpeedAngle": true}'

It replays the segment twice, once with the recorded drive's own settings (NNLC on) and once with the given params
added, and fails if controlsd dies or puts out a non-finite command. Then it prints, by speed band, how the torque
command compares with what the van logged. The replay is open loop: the recorded van does not respond to the new
command, so the comparison shows direction and size, not how the van would drive.
Use OdysseyABExperiment, OdysseyABManual and OdysseyABArm to force one arm, or --all-manual for every safe arm.
"""
import argparse
import json
import sys

import numpy as np

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.selfdrive.test.process_replay.process_replay import get_custom_params_from_lr, get_process_config, replay_process
from openpilot.tools.lib.logreader import LogReader

BASE = {"NeuralNetworkLateralControl": True, "LagdToggle": True}
BANDS = ((0.5, 4.5, "<10 mph"), (4.5, 8.0, "10-18 mph"), (8.0, 11.2, "18-25 mph"), (11.2, 60.0, ">25 mph"))


def van_car_params(rc, pm, msgs, fingerprint):
  """The van's own logged CarParams and CarParamsSP (torque tuning and NNLC), with the NNLC model path made local.
  The harness's default builds them from the fingerprint alone, which gives the Odyssey a PID controller."""
  cp = next(m for m in msgs if m.which() == 'carParams').carParams.as_builder()
  sp = next(m for m in msgs if m.which() == 'carParamsSP').carParamsSP.as_builder()
  sp.neuralNetworkLateralControl.model.path = BASEDIR + sp.neuralNetworkLateralControl.model.path.split('/data/openpilot', 1)[1]
  Params().put("CarParams", cp.to_bytes(), block=True)
  Params().put("CarParamsSP", sp.to_bytes(), block=True)


def replay(lr: list, params: dict) -> dict[str, np.ndarray]:
  cfg = get_process_config("controlsd")
  cfg.init_callback = van_car_params
  cfg.pubs = list(cfg.pubs) + ["lateralDelay", "selfdriveStateSP", "radarState"]
  custom = get_custom_params_from_lr(lr)
  custom.update(params)
  rows, state = [], "torqueState"  # carControl frames before the first controlsState are not judged
  for m in replay_process(cfg, lr, custom_params=custom, disable_progress=True):
    if m.which() == 'controlsState':
      state = m.controlsState.lateralControlState.which()
    elif m.which() == 'carControl':
      rows.append((m.carControl.actuators.torque, m.carControl.actuators.curvature, m.carControl.latActive, state == 'torqueState'))
  a = np.array(rows, dtype=float)
  return {"torque": a[:, 0], "curv": a[:, 1], "active": a[:, 2] > 0, "torque_controller": a[:, 3] > 0}


def logged(lr: list) -> dict[str, np.ndarray]:
  rows, v = [], 0.0
  for m in lr:
    if m.which() == 'carState':
      v = m.carState.vEgo
    elif m.which() == 'carControl':
      rows.append((m.carControl.actuators.torque, v, m.carControl.latActive))
  a = np.array(rows, dtype=float)
  return {"torque": a[:, 0], "v": a[:, 1], "active": a[:, 2] > 0}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("rlog")
  parser.add_argument("params", nargs="?", default="{}")
  parser.add_argument("--all-manual", action="store_true", help="replay every safe experiment/arm in manual mode")
  args = parser.parse_args()
  lr = list(LogReader(args.rlog))
  params = json.loads(args.params)
  log = logged(lr)
  runs = {"as driven": replay(lr, BASE), "with params": replay(lr, {**BASE, **params})}
  if args.all_manual:
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import EXPERIMENTS
    for experiment, arms in EXPERIMENTS.items():
      for arm in arms:
        runs[f"{experiment}/{arm}"] = replay(lr, {**BASE, **params, "OdysseyABExperiment": experiment,
                                                "OdysseyABManual": True, "OdysseyABArm": arm,
                                                "SteerLowSpeedAngle": True})
  n = min(len(log["torque"]), *(len(r["torque"]) for r in runs.values()))
  failed = False
  for name, r in runs.items():
    finite = bool(np.isfinite(r["torque"]).all() and np.isfinite(r["curv"]).all())
    whole = len(r["torque"]) >= 0.98 * len(log["torque"])
    torque_ctl = bool(r["torque_controller"][r["active"]].all()) if r["active"].any() else False
    failed |= not (finite and whole and torque_ctl)
    print(f"{name}: {len(r['torque'])} frames of {len(log['torque'])} logged, steering active {r['active'].mean():.0%} "
          + f"(logged {log['active'].mean():.0%}), finite {finite}, torque controller {torque_ctl}")
  active = log["active"][:n] & runs["as driven"]["active"][:n] & runs["with params"]["active"][:n]
  print(f"\n{'':>10} {'frames':>6}  logged |torque|, at 100%   " + "   ".join(f"{name}: |torque|, at 100%, r vs logged, same sign" for name in runs))
  for lo, hi, band in BANDS:
    m = active & (log["v"][:n] >= lo) & (log["v"][:n] < hi)
    if m.sum() < 50:
      continue
    lt = log["torque"][:n][m]
    row = f"{band:>10} {m.sum():>6}  {np.median(np.abs(lt)):>15.2f} {np.mean(np.abs(lt) >= 0.99):>8.0%}   "
    for r in runs.values():
      t = r["torque"][:n][m]
      both = (np.abs(lt) > 0.05) & (np.abs(t) > 0.05)
      same = np.mean(np.sign(t[both]) == np.sign(lt[both])) if both.any() else np.nan
      row += f"{np.median(np.abs(t)):>20.2f} {np.mean(np.abs(t) >= 0.99):>8.0%} {np.corrcoef(t, lt)[0, 1]:>+12.2f} {same:>10.0%}   "
    print(row)
  if failed:
    sys.exit("FAILED: controlsd stopped early, left the torque controller, or put out a non-finite command")


if __name__ == '__main__':
  main()
