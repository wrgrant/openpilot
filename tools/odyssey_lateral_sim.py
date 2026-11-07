#!/usr/bin/env python3
"""Closed-loop lane-keeping simulation of the Odyssey on a straight highway, built from its own drive logs.

  model      openpilot's request = road curvature + KP * offset + KD * sideways speed, fitted from the highway drives
             (R^2 0.98), seen through the model's frame-to-frame lane jitter
  controller openpilot's torque controller: feedforward, PID on lateral accel error measured through the 0.1 degree
             angle sensor, friction term, optional SteerDeadbandComp kick
  steering   the EPS motor's deadband (no torque below DEADBAND of full scale), Coulomb friction (the wheel doesn't move
             until the net push exceeds FRICTION), a first-order lag, and a constant one-sided pull (crown or EPS
             return bias)

Prints the same 7-20 s wander numbers as tools/odyssey_wander.py so sim and road compare directly.

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_lateral_sim.py                 # baseline vs kick sweep
  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_lateral_sim.py --friction 0.06 --bias 0.03 --kick 0.04
"""
import argparse
import math
from dataclasses import dataclass, replace

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import deadband_comp

DT = 0.01
MODEL_DT = 0.05
SR, WB = 17.76, 3.0  # steer ratio and wheelbase (learned steerRatio from vehicleParameters)


@dataclass
class Van:
  v: float = 29.0                # m/s (65 mph)
  kp: float = 0.336              # m/s^2 of request per m off centre        (fit, routes 18c + 19c)
  kd: float = 0.911              # m/s^2 of request per m/s drifting         (fit)
  lane_noise: float = 0.03       # m, model's frame-to-frame lane-centre jitter (measured)
  lat_delay: float = 0.3         # s, steer delay openpilot plans with (logged default)
  laf: float = 1.80              # m/s^2 per unit torque (learned latAccelFactor)
  friction_ff: float = 0.118     # openpilot's learned friction coefficient
  deadband: float = 0.037        # EPS motor deadband, fraction of full torque (measured: ~150 of 4096)
  friction: float = 0.04         # m/s^2: net push needed before the wheel moves (to fit)
  lag: float = 0.25              # s, steering response lag
  bias: float = 0.0              # m/s^2: constant pull to one side with no torque (crown / EPS return)
  angle_step: float = 0.1        # deg, angle sensor resolution
  kick: float = 0.0              # SteerDeadbandComp


def angle_to_la(deg: float, v: float) -> float:
  return math.radians(deg) / (SR * WB) * v * v


def la_to_angle(a: float, v: float) -> float:
  return math.degrees(a * SR * WB / (v * v))


def simulate(p: Van, seconds: float = 600.0, seed: int = 0, road: np.ndarray | None = None) -> dict[str, np.ndarray]:
  """road: lateral accel the road's own curves demand (m/s^2) sampled at MODEL_DT, e.g. from load_road(); straight if None."""
  rng = np.random.default_rng(seed)
  n = int(seconds / DT)
  if road is not None:
    n = min(n, len(road) * int(MODEL_DT / DT))
  e, e_dot = 0.05, 0.0     # lane centre relative to the van (m, m/s); + = centre is to the left
  a_car = 0.0              # lateral accel the van is actually doing (m/s^2, + = left)
  integ = 0.0
  req = 0.0
  seen_e = 0.0
  buf = [0.0] * max(1, int(p.lat_delay / DT))
  kp_ctrl, ki_ctrl = 0.86, 0.15  # openpilot torque PID at highway speed
  out = {k: np.zeros(n) for k in ("e", "req", "a_car", "torque", "meas")}
  model_every = int(MODEL_DT / DT)
  for i in range(n):
    a_road = float(road[i // model_every]) if road is not None else 0.0
    if i % model_every == 0:
      # the model sees the lane through its jitter, and plans lat_delay ahead (more delay = more lead on drift)
      seen_e = 0.7 * seen_e + 0.3 * (e + rng.normal(0, p.lane_noise))
      req = a_road + p.kp * (seen_e + (p.lat_delay - 0.3) * e_dot) + p.kd * e_dot
    buf.append(req)
    setpoint = buf.pop(0)
    meas = angle_to_la(round(la_to_angle(a_car, p.v) / p.angle_step) * p.angle_step, p.v)
    err = setpoint - meas
    integ += err * DT
    fric = float(np.interp(err, [-0.2, 0.2], [-p.friction_ff * p.laf, p.friction_ff * p.laf]))
    torque = (req + fric + kp_ctrl * err + ki_ctrl * integ) / p.laf
    torque = float(np.clip(deadband_comp(torque, p.kick), -1, 1))
    # EPS: nothing below the deadband, linear above it
    motor = math.copysign(max(abs(torque) - p.deadband, 0.0), torque) / (1 - p.deadband)
    target = p.laf * motor + p.bias
    push = target - a_car
    if abs(push) > p.friction:  # stick-slip: the wheel only follows once the push beats friction
      a_car += (push - math.copysign(p.friction, push)) * DT / p.lag
    # the van drifts relative to the lane centre: the lane curves by a_road, the van by a_car
    e_dot += (a_road - a_car) * DT
    e += e_dot * DT
    for k, val in (("e", e), ("req", req), ("a_car", a_car), ("torque", torque), ("meas", meas)):
      out[k][i] = val
  return out


def load_road(route_dirs: list[str], min_mph: float = 45) -> np.ndarray:
  """Lateral accel demanded by the road's curves on the engaged highway stretches of recorded drives, at MODEL_DT:
  v^2 times the curvature of the lane centre the model saw. Cached next to each route as road_profile.npy."""
  import glob
  import os
  from openpilot.tools.lib.logreader import LogReader
  parts = []
  for route in route_dirs:
    cache = os.path.join(route, "road_profile.npy")
    if os.path.exists(cache):
      parts.append(np.load(cache))
      continue
    vals, v, active, pressed = [], 0.0, False, False
    for d in sorted(glob.glob(os.path.join(route, "*--*")), key=lambda d: int(d.rsplit("--", 1)[1])):
      for m in sorted(LogReader(os.path.join(d, "rlog.zst")), key=lambda m: m.logMonoTime):
        w = m.which()
        if w == "carState":
          v, pressed = m.carState.vEgo, m.carState.steeringPressed
        elif w == "carControl":
          active = m.carControl.latActive
        elif w == "modelV2" and len(m.modelV2.laneLines) == 4:
          ok = active and not pressed and v > min_mph / 2.237 and min(m.modelV2.laneLineProbs[1], m.modelV2.laneLineProbs[2]) > 0.6
          L, R = m.modelV2.laneLines[1], m.modelV2.laneLines[2]
          x = np.array(L.x)
          k = x < 50
          c2 = np.polyfit(x[k], ((np.array(L.y) + np.array(R.y)) / 2)[k], 2)[0]
          vals.append(v * v * 2 * c2 if ok else np.nan)
    prof = np.array(vals)
    prof = prof[np.isfinite(prof)]  # engaged highway only, joined end to end
    np.save(cache, prof)
    parts.append(prof)
  # sign: lane-line y is to the right, so a road bending left has negative c2
  return -np.concatenate(parts)


def metrics(s: dict[str, np.ndarray], skip_s: float = 60.0) -> dict[str, float]:
  from tools.odyssey_wander import bandpass, peak_period, FS
  step = int(1 / (FS * DT))
  sl = slice(int(skip_s / DT), None, step)
  e, req, a = s["e"][sl], s["req"][sl], s["a_car"][sl]
  des, act = bandpass(req), bandpass(a)
  return {"wander_cm": float(np.std(bandpass(e)) * 100), "period_s": peak_period(e), "desired": float(np.std(des)),
          "follow": float(np.std(des - act) / max(np.std(des), 1e-9)), "rms_cm": float(np.std(e) * 100),
          "small_torque": float(np.mean(np.abs(s["torque"][sl]) < 0.05))}


def fmt(m: dict[str, float]) -> str:
  return (f"wander {m['wander_cm']:5.1f} cm  period {m['period_s']:5.1f} s  desired {m['desired']:.3f}  follow {m['follow']:.2f}"
          + f"  total {m['rms_cm']:5.1f} cm rms  torque<0.05 {m['small_torque']:.0%}")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  for f in ("friction", "bias", "kick", "lag", "deadband", "lat_delay", "v"):
    ap.add_argument(f"--{f.replace('_', '-')}", type=float)
  ap.add_argument("--seconds", type=float, default=1500)
  ap.add_argument("--road", nargs="*", default=[], help="route dirs whose highway curves the van drives (default: straight)")
  args = ap.parse_args()
  base = replace(Van(), **{k: v for k, v in vars(args).items() if k not in ("seconds", "road") and v is not None})
  road = load_road(args.road) if args.road else None
  sim = lambda p: metrics(simulate(p, args.seconds, road=road))  # noqa: E731
  print("road (routes 18c/19c, 65 mph): wander 5-18 cm (typ. ~9), period 15-20 s, follow ~0.25, torque<0.05 ~29%\n")
  print(f"ideal steering (no deadband/friction):  {fmt(sim(replace(base, deadband=0, friction=0, angle_step=1e-6)))}")
  print(f"this van:                               {fmt(sim(base))}")
  if args.kick is None:
    for kick in (0.02, 0.03, 0.037, 0.05):
      print(f"  with SteerDeadbandComp {kick:.3f}:        {fmt(sim(replace(base, kick=kick)))}")


if __name__ == "__main__":
  main()
