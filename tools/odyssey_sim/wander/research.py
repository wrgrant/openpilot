"""Reproducible log diagnosis, counterfactual replay, and conservative lane stress test.

Run with repo venv; JSON and NPZ artifacts go to --output (scratch by default).
No deployment or raw log modifications. Sim results are rejection screens only.
"""

import argparse
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.calib.identify import S
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.perception.metric import perceptible_roughness
from tools.odyssey_sim.wander.runtime import LaneCentering, FrictionAssist

ROUTES = ['0000018c', '0000019b', '0000019c', '000001a7', '000001aa', '000001ab']


def band(x, dt=0.1):
  x = np.asarray(x)
  f = np.fft.rfftfreq(len(x), dt)
  z = np.fft.rfft(x - np.mean(x))
  z[(f < 1 / 20) | (f > 1 / 7)] = 0
  return np.fft.irfft(z, n=len(x))


def corr(a, b):
  return float(np.corrcoef(a, b)[0, 1]) if min(np.std(a), np.std(b)) > 1e-8 else None


def windows(d):
  ok = d['ok'] & (d['v'] >= 22) & (d['lp'] > 0.6) & (d['rp'] > 0.6) & (abs(d['probe']) < 0.001)
  ok &= np.isfinite(np.array([d[k] for k in ['offset', 'road', 'fine', 'des', 'body', 'roll']])).all(axis=0)
  ix = np.where(ok)[0]
  for run in np.split(ix, np.where((np.diff(ix) > 1) | (np.diff(d['t'][ix]) > 0.16))[0] + 1):
    size = 600 if len(run) >= 600 else (300 if len(run) >= 300 else 100)
    for start in range(0, len(run) - size + 1, size):
      yield run[start : start + size]


def diagnose():
  rows, traces = [], {}
  for route in ROUTES:
    d = dict(np.load(S / 'calib' / f'{route}_aligned.npz'))
    for j, r in enumerate(windows(d)):
      w = {k: v[r] for k, v in d.items()}
      e, road = -w['offset'], -w['road']
      des, body = w['des'], w['body']
      eb, rb, db, bb = map(band, (e, road, des, body))
      f = np.fft.rfftfreq(len(e), 0.1)
      e2 = np.fft.irfft(-((2 * np.pi * f) ** 2) * np.fft.rfft(eb), n=len(e))
      # Near-vehicle geometry required for closure; legacy polynomial cache may fail it.
      geometric_accel = rb - bb
      fit = np.linalg.lstsq(np.c_[eb, np.gradient(eb, 0.1), np.ones(len(e))], db - rb, rcond=None)[0]
      wheel = -np.radians(w['fine'] - w['angle_offset']) * w['v'] ** 2 / (w['sr'] * 2.9)
      lane = LaneCentering()
      addon = []
      for i in range(len(e)):
        a = lane.update(
          dt=0.1, speed=w['v'][i], yaw=w['yaw'][i], road_curvature=road[i] / w['v'][i] ** 2, camera_offset=e[i], camera_new=True, valid=True, enabled=True
        )
        addon.append(a)
      addon = np.array(addon)
      # Outward means offset magnitude growing. Add-on must point toward the lane,
      # before peak drift, not after it. This checks intention, not vehicle outcome.
      ed = np.gradient(np.convolve(np.pad(e, (5, 5), mode='edge'), np.ones(11) / 11, mode='valid'), 0.1)
      drift = (abs(e) > 0.02) & (abs(e) < 0.08) & (e * ed > 0) & (abs(ed) > 0.005)
      row = {
        'route': route,
        'window': j,
        'start': float(w['t'][0]),
        'seconds': len(r) * 0.1,
        'wander_valid': len(r) >= 400,
        'wander_cm': float(np.std(eb) * 100),
        'offset_mean_cm': float(np.mean(e) * 100),
        'plan_band_rms': float(np.std(db)),
        'body_band_rms': float(np.std(bb)),
        'road_band_rms': float(np.std(rb)),
        'tracking_error_band_rms': float(np.std(db - bb)),
        'body_plan_band_corr': corr(bb, db),
        'body_plan_band_ratio': float(np.std(bb) / max(np.std(db), 1e-9)),
        'closure_corr': corr(e2, geometric_accel),
        'closure_rms_ratio': float(np.std(geometric_accel) / max(np.std(e2), 1e-9)),
        'road_plan_corr': corr(rb, db),
        'fine_body_corr': corr(band(wheel), bb),
        'roll_error_corr': corr(band(w['roll']), db - bb),
        'planner_gains': fit.tolist(),
        'replay_addon_rms': float(np.sqrt(np.mean(addon**2))),
        'early_drift_n': int(drift.sum()),
        'early_correct_direction_pct': float(np.mean(addon[drift] * e[drift] > 0) * 100) if drift.any() else None,
        'early_correction_median': float(np.median(abs(addon[drift]))) if drift.any() else None,
      }
      rows.append(row)
      traces[f'{route}_{j}'] = np.c_[w['t'], e, road, des, body, wheel, addon]
  return rows, traces


def simulate(mode, speed=30.0, scale=1.0, seed=1, duration=120.0, lag=0.0, plant_scale=1.0, planner=(0.17, 0.57)):
  dt = 0.01
  rng = np.random.default_rng(seed)
  p = Plant()
  # Sensitivity to unmodelled friction and delay; neither is fit to candidate scores.
  p.params = json.loads(json.dumps(p.params))
  p.params['steering'][4:7] = [x * plant_scale for x in p.params['steering'][4:7]]
  lane = LaneCentering()
  friction = FrictionAssist(mode=mode if mode in ('inverse', 'pulse', 'dither') else 'inverse', scale=scale)
  e, ed, integral = 0.10, 0.0, 0.0
  camera = e
  records = []
  queue = [0.0] * max(1, round(lag / dt))
  for i in range(round(duration / dt)):
    t = i * dt
    road = 0.12 * np.sin(2 * np.pi * t / 47)
    roll = 0.004 * np.sin(2 * np.pi * t / 70)
    if i % 5 == 0:
      camera = e + rng.normal(0, 0.03)
    desired = road + planner[0] * e + planner[1] * ed
    addon = lane.update(
      dt=dt,
      speed=speed,
      yaw=p.body / speed,
      road_curvature=road / speed**2,
      camera_offset=camera,
      camera_new=i % 5 == 0,
      valid=True,
      enabled=mode in ('lane', 'combined'),
    )
    desired += addon
    measured = round(p.y / 0.05) * 0.05
    error = desired - measured
    integral = float(np.clip(integral + 0.3 * error * dt, -0.35, 0.35))
    u = float(np.clip(desired / 2.194 + error + integral, -1, 1))
    if mode == 'kick':
      u += 0.03 * np.tanh(u / 0.01)
    if mode in ('inverse', 'pulse', 'dither', 'combined'):
      u = friction.update(u, target=desired, actual=p.y, speed=speed, rate=0.0, dt=dt, enabled=True)
    queue.append(u)
    applied = queue.pop(0)
    old = p.y
    p.step(applied, speed, roll)
    ed += (road - p.body) * dt
    e += ed * dt
    # Wheel-angle proxy conversion only for simulator perception screening.
    angle = -np.degrees(p.y * 15 * 2.9 / speed**2)
    rate = -np.degrees((p.y - old) / dt * 15 * 2.9 / speed**2)
    records.append([e, ed, desired, p.body, u, angle, rate])
  a = np.array(records)[2000:]
  rough = perceptible_roughness(dt=dt, angle_deg=a[:, 5], angle_rate_deg_s=a[:, 6], lat_accel=a[:, 3], command=a[:, 4])
  return {
    'mode': mode,
    'speed': speed,
    'scale': scale,
    'lag': lag,
    'friction_scale': plant_scale,
    'planner': list(planner),
    'wander_cm': float(np.std(band(a[:, 0], dt)) * 100),
    'offset_rms_cm': float(np.sqrt(np.mean(a[:, 0] ** 2)) * 100),
    'max_offset_cm': float(np.max(abs(a[:, 0])) * 100),
    'cost': float(np.mean((a[:, 2] - a[:, 3]) ** 2)),
    'roughness': rough['roughness'],
    'hand_torque_available': False,
    'saturation_pct': float(np.mean(abs(a[:, 4]) >= 0.99) * 100),
  }


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--output', type=Path, default=S / 'wander')
  ap.add_argument('--sim-only', action='store_true')
  args = ap.parse_args()
  args.output.mkdir(exist_ok=True)
  if not args.sim_only:
    rows, traces = diagnose()
    (args.output / 'diagnosis.json').write_text(json.dumps(rows, indent=2))
    np.savez_compressed(args.output / 'addon_replay.npz', **traces)
    print('diagnostic windows', len(rows), flush=True)
  scores = []
  for planner in [(0.17, 0.57), (0.34, 0.91)]:
    for speed in [22.0, 30.0, 35.0]:
      for lag, fs in [(0.0, 1.0), (0.3, 1.0), (0.0, 2.0)]:
        for mode in ['stock', 'kick', 'inverse', 'pulse', 'dither', 'lane', 'combined']:
          scores.append(simulate(mode, speed=speed, lag=lag, plant_scale=fs, planner=planner))
    print('sim planner', planner, 'complete', flush=True)
  (args.output / 'simulations.json').write_text(json.dumps(scores, indent=2))


if __name__ == '__main__':
  main()
