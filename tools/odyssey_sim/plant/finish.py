"""Record wheel/rate validation, route-wide saturation, and final fitted motor telemetry."""

import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.plant.plant import new_state, step

S = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')


def main():
  p = json.loads((S / 'plant_model.json').read_text())
  m = json.loads((S / 'plant_api_validation.json').read_text())['motor']
  p.update(motor_gain=m['gain'], motor_baseline=m['baseline'], motor_deadband=m['deadband'])
  p['status'] = 'validated offline phenomenological plant; lane-wander realism rejected'
  p['simulator'] = {
    'kp': 0.336,
    'kd': 0.911,
    'controller_laf': 1.47,
    'controller_friction': 0.145,
    'controller_kp': 0.86,
    'controller_ki': 0.15,
    'model_dt': 0.05,
    'lane_noise': 0.0,
  }
  (S / 'plant_model.json').write_text(json.dumps(p, indent=2) + '\n')
  out = {}
  for f in sorted((S / 'plant_data').glob('*.npz')):
    d = dict(np.load(f))
    o = (d['active'] > 0) & (d['pressed'] == 0) & (d['v'] > 5)
    rec = {
      'engaged_alone_seconds': float(o.sum() * 0.01),
      'saturation_cmd_ge_99_pct': float(np.mean(abs(d['cmd'][o]) >= 0.99) * 100) if o.any() else None,
      'logged_controller_saturated_pct': float(np.mean(d['sat'][o]) * 100) if o.any() else None,
      'max_abs_cmd': float(np.max(abs(d['cmd'][o]))) if o.any() else None,
    }
    for tag, mask in [('curves', o & (abs(d['des']) > 0.5)), ('near_straight', o & (abs(d['des']) < 0.15))]:
      rec[tag] = {
        'seconds': int(mask.sum()) * 0.01,
        'rms_actual_plan_error': float(np.sqrt(np.mean((d['act'][mask] - d['des'][mask]) ** 2))) if mask.any() else None,
      }
    # Evaluate shared API, with same eligible windows as fit.py, against fine-angle and 100 ms smoothed EPS rate.
    ix = np.r_[0, np.where(np.diff(np.floor(d['t'] * 10)) > 0)[0] + 1]
    d = {k: v[ix] for k, v in d.items()}
    ok = (d['active'] > 0) & (d['pressed'] == 0) & (abs(d['drv']) < 500) & (d['v'] > 5) & (abs(d['cmd']) < 0.95)
    eligible = ok.copy()
    for j in range(1, 51):
      eligible[j:] &= ok[:-j]
    ae = []
    re = []
    windows = 0
    for i in range(50, len(ok) - 100, 100):
      if not eligible[i : i + 100].all() or np.max(np.diff(d['t'][i : i + 100])) >= 0.16:
        continue
      y = d['act'][i]
      s = new_state(params=p, lat_accel=y, steering_accel=y, motor_cmd=np.sign(d['cmd'][i]) * max(abs(d['cmd'][i]) - 0.025, 0))
      angles = []
      for j in range(i, i + 100):
        s = step(s, float(d['cmd'][j]), float(d['v'][j]), 0.1)
        angles.append(s['angle_deg'])
      target = d['fine'][i : i + 100] - d['angle_offset'][i : i + 100]
      ae.extend(np.asarray(angles) - target)
      # Smooth rate on 0.5 seconds: instantaneous rate is noisy and quantized.
      rt = d['rate'][i : i + 100]
      pr = np.gradient(angles, 0.1)
      re.extend(np.convolve(pr - rt, np.ones(5) / 5, mode='valid'))
      windows += 1
    rec.update(
      wheel_windows=windows,
      wheel_free_10s_rmse_deg=float(np.sqrt(np.mean(np.array(ae) ** 2))) if ae else None,
      rate_500ms_rmse_deg_s=float(np.sqrt(np.mean(np.array(re) ** 2))) if re else None,
    )
    out[f.stem] = rec
  (S / 'plant_route_validation.json').write_text(json.dumps(out, indent=2) + '\n')
  print(json.dumps(out, indent=2))


if __name__ == '__main__':
  main()
