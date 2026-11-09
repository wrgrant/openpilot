"""Motor onset, manual EPS assist, VSA coherence and wheel/accel mapping."""

import json
from pathlib import Path
import numpy as np

S = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')


def main():
  out = {}
  xs = []
  ys = []
  for p in sorted((S / 'plant_data').glob('*.npz')):
    d = dict(np.load(p))
    ok = (d['v'] > 10) & (d['pressed'] == 0) & (d['active'] > 0) & (abs(d['drv']) < 500)
    motor = []
    for lo, hi in [(0, 0.01), (0.01, 0.02), (0.02, 0.03), (0.03, 0.04), (0.04, 0.06), (0.06, 0.1), (0.1, 0.2), (0.2, 0.4), (0.4, 0.8)]:
      k = ok & (abs(d['cmd']) >= lo) & (abs(d['cmd']) < hi)
      motor.append([lo, hi, int(k.sum()), float(np.median(d['motor'][k])) if k.any() else None, float(np.mean(d['motor'][k] > 3)) if k.any() else None])
    angle_a = -np.radians(d['fine'] - d['angle_offset']) * d['v'] ** 2 / (17.8 * 3)
    k = ok & (abs(angle_a) > 0.3) & (abs(d['act']) > 0.2) & (angle_a * d['act'] > 0)
    ratio = angle_a[k] / d['act'][k]
    k2 = (ratio > 0.4) & (ratio < 3)
    xs.extend((d['v'][k][k2] ** 2)[::10])
    ys.extend((ratio[k2] - 1)[::10])
    idx = np.where(ok)[0][::10]
    vsa_corr = float(np.corrcoef(d['vsa'][idx], d['act'][idx])[0, 1]) if len(idx) > 3 else None
    manual = []
    for lo, hi in [(0, 200), (200, 400), (400, 500), (500, 600), (600, 800), (800, 1200), (1200, 2000), (2000, 4000)]:
      k = (d['active'] == 0) & (d['v'] > 5) & (abs(d['drv']) >= lo) & (abs(d['drv']) < hi)
      manual.append([lo, hi, int(k.sum()), float(np.median(d['motor'][k])) if k.any() else None])
    out[p.stem] = {
      'motor_bins': motor,
      'manual_driver_bins': manual,
      'vsa_act_correlation': vsa_corr,
      'rows': len(d['t']),
      'logged_minutes': len(d['t']) / 6000,
      'live_roll_nonzero_fraction': float(np.mean(d['roll'] != 0)),
      'probe_rows': int(np.sum(abs(d['probe']) > 0)),
    }
  x = np.array(xs)
  y = np.array(ys)
  c = float(np.dot(x, y) / np.dot(x, x))
  out['understeer_coefficient'] = c
  (S / 'plant_diagnostics.json').write_text(json.dumps(out, indent=2) + '\n')
  p = json.loads((S / 'plant_model.json').read_text())
  p['understeer_coefficient'] = c
  p['public_model']['source_segments_sampled'] = 1499
  p.pop('fit_raw', None)
  p['parameter_evidence'] = {
    'deadband': 'motor onset approximately 2-4 percent; fixed .025 in dynamics fit; baseline motor telemetry around 2 CAN units',
    'motor_tau_s': 'assumed 0.03; magnitude-only telemetry and mixed driver assistance limit identification',
    'vehicle_tau_s': 'nominal .01; separate physical vehicle lag not identifiable from angle-derived acceleration',
    'roll_gain': 'William liveParameters roll is all zero; 9.81 prior, not identified',
    'friction_accel': 'effective hysteresis threshold; not a measured Coulomb torque',
    'gain': 'conditional output-error fit; closed-loop data may bias causal interpretation',
  }
  (S / 'plant_model.json').write_text(json.dumps(p, indent=2) + '\n')
  print('understeer', c)
  print(json.dumps({k: v['vsa_act_correlation'] for k, v in out.items() if isinstance(v, dict)}))


if __name__ == '__main__':
  main()
