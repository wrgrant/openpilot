"""Residual breakdown, timing checks and off-repo plots for the stopped pass 3."""

import json
import numpy as np
from tools.odyssey_sim.shake.identify3 import OUT, PLOTS, NAMES, save, quant, calibrated
from tools.odyssey_sim.shake.timing3 import extract3, band, lag_seconds
from tools.odyssey_sim.shake.identify2 import route, pieces
from tools.odyssey_sim.shake.physics import RATIO, A, B


def summarize(rows):
  if not rows:
    return {'n': 0}
  return {
    'n': len(rows),
    'passed': sum(r['passed'] for r in rows),
    'gain_2_50_97': quant([r['gain'] for r in rows]),
    'phase_2_50_97_deg': quant([r['phase_deg'] for r in rows]),
  }


def main():
  import matplotlib

  matplotlib.use('Agg')
  import matplotlib.pyplot as plt

  fits = json.loads((OUT / 'yaw3.json').read_text())['fits']
  timing = json.loads((OUT / 'timing3.json').read_text())
  result = {'residual': {}, 'steady_understeer': {}, 'additional_timing': {}}
  allrows = [r | {'speed': f['speed']} for f in fits for r in f['test']]
  held = [r for r in allrows if r['held'] and r['covered']]
  result['gate'] = summarize(held)
  result['uncovered'] = sum(r['held'] and not r['covered'] for r in allrows)
  result['residual']['by_route_speed'] = {
    f'{n}/{s}': summarize([r for r in held if r['route'] == n and r['speed'] == s]) for n in NAMES for s in (0, 1) if any(r['route'] == n for r in held)
  }
  result['residual']['by_frequency'] = {str(f): summarize([r for r in held if abs(r['frequency'] - f) < 0.01]) for f in np.arange(0.4, 3.01, 0.2)}
  result['residual']['by_amplitude_deg'] = {
    f'{lo}-{hi}': summarize([r for r in held if lo <= r['angle_rms_deg'] < hi]) for lo, hi in ((0, 0.15), (0.15, 0.3), (0.3, 0.6), (0.6, 100))
  }
  result['residual']['by_speed_mph'] = {
    f'{lo}-{hi}': summarize([r for r in held if lo <= r['v_m_s'] / 0.44704 < hi]) for lo, hi in ((25, 35), (35, 45), (45, 55), (55, 100))
  }
  # Ten-second curves: per-route steering zero and raw gyro DC are nuisance biases.
  # delta = L*r/v + K*r*v + per-route offset; model steady relation is exact.
  curves = [(n, *c) for n, t in timing.items() for c in t['steady_curves']]
  for label, selected in [
    ('train', [c for c in curves if c[0] not in ('0000019c', '000001a5', '000001ad', '000001ae')]),
    ('check', [c for c in curves if c[0] in ('0000019c', '000001a5', '000001ad', '000001ae')]),
  ]:
    names = sorted({c[0] for c in selected})
    x = np.array([[c[1] * c[3], *[int(c[0] == n) for n in names]] for c in selected])
    y = np.array([c[2] / RATIO - (A + B) * c[3] / c[1] for c in selected])
    best = np.linalg.lstsq(x, y, rcond=None)[0]
    rng = np.random.default_rng(8)
    bs = []
    for _ in range(500):
      ids = rng.integers(len(y), size=len(y))
      bs.append(np.linalg.lstsq(x[ids], y[ids], rcond=None)[0][0])
    result['steady_understeer'][label] = {
      'curves': len(y),
      'K_s2_m': float(best[0]),
      'curve_bootstrap95': np.percentile(bs, [2.5, 97.5]).tolist(),
      'rank': int(np.linalg.matrix_rank(x)),
      'columns': x.shape[1],
      'route_offsets_rad': dict(zip(names, best[1:].tolist(), strict=True)),
      'residual_wheel_deg_rms': float(np.sqrt(np.mean((x @ best - y) ** 2)) * RATIO * 180 / np.pi),
    }
  fig, ax = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
  for s in (0, 1):
    rr = [r for r in held if r['speed'] == s]
    for n in sorted({r['route'] for r in rr}):
      rs = [r for r in rr if r['route'] == n]
      ax[0, s].scatter([r['frequency'] for r in rs], [r['gain'] for r in rs], s=10, alpha=0.45, label=n[-3:])
      ax[1, s].scatter([r['frequency'] for r in rs], [r['phase_deg'] for r in rs], s=10, alpha=0.45)
    ax[0, s].axhspan(0.85, 1.15, color='green', alpha=0.15)
    ax[1, s].axhspan(-15, 15, color='green', alpha=0.15)
    ax[0, s].set(title=['25–45 mph', 'Above 45 mph'][s], ylabel='Predicted / measured yaw gain', ylim=(0, 3))
    ax[1, s].set(ylabel='Predicted − measured phase (degrees)', xlabel='Oscillations, times per second', ylim=(-100, 100))
    ax[0, s].legend(title='Check route')
  PLOTS.mkdir(exist_ok=True, parents=True)
  fig.savefig(PLOTS / 'yaw_residual3.png', dpi=160)
  # Sensor publication interval and CAN sampling interval are distinct from ECU/filter delay.
  for name in NAMES:
    n = extract3(name)
    d = route(name)
    res = {}
    for key in ('angle', 'rate', 'motor', 'command'):
      dt = np.diff(n[key][:, 0])
      dt = dt[(dt > 0) & (dt < 0.1)]
      res[key + '_interval_s'] = quant(dt)
    cl, al, gaps = [], [], []
    for w in pieces(d, manual=name in ('000001a4', '000001a5')):
      ts = w['t']
      native_angle = np.interp(ts, n['angle'][:, 0], n['angle'][:, 2])
      l, c = lag_seconds(band(native_angle)[100:-100], band(w['angle'])[100:-100], limit=0.1)
      if c > 0.7:
        al.append(l)
      a = n['angle']
      aa = a[(a[:, 0] >= ts[0] - 0.02) & (a[:, 0] <= ts[-1] + 0.02)]
      if len(aa) > 1:
        gaps.append(float(np.max(np.diff(aa[:, 0]))))
      if 'torque' in d:
        u = -np.interp(ts, d['t'], d['torque'])
        cmd = np.interp(ts, n['command'][:, 0], n['command'][:, 2]) / 4096
        l, c = lag_seconds(band(u)[100:-100], band(cmd)[100:-100], limit=0.1)
        if c > 0.7:
          cl.append(l)
    res['carState_angle_after_native_s'] = quant(al)
    res['CAN_command_after_actuators_s'] = quant(cl)
    res['native_angle_max_gap_per_window_s'] = quant(gaps)
    result['additional_timing'][name] = res
  # Exact training-shake segment, corrected raw versus pose, for the lead's timing finding.
  n = extract3('000001ac')
  d = route('000001ac')
  g, gv, _ = calibrated(n, 'gyro')
  t = np.arange(420.0, 480.0, 0.01)
  raw = np.interp(t, g[:, 0], gv[:, 2])
  pose = np.interp(t, n['pose'][:, 0], n['pose'][:, 4])
  angle = -np.interp(t, n['angle'][:, 0], n['angle'][:, 2])
  result['segment7_lags'] = {}
  for label, x, y in [('raw_to_pose', raw, pose), ('wheel_to_raw', angle, raw), ('wheel_to_pose', angle, pose)]:
    l, c = lag_seconds(band(x)[100:-100], band(y)[100:-100])
    result['segment7_lags'][label] = {'lag_s': l, 'correlation': c}
  save('summary3.json', result)
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
