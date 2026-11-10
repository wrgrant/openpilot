"""Train only on 1ac and earlier; freeze parameters before opening 1ad/1ae.

All windows are uniform 100 Hz, continuous, one controller/lane/speed arm.
The first five seconds initialize/warm up; no measured state is fed back after t=0.
"""
import json
from pathlib import Path
import numpy as np
from scipy import signal, optimize
from tools.odyssey_sim.shake.extract import SCRATCH, DATA
from tools.odyssey_sim.shake.physics import DEFAULT, simulate

OUT = Path(__file__).parent
SOS = signal.butter(3, [.3, 3.], btype='bandpass', fs=100, output='sos')


def band(x):
  return signal.sosfiltfilt(SOS, x, axis=0)


def windows(name):
  path = SCRATCH/f'{name}.npz'
  d = dict(np.load(path if path.exists() else DATA/f'{name}.npz'))
  t = d['t']
  # Physical speed bands are defined by measured speed, never by model output.
  speed = np.where(d['v'] > 45*.44704, 1, 0)
  blocks = (t//180).astype(int)
  ok = (d['lat'] > .5) & (d['v'] > 25*.44704) & (d['blinker'] == 0) & (t % 180 >= 30)
  edges = np.r_[0, np.flatnonzero((np.diff(t) > .04) | (np.diff(t) <= 0) | (np.diff(d['seg']) != 0) |
                                (np.diff(blocks) != 0) | (np.diff(speed) != 0) | (np.diff(ok.astype(int)) != 0))+1, len(t)]
  for start, end in zip(edges[:-1], edges[1:], strict=True):
    if not ok[start]:
      continue
    for s in np.arange(t[start], t[end-1]-20, 20):
      ts = s+np.arange(2000)*.01
      w = {k: np.interp(ts, t, d[k]) for k in ('torque', 'angle', 'v', 'yaw', 'des_curv', 'pressed', 'drv')}
      w.update(name=name, t=ts, block=int(s//180), model=int(s//180)%3 if name[-2:] in ('ac', 'ad', 'ae') else 0,
               lane=int(s//180)%2 if name[-2:] in ('ac', 'ad', 'ae') else 0, speed=int(speed[start]))
      w['u'] = -w['torque']
      w['truth'] = np.column_stack([-np.radians(w['angle']), w['v']*w['yaw']])
      # Steady-state initial lateral velocity is not observable: explicitly assumed zero.
      w['initial'] = np.array([w['truth'][0, 0], 0., 0., w['yaw'][0], 0., 0., 0., 0.])
      yield w


def fit():
  train = list(windows('000001ac'))
  earlier = [w for name in ('000001a7', '000001aa', '000001ab') for w in windows(name)]
  # Balanced model/speed coverage, deterministic bounded training workload.
  chosen = []
  for arm in range(3):
    for speed in range(2):
      chosen += [w for w in train if w['model'] == arm and w['speed'] == speed][:4]
  chosen += earlier[::max(1, len(earlier)//6)][:6]
  print('train windows', len(train), len(earlier), 'fit', len(chosen), flush=True)
  p = DEFAULT.copy()
  lower = [.005, .02, .01, .002, 30000, 30000, 1500, .005, .05]
  upper = [.3, 5., 1.8, .15, 300000, 300000, 10000, .25, 3.]
  targets = [band(w['truth'])[500:-100:10] for w in chosen]
  scale = np.array([np.radians(.4), .08])

  def residual(logp):
    p[:9] = np.exp(logp)
    result = []
    for w, target in zip(chosen, targets, strict=True):
      y = simulate(w['u'], w['v'], p, w['initial'])
      err = (band(y[:, :2])[500:-100:10]-target)/scale
      result.append(np.nan_to_num(err, nan=1e4, posinf=1e4, neginf=-1e4).clip(-1e4, 1e4).ravel())
    return np.concatenate(result)

  trials = []
  for dahl in (0., .003):
    p[12] = dahl
    result = optimize.least_squares(residual, np.log(DEFAULT[:9]), bounds=(np.log(lower), np.log(upper)),
                                    max_nfev=65, diff_step=.01, loss='soft_l1', verbose=0)
    trials.append({'kind': 'Coulomb' if dahl == 0 else 'Dahl', 'cost': float(np.mean(residual(result.x)**2)),
                       'parameters': p.tolist(), 'nfev': result.nfev, 'optimality': float(result.optimality)})
    print(trials[-1], flush=True)
  winner = min(trials, key=lambda r: r['cost'])
  frozen = {'training': ['1ac', '1a7', '1aa', '1ab'], 'heldout': ['1ad', '1ae'], 'trials': trials,
            'parameters': winner['parameters'], 'training_windows': len(chosen),
            'parameter_bounds': [lower, upper], 'valid_for_controller_design': False}
  (OUT/'fit.json').write_text(json.dumps(frozen, indent=2))
  return frozen


def metrics(w, y):
  actual, predicted = band(w['truth'])[500:-100], band(y[:, :2])[500:-100]
  rows = []
  for j, label in enumerate(('wheel', 'yaw_acceleration')):
    f, pu = signal.welch(w['u'][500:-100], fs=100, nperseg=1000, noverlap=900)
    _, cross_a = signal.csd(w['u'][500:-100], actual[:, j], fs=100, nperseg=1000, noverlap=900)
    _, cross_p = signal.csd(w['u'][500:-100], predicted[:, j], fs=100, nperseg=1000, noverlap=900)
    _, coherence = signal.coherence(w['u'][500:-100], actual[:, j], fs=100, nperseg=1000, noverlap=900)
    bins = []
    for hz in (.5, 1., 1.5, 2.):
      k = np.argmin(abs(f-hz))
      ratio = cross_p[k]/cross_a[k] if abs(cross_a[k]) > 1e-12 else np.nan+0j
      bins.append({'frequency': hz, 'actual_bin': float(f[k]), 'gain_ratio': float(abs(ratio)), 'phase_error_deg': float(np.angle(ratio, deg=True)),
                       'coherence': float(coherence[k]), 'command_psd': float(pu[k])})
    rows.append({'output': label, 'corr': float(np.corrcoef(actual[:, j], predicted[:, j])[0, 1]),
                     'road_rms': float(np.std(actual[:, j])), 'sim_rms': float(np.std(predicted[:, j])), 'bins': bins})
  return rows


def summarize(rows):
  summary = []
  for model in range(3):
    for speed in range(2):
      for output in ('wheel', 'yaw_acceleration'):
        group = [r for r in rows if r['model'] == model and r['speed'] == speed]
        measurements = [next(x for x in r['metrics'] if x['output'] == output) for r in group]
        if not measurements:
          summary.append({'model': model, 'speed': speed, 'output': output, 'windows': 0, 'passed': False})
          continue
        bins = []
        for j in range(4):
          b = [m['bins'][j] for m in measurements]
          bins.append({'frequency': b[0]['frequency'], 'median_gain': float(np.median([x['gain_ratio'] for x in b])),
                           'median_phase': float(np.median([x['phase_error_deg'] for x in b])),
                           'median_coherence': float(np.median([x['coherence'] for x in b]))})
        corr = float(np.median([m['corr'] for m in measurements]))
        passed = corr >= .8 and all(.8 <= b['median_gain'] <= 1.2 and abs(b['median_phase']) <= 20 for b in bins)
        summary.append({'model': model, 'speed': speed, 'output': output, 'windows': len(group), 'median_corr': corr, 'bins': bins, 'passed': passed})
  return summary


def evaluate(frozen):
  p = np.array(frozen['parameters'])
  rows = []
  for name in ('000001ad', '000001ae'):
    for w in windows(name):
      y = simulate(w['u'], w['v'], p, w['initial'])
      rows.append({'route': name, 'start': float(w['t'][0]), 'block': w['block'], 'model': w['model'],
                   'lane': w['lane'], 'speed': w['speed'], 'metrics': metrics(w, y)})
  summary = summarize(rows)
  (OUT/'gate1.json').write_text(json.dumps({'rows': rows, 'summary': summary, 'passed': all(r['passed'] for r in summary)}, indent=2))
  for r in summary:
    print(r, flush=True)
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig, axes = plt.subplots(2, 2, figsize=(11, 7))
  for j, output in enumerate(('wheel', 'yaw_acceleration')):
    for model, label in enumerate(('stock', 'S3', 'S5')):
      group = [r for r in summary if r['model'] == model and r['output'] == output and r['windows']]
      for r in group:
        style = '-' if r['speed'] else '--'
        axes[j, 0].plot([b['frequency'] for b in r['bins']], [b['median_gain'] for b in r['bins']], style,
                        label=f'{label} {">45" if r["speed"] else "25–45"} mph')
        axes[j, 1].plot([b['frequency'] for b in r['bins']], [b['median_phase'] for b in r['bins']], style)
    axes[j, 0].axhspan(.8, 1.2, alpha=.15, color='green')
    axes[j, 1].axhspan(-20, 20, alpha=.15, color='green')
    axes[j, 0].set_ylabel(output+' gain ratio')
    axes[j, 1].set_ylabel('phase error (degrees)')
  axes[0, 0].legend(fontsize=7)
  for ax in axes.flat:
    ax.set_xlabel('oscillations per second')
    ax.grid(alpha=.3)
  fig.tight_layout()
  fig.savefig(OUT/'gate1.png', dpi=150)
  # Training failure event: diagnostic only, never claimed held-out validation.
  w = max((w for w in windows('000001ac') if w['block'] == 2), key=lambda w: np.std(band(w['truth'])[:, 1]))
  y = simulate(w['u'], w['v'], p, w['initial'])
  fig, axes = plt.subplots(2, 1, figsize=(11, 6))
  ts = np.arange(2000)*.01
  axes[0].plot(ts, band(w['truth'])[:, 1]*80, label='road')
  axes[0].plot(ts, band(y[:, :2])[:, 1]*80, label='physical model')
  axes[0].set_ylabel('push on 80 kg (N)')
  axes[0].secondary_yaxis('right', functions=(lambda x: x/80, lambda x: x*80)).set_ylabel('lateral acceleration (m/s²)')
  axes[0].legend()
  axes[1].plot(ts, np.degrees(band(w['truth'])[:, 0]), label='road')
  axes[1].plot(ts, np.degrees(band(y[:, :2])[:, 0]), label='model')
  axes[1].set_ylabel('wheel angle (degrees)')
  axes[1].set_xlabel('time (seconds)')
  fig.tight_layout()
  fig.savefig(OUT/'training_shake.png', dpi=150)


if __name__ == '__main__':
  import sys
  frozen = fit() if '--fit' in sys.argv else json.loads((OUT/'fit.json').read_text())
  evaluate(frozen)
