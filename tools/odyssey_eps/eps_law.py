"""Logs-only EPS law identification. NumPy fits; existing CANParser and LogReader decoding.

Run with --extract once, then without arguments. All caches/results/figures are
outside the repository. Magnitude fits do NOT impute measured motor direction.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path

import numpy as np

from tools.odyssey_eps.fit import TORQUE_BP
from tools.odyssey_eps_report import fine_angle, segment_key

OUT = Path.home() / 'comma-logs/odyssey-evidence/eps_law'
COLS = 't u motor bit disabled v rate angle drv active pressed req fresh seg cs_rate'.split()


def extract(route):
  from opendbc.can import CANParser
  from openpilot.tools.lib.logreader import LogReader
  out = OUT / (route.name + '.npz')
  if out.exists():
    return str(out)
  rows = []
  for path in sorted(route.glob('*/rlog.zst'), key=segment_key):
    parser = CANParser('honda_odyssey_exl_2018_generated', [('STEER_MOTOR_TORQUE', 0), ('STEERING_CONTROL', 0), ('STEER_STATUS', 0)], 0)
    u, active, req = np.nan, False, False
    motor_t, cmd_t, control_t = -np.inf, -np.inf, -np.inf
    for m in sorted(LogReader(str(path)), key=lambda m: m.logMonoTime):
      w, t = m.which(), m.logMonoTime/1e9
      if w in ('can', 'sendcan'):
        messages = getattr(m, w)
        parser.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in messages])])
        if any(c.address == 427 and c.src == 0 for c in messages):
          motor_t = t
        if w == 'sendcan' and any(c.address == 228 and c.src == 0 for c in messages):
          vl = parser.vl['STEERING_CONTROL']
          u, req, cmd_t = -vl['STEER_TORQUE']/4096, bool(vl['STEER_TORQUE_REQUEST']), t
      elif w == 'carControl':
        active, control_t = bool(m.carControl.latActive), t
      elif w == 'carState':
        cs, vl = m.carState, parser.vl['STEER_MOTOR_TORQUE']
        fresh = 0 <= t-motor_t < .04 and 0 <= t-cmd_t < .04 and 0 <= t-control_t < .04
        rows.append((t, u, vl['MOTOR_TORQUE'], vl['UNKNOWN_TORQUE_STATE_BIT'], vl['OUTPUT_DISABLED'],
                     cs.vEgo, parser.vl['STEER_STATUS']['STEER_ANGLE_RATE'], cs.steeringAngleDeg, cs.steeringTorque,
                     active, cs.steeringPressed, req, fresh, segment_key(path)[1], cs.steeringRateDeg))
  a = np.asarray(rows)
  d = dict(zip(COLS, a.T, strict=True))
  d['fine_angle'] = fine_angle(d['angle'], d['rate'], np.r_[.01, np.clip(np.diff(d['t']), 0, .03)])
  np.savez_compressed(out, **d)
  print('decoded', route.name, len(a), flush=True)
  return str(out)


def filter_rate(t, rate, tau):
  """Causal exponential filter; reset on timestamp gaps or backwards time."""
  if tau == 0:
    return rate.copy()
  out = np.empty_like(rate)
  for i in range(len(rate)):
    dt = t[i]-t[i-1] if i else 0
    out[i] = rate[i] if not 0 < dt < .05 else out[i-1]+(-np.expm1(-dt/tau))*(rate[i]-out[i-1])
  return out


def basis(u, dead, cap):
  return np.sign(u)*np.minimum(np.maximum(abs(u)-dead, 0), cap)


def predict(u, rate, coef, dead, cap):
  return abs(coef[0]*basis(u, dead, cap)-coef[1]*rate)


def fit_linear(u, rate, y, dead, cap, damping):
  """Alternating signed least squares on unsigned telemetry, multiple starts.

  Positive damping opposes steering in carState angle convention. Unconstrained
  damping is intentional: the sign itself is part of the hypothesis test.
  """
  x = np.column_stack((basis(u, dead, cap), -rate))
  if not damping:
    g = abs(x[:, 0])
    return np.array([g@y/max(g@g, 1e-20), 0.])
  best = None
  xx = x.T@x
  for k in (-3., 0., 3.):
    coef = np.array([100., k])
    for _ in range(20):
      signed = np.where(x@coef >= 0, y, -y)
      new = np.linalg.lstsq(xx, x.T@signed, rcond=None)[0]
      if np.max(abs(new-coef)) < 1e-7:
        break
      coef = new
    err = np.mean((abs(x@coef)-y)**2)
    if best is None or err < best[0]:
      best = err, coef.copy()
  return best[1]


def eligible(d, cut=100):
  return ((d['active'] > 0) & (d['req'] > 0) & (d['fresh'] > 0) & (d['disabled'] == 0)
          & (d['pressed'] == 0) & (abs(d['drv']) < cut) & np.isfinite(d['u']) & (abs(d['u']) <= 1))


def lagged(d, lag):
  idx = np.searchsorted(d['t'], d['t']-lag, side='right')-1
  idx = np.maximum(idx, 0)
  valid = (d['t']-d['t'][idx] < lag+.025) & (d['seg'] == d['seg'][idx])
  return d['u'][idx], valid


def score(y, pred):
  if not len(y):
    return {'n': 0, 'r2': None, 'rmse': None}
  return {'n': len(y), 'r2': float(1-np.mean((y-pred)**2)/max(np.var(y), 1e-20)),
          'rmse': float(np.sqrt(np.mean((y-pred)**2)))}


def search(u, rates, y, damping):
  best = None
  for dead in (0., .01, .02, .028, .04, .06):
    for cap in (.55, .65, .75, .85, 1.):
      for tau, rate in rates.items():
        coef = fit_linear(u, rate, y, dead, cap, damping)
        mse = np.mean((predict(u, rate, coef, dead, cap)-y)**2)
        if best is None or mse < best['mse']:
          best = {'mse': float(mse), 'dead': dead, 'cap': cap, 'tau': tau, 'coef': coef.tolist()}
  return best


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--extract', action='store_true')
  args = p.parse_args()
  OUT.mkdir(parents=True, exist_ok=True)
  if args.extract:
    routes = sorted((Path.home()/'comma-logs').glob('04b44e4c4fe44232_*'))
    with ProcessPoolExecutor(max_workers=4) as pool:
      list(pool.map(extract, routes))
    return
  analyze()
  speed_comparison()
  rate_ablation()


def spline_x(u):
  """Use the existing characterization command knots; no new decoding/map copy."""
  knots = np.r_[0., TORQUE_BP[1:]]
  h = np.array([np.interp(abs(u), knots, np.eye(len(knots))[i]) for i in range(1, len(knots))]).T
  return h*np.sign(u)[:, None]


def spline_fit(u, rate, y, damping):
  x = spline_x(u)
  if damping:
    x = np.column_stack((x, -rate))
  coef = np.linalg.lstsq(x, y*np.sign(u), rcond=None)[0]
  for _ in range(15):
    coef = np.linalg.lstsq(x, np.where(x@coef >= 0, y, -y), rcond=None)[0]
  return coef


def spline_pred(u, rate, coef):
  x = spline_x(u)
  return abs(x@coef[:x.shape[1]] - (coef[-1]*rate if len(coef) > x.shape[1] else 0))


def residual_bins(rate, residual):
  edges = [-1000, -10, -5, -2, -.5, .5, 2, 5, 10, 1000]
  return [{'lo': lo, 'hi': hi, 'n': int(m.sum()), 'mean': float(np.mean(residual[m])) if m.any() else None}
          for lo, hi in zip(edges[:-1], edges[1:], strict=True)
          for m in [(rate >= lo) & (rate < hi)]]


def plot_svg(path, series, title):
  """Standalone residual figure using only standard library and NumPy."""
  parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="500" viewBox="0 0 900 500">',
           '<rect width="900" height="500" fill="white"/>',
           f'<text x="60" y="30" font-size="20">{title}</text>']
  maxy = max(2., max(abs(r['mean']) for rows in series.values() for r in rows if r['mean'] is not None))*1.2
  for y in np.linspace(-maxy, maxy, 5):
    py = 250-180*y/maxy
    parts.append(f'<path d="M60 {py} H840" stroke="#ddd"/><text x="4" y="{py}" font-size="13">{y:.1f}</text>')
  labels = ['&lt;-10', '-10:-5', '-5:-2', '-2:-.5', '-.5:.5', '.5:2', '2:5', '5:10', '&gt;10']
  for i, lab in enumerate(labels):
    parts.append(f'<text x="{70+i*90}" y="455" font-size="12">{lab}</text>')
  for j, (name, rows) in enumerate(series.items()):
    color = ['#ce4a18', '#165baa', '#388438', '#8648a0'][j]
    pts = ' '.join(f'{90+i*90},{250-180*r["mean"]/maxy}' for i, r in enumerate(rows) if r['mean'] is not None)
    parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="3"/>')
    parts.append(f'<text x="{70+j*200}" y="55" fill="{color}" font-size="14">{name}</text>')
  parts.append('<text x="250" y="490">Rate in command direction (deg/s); residual = observed minus predicted (raw)</text></svg>')
  path.write_text('\n'.join(parts))


def analyze():
  data = {}
  taus = (0., .01, .03, .1, .3)
  for path in sorted(OUT.glob('04*.npz')):
    name = path.stem.split('_')[1].split('--')[0]
    d = dict(np.load(path))
    d['rates'] = {tau: filter_rate(d['t'], d['rate'], tau) for tau in taus}
    data[name] = d
  held = ['000001ac', '000001ad', '000001ae']
  train = [k for k in data if k not in held and k != '000001ab']
  validation = ['000001ab']
  def arrays(names, band, lag, stride=1, cut=100):
    collected = []
    for name in names:
      d = data[name]
      u, ok = lagged(d, lag)
      good = eligible(d, cut) & ok & np.isfinite(u) & (d['v']*2.23694 >= band[0]) & (d['v']*2.23694 < band[1])
      ix = np.flatnonzero(good)[::stride]
      collected.append(np.column_stack((u[ix], d['motor'][ix], d['rate'][ix], d['t'][ix],
                                        *[d['rates'][tau][ix] for tau in taus])))
    return np.concatenate(collected)
  results = {'train': train, 'validation': validation, 'held': held, 'bands': {}, 'routes': {}}
  for name, d in data.items():
    on = (d['active'] > 0) & (d['req'] > 0) & (d['fresh'] > 0)
    results['routes'][name] = {'frames': len(d['t']), 'eligible': int(eligible(d).sum()),
      'driver_abs_percentiles_on': np.percentile(abs(d['drv'][on]), [10, 50, 90, 95, 99]).tolist() if on.any() else [],
      'bit_by_command_sign': [float(np.mean(d['bit'][on & (d['motor'] > 5) & (np.sign(d['u']) == sign)]))
                              if np.any(on & (d['motor'] > 5) & (np.sign(d['u']) == sign)) else None for sign in (-1, 1)],
      'rate_zero_fraction': [float(np.mean(d[key][on] == 0)) if on.any() else None for key in ('cs_rate', 'rate')]}
  for band in [(0, 15), (15, 30), (30, 45), (45, 200)]:
    label = f'{band[0]}-{band[1]}'
    fitted = {}
    for family, damping in [('A', False), ('B', True)]:
      candidates = []
      for lag in (0., .02, .04, .06, .1):
        a = arrays(train, band, lag, 20)
        model = search(a[:, 0], {tau: a[:, 4+i] for i, tau in enumerate(taus) if damping or tau == 0}, a[:, 1], damping)
        v = arrays(validation, band, lag, 5)
        rate = v[:, 4+taus.index(model['tau'])]
        loss = np.mean((predict(v[:, 0], rate, model['coef'], model['dead'], model['cap'])-v[:, 1])**2)
        candidates.append((loss, lag, model))
      _, lag, model = min(candidates, key=lambda x: x[0])
      model['lag'] = lag
      a = arrays(train, band, lag, 5)
      r = a[:, 4+taus.index(model['tau'])]
      model['coef'] = fit_linear(a[:, 0], r, a[:, 1], model['dead'], model['cap'], damping).tolist()
      # Bootstrap 10-second time clusters, holding selected grid parameters fixed.
      blocks = np.floor(a[:, 3]/10).astype(np.int64)
      unique = np.unique(blocks)
      rng = np.random.default_rng(37)
      draws = []
      for _ in range(100):
        sampled, counts = np.unique(rng.choice(unique, len(unique)), return_counts=True)
        ix = np.concatenate([np.tile(np.flatnonzero(blocks == b), n) for b, n in zip(sampled, counts, strict=True)])
        draws.append(fit_linear(a[ix, 0], r[ix], a[ix, 1], model['dead'], model['cap'], damping))
      model['ci95_conditional'] = np.percentile(draws, [2.5, 97.5], axis=0).tolist()
      fitted[family] = model
      print(label, family, model, flush=True)
    # Flexible static map tests whether rate merely proxies a curved command law.
    b = fitted['B']
    a = arrays(train, band, b['lag'], 5)
    for family, damping in [('C_static', False), ('C_rate', True)]:
      coef = spline_fit(a[:, 0], a[:, 4+taus.index(b['tau'])], a[:, 1], damping)
      fitted[family] = {'coef': coef.tolist(), 'tau': b['tau'], 'lag': b['lag']}
    rows = {'fits': fitted, 'scores': {}, 'residuals': {}, 'small': {}, 'sensitivity': {}}
    for name in held:
      rows['scores'][name] = {}
      for family, model in fitted.items():
        a = arrays([name], band, model['lag'])
        rate = a[:, 4+taus.index(model['tau'])]
        pred = (spline_pred(a[:, 0], rate, model['coef']) if family.startswith('C') else
                predict(a[:, 0], rate, model['coef'], model['dead'], model['cap']))
        rows['scores'][name][family] = score(a[:, 1], pred)
    series = {}
    for family, model in fitted.items():
      a = arrays(held, band, model['lag'])
      r = a[:, 4+taus.index(model['tau'])]
      pred = (spline_pred(a[:, 0], r, model['coef']) if family.startswith('C') else
              predict(a[:, 0], r, model['coef'], model['dead'], model['cap']))
      error = a[:, 1]-pred
      series[family] = residual_bins(a[:, 2]*np.sign(a[:, 0]), error)
      rows['residuals'][family] = {'bins': series[family], 'sign': {}}
      for sign, mask in [('turn_in', a[:, 0]*a[:, 2] > .1), ('exit', a[:, 0]*a[:, 2] < -.1), ('still', abs(a[:, 2]) < .5)]:
        rows['residuals'][family]['sign'][sign] = {'n': int(mask.sum()), 'bias': float(np.mean(error[mask])) if mask.any() else None,
                                                        'rmse': float(np.sqrt(np.mean(error[mask]**2))) if mask.any() else None}
    a = arrays(held, band, 0)
    for sign, mask in [('turn_in', a[:, 0]*a[:, 2] > .01), ('exit', a[:, 0]*a[:, 2] < -.01), ('still', abs(a[:, 2]) < .5)]:
      mask &= (abs(a[:, 0]) >= .01) & (abs(a[:, 0]) <= .03)
      rows['small'][sign] = {'n': int(mask.sum()), 'median': float(np.median(a[mask, 1])) if mask.any() else None,
                             'p90': float(np.percentile(a[mask, 1], 90)) if mask.any() else None}
    for cut in (50, 200, 300):
      a = arrays(train, band, b['lag'], 10, cut)
      rows['sensitivity'][str(cut)] = fit_linear(a[:, 0], a[:, 4+taus.index(b['tau'])], a[:, 1], b['dead'], b['cap'], True).tolist()
    plot_svg(OUT/f'residual_{label}.svg', series, f'Held-out EPS motor residuals: {label} mph')
    results['bands'][label] = rows
    (OUT/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False))
  (OUT/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False))
  print('saved', OUT/'results.json', flush=True)


def speed_comparison():
  """Direct competing speed fades relative to the fitted 15-30 mph law."""
  results = json.loads((OUT/'results.json').read_text())
  ref = results['bands']['15-30']['fits']['B']
  groups = {'train': [], 'held': []}
  for path in sorted(OUT.glob('04*.npz')):
    name = path.stem.split('_')[1].split('--')[0]
    if name == '000001ab':
      continue
    d = dict(np.load(path))
    good = eligible(d)
    ix = np.flatnonzero(good)[::10] if name in results['train'] else np.flatnonzero(good)
    rate = filter_rate(d['t'], d['rate'], ref['tau'])
    groups['train' if name in results['train'] else 'held'].append(
      np.column_stack((d['u'][ix], d['motor'][ix], rate[ix], d['v'][ix]*2.23694)))
  groups = {k: np.concatenate(v) for k, v in groups.items()}
  comparisons = {}
  for lo, hi in [(0, 5), (5, 10), (10, 15), (15, 30), (30, 45), (45, 200)]:
    arrays = {k: a[(a[:, 3] >= lo) & (a[:, 3] < hi)] for k, a in groups.items()}
    fits = {}
    for family in ('whole', 'command_only', 'rate_only'):
      a = arrays['train']
      g = ref['coef'][0]*basis(a[:, 0], ref['dead'], ref['cap'])
      damp = ref['coef'][1]*a[:, 2]
      def pred(factor, g, damp, family=family):
        if family == 'whole':
          return factor*abs(g-damp)
        return abs(factor*g-damp) if family == 'command_only' else abs(g-factor*damp)
      candidates = [(np.mean((pred(f, g, damp)-a[:, 1])**2), f) for f in np.linspace(0, 1.5, 301)]
      _, factor = min(candidates)
      a = arrays['held']
      fits[family] = {'factor': float(factor), **score(a[:, 1], pred(factor,
        ref['coef'][0]*basis(a[:, 0], ref['dead'], ref['cap']), ref['coef'][1]*a[:, 2]))}
    comparisons[f'{lo}-{hi}'] = fits
  results['speed_comparison'] = comparisons
  (OUT/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False))
  print('speed comparisons', comparisons, flush=True)


def rate_ablation():
  """Report raw versus filtered rate without selecting on held-out drives."""
  results = json.loads((OUT/'results.json').read_text())
  groups = {'train': [], 'held': []}
  for path in sorted(OUT.glob('04*.npz')):
    name = path.stem.split('_')[1].split('--')[0]
    if name not in results['train']+results['held']:
      continue
    d = dict(np.load(path))
    u, valid = lagged(d, 0)
    ix = np.flatnonzero(eligible(d) & valid & np.isfinite(u))
    if name in results['train']:
      ix = ix[::10]
    filtered = filter_rate(d['t'], d['rate'], .1)
    groups['train' if name in results['train'] else 'held'].append(
      np.column_stack((u[ix], d['motor'][ix], d['rate'][ix], filtered[ix], d['v'][ix]*2.23694)))
  groups = {k: np.concatenate(v) for k, v in groups.items()}
  out = {}
  for lo, hi in [(15, 30), (30, 45), (45, 200)]:
    b = results['bands'][f'{lo}-{hi}']['fits']['B']
    a = groups['train']
    a = a[(a[:, 4] >= lo) & (a[:, 4] < hi)]
    h = groups['held']
    h = h[(h[:, 4] >= lo) & (h[:, 4] < hi)]
    out[f'{lo}-{hi}'] = {}
    for i, label in [(2, 'raw'), (3, 'filtered_100ms')]:
      coef = fit_linear(a[:, 0], a[:, i], a[:, 1], b['dead'], b['cap'], True)
      out[f'{lo}-{hi}'][label] = {'coef': coef.tolist(), **score(h[:, 1], predict(h[:, 0], h[:, i], coef, b['dead'], b['cap']))}
  results['rate_ablation'] = out
  (OUT/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False))
  print('rate ablation', out)


if __name__ == '__main__':
  main()
