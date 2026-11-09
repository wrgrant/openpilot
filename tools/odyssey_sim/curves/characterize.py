"""Reproducible event/release characterization, immutable extraction caches."""

import json
import numpy as np
from tools.odyssey_sim.calib.identify import S, load
from tools.odyssey_sim.calib.evaluate import records
from openpilot.selfdrive.modeld.constants import ModelConstants

OUT = S / 'curves'
KEYS = ['000001a7', '000001aa', '000001ab', '0000018c', '0000019c']


def runs(mask, t, min_s=0.3):
  ix = np.flatnonzero(mask)
  for r in np.split(ix, np.where((np.diff(ix) > 1) | (np.diff(t[ix]) > 0.16))[0] + 1):
    if len(r) > 1 and t[r[-1]] - t[r[0]] >= min_s:
      yield r


def summary(x):
  a = np.asarray(x, dtype=float)
  a = a[np.isfinite(a)]
  return {
    'n': len(a),
    'median': float(np.median(a)) if len(a) else None,
    'p10': float(np.percentile(a, 10)) if len(a) else None,
    'p90': float(np.percentile(a, 90)) if len(a) else None,
  }


def events(d):
  out = []
  # Threshold segments require the same sign and continuous autonomous eligibility.
  for r in runs(d['ok'] & (abs(d['des']) > 0.4), d['t'], 1.0):
    start, end = r[0], r[-1]
    if start < 30 or end + 60 >= len(d['t']):
      continue
    sl = slice(start - 30, end + 61)
    if not d['ok'][sl].all() or np.max(np.diff(d['t'][sl])) > 0.16:
      continue
    sign = np.sign(d['des'][start])
    peak = int(r[np.argmax(sign * d['des'][r])])
    amp = sign * d['des'][peak]
    if amp < 0.5:
      continue
    row = {'start_t': float(d['t'][start]), 'end_t': float(d['t'][end]), 'peak': float(amp), 'speed': float(d['v'][peak])}
    for name in ['des', 'act', 'body', 'geometry_road']:
      y = sign * d[name]
      before = np.arange(start - 30, peak + 1)
      after = np.arange(peak, min(end + 61, len(y)))
      cross = {}
      for pct in [0.2, 0.8]:
        ii = before[y[before] >= pct * amp]
        jj = after[y[after] <= pct * amp]
        cross[str(pct)] = [float(d['t'][ii[0]]) if len(ii) and ii[0] > before[0] else None, float(d['t'][jj[0]]) if len(jj) else None]
      row[name] = cross
    for phase, col in [('entry', 0), ('exit', 1)]:
      for name in ['act', 'body', 'geometry_road']:
        a, b = row[name]['0.2'][col], row['des']['0.2'][col]
        row[f'{phase}_{name}_minus_plan_s'] = None if a is None or b is None else a - b
    a, b = row['des']['0.8'][1], row['des']['0.2'][1]
    row['plan_exit_80_20_s'] = None if a is None or b is None else b - a
    ii = np.arange(peak, end + 61)
    ii = ii[(sign * d['des'][ii] < 0.8 * amp) & (sign * d['des'][ii] > 0.2 * amp)]
    row['exit_holding_command_fraction'] = float(np.mean(sign * d['cmd'][ii] > 0.028)) if len(ii) else None
    out.append(row)
  return out


def main():
  OUT.mkdir(exist_ok=True)
  result = {'events': {}, 'natural_release': {}, 'forecasts': {}}
  for key in KEYS:
    d = load(key)
    geom = np.load(S / 'calib3' / f'{key}.npz')
    ix = np.clip(np.searchsorted(geom['t'], d['t'], side='right') - 1, 0, len(geom['t']) - 1)
    # Cubic 15 m origin curvature; model geometry, not surveyed road truth.
    road = d['v'] ** 2 * geom['geometry'][ix, 3, 2]
    road[(d['t'] - geom['t'][ix] > 0.15) | (geom['probs'][ix, 1] < 0.5) | (geom['probs'][ix, 2] < 0.5)] = np.nan
    d['geometry_road'] = np.convolve(np.pad(road, (2, 2), mode='edge'), np.ones(5) / 5, mode='valid')
    ev = events(d)
    result['events'][key] = {
      'rows': ev,
      'summary': {
        k: summary([r[k] for r in ev])
        for k in [
          'plan_exit_80_20_s',
          'exit_act_minus_plan_s',
          'exit_body_minus_plan_s',
          'entry_act_minus_plan_s',
          'exit_geometry_road_minus_plan_s',
          'exit_holding_command_fraction',
        ]
      },
    }
  for key in KEYS + ['000001a4', '000001a5', '000001a3']:
    d = dict(np.load(S / 'plant_data' / f'{key}.npz'))
    # CAN command is acceleration-positive; raw wheel angle opposite.
    mask = (abs(d['cmd']) < 0.028) & (abs(d['angle']) > 5) & (d['v'] > 5) & (abs(d['drv']) < 100) & (d['pressed'] == 0)
    rows = []
    for r in runs(mask, d['t']):
      inward = -np.sign(d['angle'][r]) * d['rate'][r]
      rows.append(
        {
          't': float(d['t'][r[0]]),
          'seconds': float(d['t'][r[-1]] - d['t'][r[0]]),
          'angle': float(d['angle'][r[0]]),
          'speed': float(np.mean(d['v'][r])),
          'inward_rate': float(np.median(inward)),
          'angle_reduction': float(abs(d['angle'][r[0]]) - abs(d['angle'][r[-1]])),
          'active': float(np.mean(d['active'][r])),
          'probe_max': float(np.max(abs(d['probe'][r]))),
        }
      )
    engaged = (d['active'] > 0) & (d['pressed'] == 0) & (abs(d['drv']) < 100) & (abs(d['angle']) > 5) & (d['v'] > 5) & (-np.sign(d['angle']) * d['rate'] > 0.1)
    result['natural_release'][key] = {
      'rows': rows,
      'event_inward_rate': summary([r['inward_rate'] for r in rows]),
      'engaged_return_rate': summary((-np.sign(d['angle']) * d['rate'])[engaged]),
    }
  probes = {}
  for key in ['000001a3', '000001a7']:
    d = dict(np.load(S / 'plant_data' / f'{key}.npz'))
    rows = []
    for r in runs(abs(d['probe']) > 0.001, d['t'], 0.15):
      end = r[-1]
      ix = np.flatnonzero((d['t'] > d['t'][end]) & (d['t'] < d['t'][end] + 0.5))
      if len(ix) < 10:
        continue
      rows.append(
        {
          't': float(d['t'][r[0]]),
          'peak_probe': float(np.max(abs(d['probe'][r]))),
          'post_command_median': float(np.median(abs(d['cmd'][ix]))),
          'post_driver_median': float(np.median(abs(d['drv'][ix]))),
          'post_angle_median': float(np.median(abs(d['angle'][ix]))),
          'post_inward_rate': float(np.median(-np.sign(d['angle'][ix]) * d['rate'][ix])),
        }
      )
    probes[key] = {
      'rows': rows,
      'post_command': summary([r['post_command_median'] for r in rows]),
      'post_inward_rate': summary([r['post_inward_rate'] for r in rows]),
    }
  (OUT / 'probe_releases.json').write_text(json.dumps(probes, indent=2))
  for key, d in records():
    if key[:8] not in KEYS:
      continue
    rows = []
    a = d['plans'][:, 0]
    ts = ModelConstants.T_IDXS
    for h in [0.7, 1.0, 1.5, 2.0]:
      pred = np.array([np.interp(h, ts, z) - np.interp(0, ts, z) for z in a])
      k = round(h * 10)
      truth = d['des'][k:] - d['des'][:-k]
      ok = d['ok'][:-k] & d['ok'][k:] & (abs(d['t'][k:] - d['t'][:-k] - h) < 0.03)
      for kind, mask in [('all', ok), ('transition', ok & (abs(truth) > 0.1))]:
        p, y = pred[:-k][mask], truth[mask]
        rows.append(
          {
            'horizon': h,
            'kind': kind,
            'n': len(p),
            'forecast_rmse': float(np.sqrt(np.mean((p - y) ** 2))),
            'persistence_rmse': float(np.sqrt(np.mean(y * y))),
            'correlation': float(np.corrcoef(p, y)[0, 1]),
          }
        )
    result['forecasts'][key[:8]] = rows
  (OUT / 'characterization.json').write_text(json.dumps(result, indent=2, allow_nan=False))
  for k, v in result['events'].items():
    print(k, v['summary'])
  for k, v in result['natural_release'].items():
    print(k, 'release', v['event_inward_rate'], 'engaged', v['engaged_return_rate'])


if __name__ == '__main__':
  main()
