"""Build matched evidence tables, preserving failures and visibility limitations."""
import argparse
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.calib.identify import S
from tools.odyssey_sim.vsim.transfer import analyze as transfer_analysis, perceive
from tools.odyssey_wander import bandpass


def direct(base, path):
  indexed = {r['frame']: r for r in path}
  pairs = [(b, indexed[b['frame']]) for b in base if b['frame'] in indexed]
  t = np.array([b['t'] for b, _ in pairs])
  grid = np.arange(t[0], t[-1], .05)
  def band(y):
    return bandpass(np.interp(grid, t, y))
  out = {}
  for index, name in ((0, 'unwarped'), (1, 'smoothed_path')):
    rows = [p[index] for p in pairs]
    accel = np.array([r['curvature']*r['speed']**2 for r in rows])
    lane = -np.array([np.mean(np.array(r['lane'])[:, 0]) for r in rows])
    b = band(accel)
    _, heading, road_curvature = perceive(rows, horizon=25)
    road = road_curvature*np.array([r['speed']**2 for r in rows])
    adjusted = band(accel-road)
    f = np.fft.rfftfreq(len(b), .05)
    spec = np.fft.rfft(b)
    keep = (f >= 1/20) & (f <= 1/7)
    out[name] = {'request_band_rms_mps2': float(np.std(b)),
                 'road_adjusted_proxy_band_rms_mps2': float(np.std(adjusted)),
                 'perceived_heading_band_deg': float(np.std(band(heading))*180/np.pi), 'perceived_offset_band_cm': float(np.std(band(lane))*100),
                 'request_displacement_equivalent_cm': float(np.sqrt(2*np.sum(abs(spec[keep]/(2*np.pi*f[keep])**2)**2))/len(b)*100)}
  base_rows = [p[0] for p in pairs]
  path_rows = [p[1] for p in pairs]
  base_a = band(np.array([r['curvature']*r['speed']**2 for r in base_rows]))
  path_a = band(np.array([r['curvature']*r['speed']**2 for r in path_rows]))
  road_a = band(perceive(base_rows, horizon=25)[2]*np.array([r['speed']**2 for r in base_rows]))
  frequency = np.fft.rfftfreq(len(base_a), .05)
  keep = (frequency >= 1/20) & (frequency <= 1/7)
  parts = np.array([road_a, base_a-path_a, path_a-road_a])
  spectra = np.fft.rfft(parts, axis=1)[:, keep]/(-(2*np.pi*frequency[keep])**2)*100
  covariance = 2*np.real(spectra@spectra.conj().T)/len(base_a)**2
  out['direct_components_equivalent_cm_road_pose_removed_residual'] = np.sqrt(np.diag(covariance)).tolist()
  out['direct_component_covariance_cm2'] = covariance.tolist()
  warped = [p[1] for p in pairs]
  out['max_warp_m'] = max(abs(r['dy']) for r in warped)
  out['max_warp_rad'] = max(abs(r['dpsi']) for r in warped)
  out['within_warp_domain'] = out['max_warp_m'] <= 1 and out['max_warp_rad'] <= .1
  out['n'] = len(pairs)
  out['logged_lane_confident_fraction'] = float(np.mean([r['logged_lane_prob'][1] > .5 and r['logged_lane_prob'][2] > .5 for r, _ in pairs]))
  out['warning'] = 'Smoothed-path request includes true road forcing and model residual. Near-lane estimate is not independent pose truth.'
  return out


def timing(rows, events):
  t = np.array([r['t'] for r in rows])
  accel = np.array([r['curvature']*r['speed']**2 for r in rows])
  result = []
  for ev in events:
    logged = ev['logged']
    start, end = logged['start_t'], logged['end_t']
    logged_accel = np.array([r['logged_curvature']*r['speed']**2 for r in rows])
    sign = np.sign(np.mean(logged_accel[(t >= start) & (t <= end)]))
    threshold = .2*logged['peak']
    grid = np.arange(max(t[0], start-6), min(t[-1], end+6), .05)
    y = sign*np.interp(grid, t, accel)
    middle = (grid >= start) & (grid <= end)
    if not middle.any():
      continue
    peak = np.argmax(np.where(middle, y, -np.inf))
    entering = np.flatnonzero((y[:-1] <= threshold) & (y[1:] > threshold) & (np.arange(len(y)-1) < peak))
    leaving = np.flatnonzero((y[:-1] > threshold) & (y[1:] <= threshold) & (np.arange(len(y)-1) >= peak))
    entry, exit_time = logged['des']['0.2']
    result.append({'start_t': start, 'end_t': end,
                   'entry_minus_logged_s': float(grid[entering[-1]+1]-entry) if len(entering) and entry is not None else None,
                   'exit_minus_logged_s': float(grid[leaving[0]+1]-exit_time) if len(leaving) and exit_time is not None else None})
  return result


def body_timing(trace_path, motion_path, origin, events):
  if not events or not trace_path.exists() or not motion_path.exists():
    return []
  rows = json.loads(trace_path.read_text())
  motion = json.loads(motion_path.read_text())
  if len(motion) < 2:
    return []
  t = np.array([r['t']+origin for r in rows])
  body = np.interp(t, [r['t'] for r in motion], [r['body'] for r in motion])
  proxy = [dict(r, curvature=float(b/max(r['speed']**2, .01))) for r, b in zip(rows, body, strict=True)]
  return timing(proxy, events)


def transported(rows, confidence):
  t = np.array([r['t'] for r in rows])
  if len(rows) < 900 or np.mean(confidence) <= .8 or np.max(np.diff(t)) > .1:
    return None
  offset = np.array([-np.mean(np.array(r['logged_lane'])[:, 0])+r['dy'] for r in rows])
  mask = np.array(confidence, bool)
  grid = np.arange(t[0], t[-1], .05)
  return float(np.std(bandpass(np.interp(grid, t[mask], offset[mask])))*100)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--scratch', type=Path, required=True)
  a = ap.parse_args()
  from tools.odyssey_sim.vsim.metrics import logged_lane_mask
  inv = json.loads((S/'vsim3/inventory.json').read_text())
  historical = json.loads((S/'vsim3/analysis.json').read_text())['windows']
  out = {'direct': [], 'closed': [], 'transfers': []}
  for item, arms in historical.items():
    key = item.split('_')[0]
    w = next(w for w in inv[key]['windows'] if w['id'] == item)
    origin = w['native_start']+.03-w['lo']
    baseline_rows = json.loads((S/'vsim3'/f'{item}_stock.traces.json').read_text())
    baseline_transport = transported(baseline_rows, logged_lane_mask(baseline_rows, key, origin))
    def transport_file(file, key=key, origin=origin):
      if not file.exists():
        return None
      rows = json.loads(file.read_text())
      return transported(rows, logged_lane_mask(rows, key, origin))
    p, q = [a.scratch/f'{item}_{mode}.traces.json' for mode in ('stock', 'path')]
    if p.exists() and q.exists():
      out['direct'].append(dict(window=item, model='default', **direct(json.loads(p.read_text()), json.loads(q.read_text()))))
    refined = a.scratch/f'{item}_refined.traces.json'
    if p.exists() and refined.exists():
      out['direct'].append(dict(window=item, model='default_refined',
                                **direct(json.loads(p.read_text()), json.loads(refined.read_text()))))
    for mode in ('lead', 'lane', 'smooth'):
      p = a.scratch/f'{item}_{mode}.json'
      if not p.exists():
        continue
      d = json.loads(p.read_text())
      r = d.get('canonical_lane_metrics', {})
      out['closed'].append({'window': item, 'arm': mode, 'failure': d.get('closure_failure', d.get('process_failure')),
                             'score': r, 'stock': arms['stock'], 'curves': d.get('matched_curve_metrics'),
                             'transported_lane_cm': transport_file(p.with_suffix('.traces.json')), 'stock_transported_lane_cm': baseline_transport,
                             'body_timing': body_timing(p.with_suffix('.traces.json'), p.with_suffix('.motion.json'),
                                                        origin, arms['stock']['curves']['events'])})
    for model in ('TSFDOM', 'CHEDDAR'):
      p, q = [a.scratch/f'{item}_{model}_{mode}.traces.json' for mode in ('stock', 'path')]
      if p.exists() and q.exists():
        r = direct(json.loads(p.read_text()), json.loads(q.read_text()))
        r['curve_timing'] = timing(json.loads(p.read_text()), arms['stock']['curves']['events'])
        out['direct'].append(dict(window=item, model=model, **r))
      p = a.scratch/f'{item}_{model}_closed.json'
      if p.exists():
        d = json.loads(p.read_text())
        out['closed'].append({'window': item, 'arm': model, 'failure': d.get('closure_failure', d.get('process_failure')),
                               'score': d.get('canonical_lane_metrics'), 'stock': arms['stock'], 'curves': d.get('matched_curve_metrics'),
                               'transported_lane_cm': transport_file(p.with_suffix('.traces.json')), 'stock_transported_lane_cm': baseline_transport,
                             'body_timing': body_timing(p.with_suffix('.traces.json'), p.with_suffix('.motion.json'),
                                                        origin, arms['stock']['curves']['events'])})
  for model in ('TSFDOM', 'CHEDDAR'):
    b = a.scratch/f'{model}_transfer_baseline.traces.json'
    if not b.exists():
      continue
    base = {r['frame']: r for r in json.loads(b.read_text())}
    for period in (7, 10, 15, 20):
      p = a.scratch/f'{model}_sine_{period}.json'
      if p.exists() and p.with_suffix('.traces.json').exists():
        out['transfers'].append(dict(model=model, **transfer_analysis(p, base, [0, 0, 0])))
  (a.scratch/'summary.json').write_text(json.dumps(out, indent=2))
  print('direct',len(out['direct']), 'closed',len(out['closed']), 'transfer',len(out['transfers']))


if __name__ == '__main__':
  main()
