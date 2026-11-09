"""Frequency-domain decomposition with cross terms; no causal percentage claims."""
import argparse
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.calib.identify import S
from tools.odyssey_sim.vsim.transfer import perceive
from tools.odyssey_sim.vsim.closed_loop import native_data


def decompose(rows, native, origin, transfer):
  t = np.array([r['t'] for r in rows])
  grid = np.arange(t[0], t[-1], .05)
  v = np.interp(grid, t, [r['speed'] for r in rows])
  offset, _, road = perceive(rows, 'logged_', 25)
  def interp(y):
    return np.interp(grid, t, y)
  n = len(grid)
  f = np.fft.rfftfreq(n, .05)
  keep = (f >= 1/20) & (f <= 1/7)
  def fft(y):
    y = y-np.polyval(np.polyfit(grid-grid[0], y, 1), grid-grid[0])
    return np.fft.rfft(y)[keep]
  freqs = np.array([1/r['period'] for r in transfer])
  order = np.argsort(freqs)
  gains = np.array([r['plan_accel_per_offset']['gain'] for r in transfer])[order]
  phases = np.unwrap(np.deg2rad([r['plan_accel_per_offset']['phase_deg'] for r in transfer])[order])
  h = np.interp(f[keep], freqs[order], gains)*np.exp(1j*np.interp(f[keep], freqs[order], phases))
  request = fft(interp(np.array([r['logged_curvature']*r['speed']**2 for r in rows])))
  road_component = fft(interp(road)*v*v)
  response = h*fft(interp(offset))
  residual = request-road_component-response
  scale = -(2*np.pi*f[keep])**-2
  # cm-equivalent: displacement under ideal acceleration execution, not measured lane contribution.
  components = np.array([road_component, response, residual])*scale*100
  cov = 2*np.real(components@components.conj().T)/n**2
  body = fft(np.interp(grid+origin, native['t'], native['body']))
  gap = (request-body)*scale*100
  lane_spectrum = fft(interp(offset))*100
  ideal = float(np.sqrt(2*np.sum(abs(lane_spectrum+gap)**2))/n)
  confidence = np.array([r['logged_lane_prob'][1]>.5 and r['logged_lane_prob'][2]>.5 for r in rows])
  return {'n': n, 'lane_confident_fraction': float(confidence.mean()),
          'cm_equivalent_components_road_response_residual': np.sqrt(np.diag(cov)).tolist(),
          'covariance_cm2': cov.tolist(), 'sum_with_cross_terms_cm': float(np.sqrt(cov.sum())),
          'perfect_execution_change_cm_equivalent': float(np.sqrt(2*np.sum(abs(gap)**2))/n),
          'observed_lane_band_cm': float(np.sqrt(2*np.sum(abs(fft(interp(offset)))**2))/n*100),
          'fixed_logged_request_ideal_execution_lane_cm': ideal,
          'fixed_request_caveat': 'Holds recorded request and road fixed; no replanning, no independent lane truth. Not a closed-loop benefit.',
          'causal_removable_wander_cm': None,
          'warning': ('Road is a model-derived proxy. Single-window injected response is transported without validation. ' +
                      'Components correlate; cm do not add. Tracking gap is a change, not guaranteed improvement.')}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--scratch', type=Path, required=True)
  a = ap.parse_args()
  transfer = [r for r in json.loads((S/'vsim2/transfer.json').read_text())['injections'] if r['name'].startswith('sine_')]
  inventory = json.loads((S/'vsim3/inventory.json').read_text())
  out = {'decomposition': [], 'runs': []}
  for key in ('000001a3', '000001a7', '000001aa', '000001ab'):
    native = native_data(key)
    for w in inventory[key]['windows']:
      p = a.scratch/f'{w["id"]}_stock.traces.json'
      if p.exists():
        rows = json.loads(p.read_text())
        origin = w['native_start']+.03-w['lo']
        d = decompose(rows, native, origin, transfer)
        d['window'] = w['id']
        out['decomposition'].append(d)
      for mode in ('lead', 'lane', 'smooth'):
        p = a.scratch/f'{w["id"]}_{mode}.json'
        if not p.exists():
          continue
        r = json.loads(p.read_text())
        out['runs'].append({'window': w['id'], 'mode': mode, 'failure': r.get('closure_failure', r.get('process_failure')),
                            'lane': r.get('canonical_lane_metrics'), 'curve': r.get('matched_curve_metrics')})
  (a.scratch/'analysis.json').write_text(json.dumps(out, indent=2))
  print('decomposed', len(out['decomposition']), 'closed-loop',len(out['runs']))


if __name__ == '__main__':
  main()
