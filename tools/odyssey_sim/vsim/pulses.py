"""Schedule-aligned pulse validation using body yaw, never VSA LAT_ACCEL.

Fit steering gain/friction/lag scales on 1a3 only; report untouched 1a7.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np
from tools.odyssey_eps_report import load, runs, schedule
from tools.odyssey_sim.vsim.closed_loop import native_data
from tools.odyssey_sim.vsim.perturbation import PerturbationPlant
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.calib.identify import align


def extract(route, cache):
  key = route.name.split('_')[1].split('--')[0]
  path = cache / f'{key}_pulses_v2.npz'
  if path.exists():
    return dict(np.load(path))
  d, names = load(str(route))
  native = native_data(key)
  # Reuse schedule parsing, replacing the invalid VSA channel with validated yaw.
  out = defaultdict(list)
  for r in runs(d):
    name = names[int(d['maneuver'][r[0]])]
    if not name.startswith('held pulses'):
      continue
    f, maneuver = schedule(name)
    width = float(name.split()[2].rstrip('s'))
    for j, hold in enumerate(maneuver.holds):
      onset = d['t'][r[0]] + hold[0] + .3
      time = onset - .2 + np.arange(170)*.01
      if time[-1] > d['t'][r[-1]]:
        continue
      level = float(f(hold[0] + .3 + width/2))
      sign = np.sign(level) if level else (1 if j % 2 == 0 else -1)*(1 if 'left first' in name else -1)
      x = {k: align(time, np.c_[native['t'], native[k]], 1, .04) for k in ('v','roll','cmd','act','body','yaw','drv','pressed')}
      if not np.isfinite(list(x.values())).all():
        continue
      for k, v in x.items():
        out[k].append(v)
      out['level'].append(round(abs(level), 3))
      out['direction'].append(sign)
      out['width'].append(width)
      out['onset'].append(onset)
  out = {k: np.array(v) for k, v in out.items()}
  np.savez_compressed(path, **out)
  return out


def prediction(d, scales):
  params = json.loads(json.dumps(Plant().params))
  for indexes, scale in zip(([0,1,2],[4,5,6],[3]), scales, strict=True):
    for i in indexes:
      params['steering'][i] *= float(scale)
  pred = []
  for j in range(len(d['level'])):
    p = PerturbationPlant(params)
    p.reset(d['act'][j,0], d['body'][j,0], d['cmd'][j,0])
    row = []
    for i in range(170):
      # Measured applied pulse vs frozen pre-pulse command, same initial states.
      _, a = p.step(d['cmd'][j,i], d['cmd'][j,0], d['v'][j,i], d['roll'][j,i], 0, 0)
      row.append(a)
    pred.append(np.array(row)[20:] - row[20])
  return np.array(pred)


def targets(d):
  y = d['body'][:,20:] - d['body'][:,20:21]
  out = y.copy()
  for j in range(len(y)):
    clean = ~np.any((d['pressed']>0)|(abs(d['drv'])>=500),axis=1)
    ctrl = (d['level']==0)&(d['width']==d['width'][j])&(d['direction']==d['direction'][j])&clean
    if ctrl.any():
      out[j] -= y[ctrl].mean(axis=0)
    else:
      raise ValueError('no matched zero-pulse holds')
  return out


def summarize(d, pred):
  truth = targets(d)
  pred = pred.copy()
  # Apply identical matched no-pulse subtraction to observed and predicted responses.
  original = pred.copy()
  for j in range(len(pred)):
    clean = ~np.any((d['pressed']>0)|(abs(d['drv'])>=500),axis=1)
    ctrl = (d['level']==0)&(d['width']==d['width'][j])&(d['direction']==d['direction'][j])&clean
    pred[j] -= original[ctrl].mean(axis=0)
  rows = []
  for width in sorted(set(d['width'])):
    for level in sorted(set(d['level'])):
      for direction in (-1,1):
        mask=(d['width']==width)&(d['level']==level)&(d['direction']==direction)
        dirty = int(np.sum(mask & np.any((d['pressed']>0)|(abs(d['drv'])>=500),axis=1)))
        mask &= ~np.any((d['pressed']>0)|(abs(d['drv'])>=500),axis=1)
        if not mask.any():
          continue
        n = min(150, int(round((width + .7)*100)))
        a, b = truth[mask,:n].mean(axis=0), pred[mask,:n].mean(axis=0)
        v = d['v'][mask,20:20+n].mean(axis=0)
        rows.append({'width':float(width),'level':float(level),'direction':direction,'n':int(mask.sum()),
                     'excluded_driver_override_pulses':dirty,
                     'body_observed_end':float(a[-20:].mean()),'body_predicted_end':float(b[-20:].mean()),
                     'body_trace_rmse':float(np.sqrt(np.mean((a-b)**2))),
                     'yaw_trace_rmse':float(np.sqrt(np.mean(((a-b)/np.maximum(v,.1))**2))),
                     'observed_peak_time':float(np.argmax(abs(a))*.01),'predicted_peak_time':float(np.argmax(abs(b))*.01),
                     'observed_trace':a.tolist(),'predicted_trace':b.tolist()})
  return rows


def main():
  ap=argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--scratch',type=Path,required=True)
  args=ap.parse_args()
  data={}
  for key in ('000001a3','000001a7'):
    route=next((Path.home()/'comma-logs').glob(f'*{key}*'))
    data[key]=extract(route,args.scratch)
    print(key,len(data[key].get('level', [])),flush=True)
  scales = [1.,1.,1.]
  fit_status = 'unavailable: 1a3 contains older torque pulses, no frozen no-pulse holds; no fitting on validation route'
  result={'fit_routes':[], 'requested_fit_route':'000001a3', 'fit_status':fit_status,
          'validation_routes':['000001a7'],'gain_friction_lag_scales':scales,
          'rows':{k:summarize(d,prediction(d,scales)) for k,d in data.items() if 'level' in d}}
  (args.scratch/'pulses.json').write_text(json.dumps(result,indent=2))


if __name__ == '__main__':
  main()
