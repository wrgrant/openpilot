"""Paired real-model transfer measurements; phase positive means output leads."""
import argparse
import json
from pathlib import Path
import numpy as np


def perceive(rows, prefix='', horizon=30):
  x = 192*(np.arange(33)/32)**2
  mask = x <= horizon
  lane = np.array([r[prefix+'lane'] for r in rows]).mean(axis=1)
  co = np.polynomial.polynomial.polyfit(x[mask], lane[:,mask].T, 3)
  return -co[0], -np.arctan(co[1]), 2*co[2]


def harmonic(t, y, period):
  w = 2*np.pi/period
  design = np.c_[np.sin(w*t),np.cos(w*t),np.ones(len(t)),t-t.mean()]
  co = np.linalg.lstsq(design,y,rcond=None)[0]
  return complex(co[0],co[1])


def ratio(a, b):
  z = a/b
  return {'gain':float(abs(z)), 'phase_deg':float(np.angle(z,deg=True))}


def analyze(path, baseline, coefficients):
  meta = json.loads(path.read_text())
  data = json.loads(path.with_suffix('.traces.json').read_text())
  pairs = [(r,baseline[r['frame']]) for r in data if r['frame'] in baseline and r['warm_frames']>=200]
  rows, base = zip(*pairs,strict=True)
  t = np.array([r['t'] for r in rows])
  period = meta['injection']['period']
  off, heading, _ = perceive(rows, horizon=25)
  boff, bheading, _ = perceive(base, horizon=25)
  dy, psi = [np.array([r[k] for r in rows]) for k in ('dy','dpsi')]
  accel = np.array([(r['curvature']-b['curvature'])*r['speed']**2 for r,b in pairs])
  hy,hp = harmonic(t,dy,period),harmonic(t,psi,period)
  return {'name':path.stem,'n':len(rows),'period':period,
          'offset':ratio(harmonic(t,off-boff,period),hy),'heading':ratio(harmonic(t,heading-bheading,period),hp),
          'plan_accel_per_offset':ratio(harmonic(t,accel,period),hy),
          'natural_fit_predicted_accel_per_offset':ratio(
            harmonic(t,coefficients[0]*(off-boff)+coefficients[1]*(heading-bheading),period),hy),
          'amplitude':meta['injection']['dy']}


def natural(rows):
  t=np.array([r['t'] for r in rows])
  offset,heading,road=perceive(rows,'logged_')
  v=np.array([r['speed'] for r in rows])
  accel=np.array([r['logged_curvature'] for r in rows])*v*v
  result=[]
  for period in (7,10,15,20):
    # A single natural window cannot independently identify two correlated inputs.
    # Report observed complex ratios and conditioning rather than imply causality.
    hy=harmonic(t,offset,period)
    result.append({'period':period,'observed_accel_per_offset':ratio(harmonic(t,accel,period),hy),
                   'road_adjusted_accel_per_offset':ratio(harmonic(t,accel-road*v*v,period),hy),
                   'offset_amplitude_m':abs(hy),'heading_amplitude_rad':abs(harmonic(t,heading,period))})
  return result


def fit_natural():
  from tools.odyssey_sim.calib.identify import S, align
  from tools.odyssey_wander import bandpass
  designs, outputs, windows = [], [], []
  for key in ('000001a3','000001a7'):
    d=np.load(S/'calib3'/f'{key}.npz')
    native=d['native']
    v=align(d['t'],native,1,.1)
    active=align(d['t'],native,7,.1)
    pressed=align(d['t'],native,5,.1)
    blink=align(d['t'],native,6,.1)
    # Cubic near-field lane origin, 25m horizon, from existing observer cache.
    off, heading, road = d['geometry'][:,4,:].T
    ok=(v>5)&(active>0)&(pressed==0)&(blink==0)&np.isfinite(off+heading+road)
    ix=np.where(ok)[0]
    runs=np.split(ix,np.where((np.diff(ix)>1)|(np.diff(d['t'][ix])>.1))[0]+1)
    for run in runs:
      for start in range(0,len(run)-1200+1,1200):
        z=run[start:start+1200]
        x=np.c_[-off[z],-heading[z],road[z]*v[z]**2]
        x=np.column_stack([bandpass(c) for c in x.T])
        y=bandpass(d['action'][z]*v[z]**2)
        designs.append(x)
        outputs.append(y)
        windows.append({'route':key,'t0':float(d['t'][z[0]]),'n':len(z)})
  x,y=np.concatenate(designs),np.concatenate(outputs)
  coef=np.linalg.lstsq(x,y,rcond=None)[0]
  return {'windows':windows,'coefficients_offset_heading_road':coef.tolist(),
          'r2':float(1-np.var(y-x@coef)/np.var(y)),
          'standardized_condition':float(np.linalg.cond(x/x.std(axis=0))),
          'interpretation':'Instantaneous band-limited observational fit; phases and causal response remain unidentifiable.'}


def main():
  ap=argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--scratch',type=Path,required=True)
  ap.add_argument('--baseline',type=Path,required=True)
  args=ap.parse_args()
  baseline=json.loads(args.baseline.read_text())
  base={r['frame']:r for r in baseline}
  fit = fit_natural()
  paths = [p for p in sorted(args.scratch.glob('*.json'))
           if p.stem.startswith(('sine_','ramp_','step_')) and not p.stem.endswith('.traces')]
  rows = [analyze(p,base,fit['coefficients_offset_heading_road']) for p in paths]
  result={'injections':rows,'natural_training_window':natural(baseline),
          'natural_multivariable_fit': fit,
          'natural_warning': ('Observational ratios confound road and correlated heading/offset; ' +
                                      'not an identified multivariable transfer function.')}
  (args.scratch/'transfer.json').write_text(json.dumps(result,indent=2))
  print(json.dumps(result,indent=2))


if __name__=='__main__':
  main()
