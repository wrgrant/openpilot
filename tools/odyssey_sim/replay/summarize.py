"""Compute descriptive open-loop metrics, retaining paired route/frame masks."""
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.replay.harness import ControllerConfig

def uniform_filter1d(values, size):
  padded = np.pad(np.asarray(values, dtype=float), (size//2, size//2), mode='edge')
  sums = np.concatenate(([0.0], np.cumsum(padded)))
  return (sums[size:] - sums[:-size]) / size


def binary_erosion(mask, iterations):
  result = uniform_filter1d(mask, 2*iterations+1) > 1-1e-9
  result[:iterations] = False
  result[-iterations:] = False
  return result


BANDS = [(0,10),(10,20),(20,35),(35,50),(50,200)]
CONFIGS = [ControllerConfig(n,d).name for d in (None,.32,.45) for n in (True,False)]


def correlation(a,b):
  return float(np.corrcoef(a,b)[0,1]) if len(a)>2 and np.std(a)>0 and np.std(b)>0 else None


def summarize(files):
  data = [dict(np.load(f)) for f in files]
  validation = []
  for f,d in zip(files,data,strict=True):
    mask=d['active'] & ~d['pressed']
    err=d['nnlc_v0_logged'][mask]-d['logged'][mask]
    validation.append({'route': Path(f).stem,'n': int(mask.sum()),'correlation': correlation(d['logged'][mask],d['nnlc_v0_logged'][mask]),
                           'rms': float(np.sqrt(np.mean(err**2))),'p99': float(np.quantile(abs(err),.99)),
                           'factor': np.quantile(d['factor'][mask],[.05,.5,.95]).tolist(),
                           'friction': np.quantile(d['friction'][mask],[.05,.5,.95]).tolist(),
                           'offset': np.quantile(d['offset'][mask],[.05,.5,.95]).tolist(),'versions': np.unique(d['version']).tolist()})
  result = {'validation': validation, 'bands': []}
  for d in data:
    d['smooth_desired'] = uniform_filter1d(d['desired'], size=31)
    d['da'] = np.gradient(d['smooth_desired'], .01)
    d['safe'] = binary_erosion(d['active'] & ~d['pressed'], iterations=110)
    for cfg in CONFIGS:
      d[cfg+'_dq'] = np.gradient(uniform_filter1d(-d[cfg], size=31), .01)
  for low,high in BANDS:
    band={'mph': f'{low}-{high if high<200 else "+"}', 'configs': {}}
    masks=[d['active'] & ~d['pressed'] & (d['speed']*2.236936>=low) & (d['speed']*2.236936<high) for d in data]
    actual=np.concatenate([(d['desired']-d['actual'])[m] for d,m in zip(data,masks,strict=True)])
    band['seconds']=len(actual)*.01
    band['recorded_tracking_rms']=float(np.sqrt(np.mean(actual**2)))
    for cfg in CONFIGS:
      torques=np.concatenate([d[cfg][m] for d,m in zip(data,masks,strict=True)])
      flips=0
      raw_flips=0
      tv=[]
      for d,m in zip(data,masks,strict=True):
        q=d[cfg]
        pairs=m[1:] & m[:-1] & (np.diff(d['t'])<.03)
        raw_flips+=int(((q[1:]*q[:-1]<0)&pairs).sum())
        idx=np.flatnonzero(m & (abs(q)>.01))
        if len(idx)>1:
          good=(np.diff(idx)<100)&(d['t'][idx[1:]]-d['t'][idx[:-1]]<1)&(q[idx[1:]]*q[idx[:-1]]<0)
          # Do not connect across a disengagement or a different speed band.
          for j in np.flatnonzero(good):
            flips+=int(m[idx[j]:idx[j+1]+1].all())
        tv.extend((abs(np.diff(q))[pairs]/.01).tolist())
      metrics={'saturation_pct': float(np.mean(abs(torques)>=.999)*100), 'flips_per_s': flips/(len(torques)*.01),
                   'raw_flips_per_s': raw_flips/(len(torques)*.01), 'torque_rms': float(np.sqrt(np.mean(torques**2))),
                   'mean_abs_torque_rate': float(np.mean(tv))}
      for phase in ['entry','exit']:
        lags=np.arange(-100,101,5)
        scores=[]
        ns=[]
        for lag in lags:
          aa=[]
          bb=[]
          for d,m in zip(data,masks,strict=True):
            desired=d['smooth_desired']
            da=d['da']
            dq=d[cfg+'_dq']
            slope=da*np.sign(desired)
            ok=d['safe']&m&(abs(desired)>.1)
            ok &= slope>.05 if phase=='entry' else slope<-.05
            idx=np.flatnonzero(ok)
            shifted=idx+lag
            good=(shifted>=0)&(shifted<len(m))
            idx=idx[good]
            shifted=shifted[good]
            # Eliminate missing-segment gaps.
            good=abs((d['t'][shifted]-d['t'][idx])-lag*.01)<.04
            aa.extend(da[idx[good]])
            bb.extend(dq[shifted[good]])
          scores.append(correlation(aa,bb) or -1)
          ns.append(len(aa))
        best=int(np.argmax(scores))
        metrics[phase+'_lag_s']=float(lags[best]*.01)
        metrics[phase+'_correlation']=scores[best]
        metrics[phase+'_samples']=ns[best]
      band['configs'][cfg]=metrics
    result['bands'].append(band)
  return result


if __name__ == '__main__':
  import argparse
  p=argparse.ArgumentParser()
  p.add_argument('files',nargs='+')
  p.add_argument('--output',required=True)
  a=p.parse_args()
  Path(a.output).write_text(json.dumps(summarize(a.files),indent=2))
