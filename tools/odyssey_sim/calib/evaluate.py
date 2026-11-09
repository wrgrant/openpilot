"""One-command held-out calibration and candidate evaluation; never deploys.

PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate [--model /absolute/candidate.json]
Model hook accepts an 18-input NNLC JSON, or module:factory through --controller.
"""
import argparse
import importlib
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib.identify import S, TRAIN, TEST, align
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.closedloop.controller import Config, model_from_array
from tools.odyssey_sim.calib.controller import Controller
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_wander import bandpass


def records():
  for p in sorted((S/'closedloop_data').glob('*.npz')):
    if p.stem[:8] not in TRAIN+TEST:
      continue
    d = dict(np.load(p))
    b = dict(np.load(S/'calib'/f'{p.stem[:8]}_aligned.npz'))
    ix = np.clip(np.searchsorted(b['t'],d['t']),0,len(b['t'])-1)
    for k in ['offset','road','lp','rp','body','des','logoutput','blinker','drv','pitch']:
      d[k] = b[k][ix]
    # Legacy cache negates model lane y curvature and centre offset.
    # Calibrated yaw proves lane y curvature already shares body-accel sign.
    d['road'] = -d['road']
    d['offset'] = -d['offset']
    supplement = np.load(S/'calib'/f'{p.stem[:8]}_body.npz')
    for name,col in [('delay',1),('factor',2),('friction',3),('torque_offset',4)]:
      d[name] = align(d['t'],supplement['tuning'],col,.15)
    d['ok'] = (d['active']>.5)&(d['pressed']<.5)&(abs(d['drv'])<500)&(d['blinker']==0)&(d['vEgo']>5)
    d['ok'] &= np.isfinite(d['offset'])&np.isfinite(d['road'])&np.isfinite(d['body'])
    yield p.stem,d


def pieces(d, seconds=60):
  ix = np.where(d['ok'])[0]
  for r in np.split(ix,np.where((np.diff(ix)>1)|(np.diff(d['t'][ix])>.16))[0]+1):
    # 60 s nonoverlapping windows; 10 s warmup + 50 s scored.
    for start in range(0,len(r)-seconds*10+1,seconds*10):
      z = r[start:start+seconds*10]
      if np.mean((d['lp'][z]>.5)&(d['rp'][z]>.5))>.8:
        yield z


def wander(x):
  # odyssey_wander.bandpass expects 20 Hz. Interpolate native 10 Hz samples.
  y = np.interp(np.arange(len(x)*2)/2,np.arange(len(x)),x)
  return float(np.std(bandpass(y))*100)


def stats(target, actual, u, mask):
  adjacent = mask[1:] & mask[:-1]
  return {'cost':float(100*np.mean((target-actual)**2)),
          'reversals_s':float(np.sum((u[1:]*u[:-1]<0)&(abs(np.diff(u))>.01)&adjacent)/(mask.sum()*.01)),
          'saturation_pct':float(100*np.mean(abs(u[mask])>=.99))}


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--model', type=Path)
  ap.add_argument('--controller', help='module:factory(meta, config) -> update-compatible controller')
  ap.add_argument('--planner-gains',type=float,nargs=2,default=None)
  ap.add_argument('--logged-plan-only',action='store_true')
  ap.add_argument('--output',default='evaluation.json')
  args = ap.parse_args()
  output = args.output
  data = list(records())
  X,Y = [],[]
  for route,d in data:
    if route[:8] not in TRAIN:
      continue
    for r in pieces(d):
      # Smooth quantized camera offset before numerical differentiation.
      off = np.convolve(np.pad(d['offset'][r],(5,5),mode='edge'),np.ones(11)/11,mode='valid')
      X.extend(np.c_[off,np.gradient(off,.1),np.ones(len(off))][10:-10:5])
      Y.extend((d['des'][r]-d['road'][r])[10:-10:5])
  coef = np.linalg.lstsq(np.array(X),np.array(Y),rcond=None)[0]
  gains = coef[:2] if args.planner_gains is None else np.array(args.planner_gains)
  bias = float(coef[2]) if args.planner_gains is None else 0.
  result = {'planner_fit':coef.tolist(),'planner_train_r2':float(1-np.var(np.array(X)@coef-Y)/np.var(Y)),
            'planner_gains_used':gains.tolist(),'planner_bias_used':bias,'tolerance':.30,'rows':[],'ranking_authorized':False}
  for route,d in data:
    if route[:8] not in TEST:
      continue
    meta = metadata(route)
    param = meta['params']
    is_nn = param.get('NeuralNetworkLateralControl',b'0') == b'1'
    print('META',route,'NNLC',is_nn,'AB',param.get('NnlcAB'),flush=True)
    body = np.load(S/'calib'/f'{route[:8]}_body.npz')
    selections = list(pieces(d))
    if not selections:
      selections = list(pieces(d,30))
    for j,r in enumerate(selections):
      w = {k:v[r] for k,v in d.items()}
      for mode in (('logged_plan',) if args.logged_plan_only else ('logged_plan','lane_loop')):
        cfg = Config(nnlc=is_nn)
        c = Controller(meta,cfg)
        if args.controller:
          module,factory = args.controller.split(':')
          c = getattr(importlib.import_module(module),factory)(meta,cfg)
        if args.model:
          from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel
          c = Controller(meta,Config(nnlc=True))
          stock_path = Path.cwd()/'openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json'
          c.lac.extension.model = SafeNNTorqueModel(args.model,stock_path)
          if c.lac.extension.model.input_size != 18:
            raise ValueError('Use --controller adapter for models with inputs other than 18')
        plant = Plant()
        plant.reset(w['actual'][0],w['body'][0],-w['steerCommand'][0])
        e = float(w['offset'][0])
        ed = float((w['offset'][5]-w['offset'][0])/.5)
        ys,es,us,ds = [],[],[],[]
        for i in range(len(r)):
          state = NS(v_ego=w['vEgo'][i],a_ego=w['aEgo'][i],roll=w['roll'][i],offset=w['angleOffset'][i],ratio=w['steerRatio'][i],stiffness=w['stiffness'][i])
          for name in ('delay','factor','friction','torque_offset','pitch'):
            setattr(state,name,float(w[name][i]))
          state.limited = abs(plant.u-float(us[-1] if us else plant.u))>.01
          model = model_from_array(w['plans'][i])
          for _ in range(10):
            target = w['des'][i] if mode=='logged_plan' else w['road'][i]+gains[0]*e+gains[1]*ed+bias
            u = c.update(target,plant.y,state,model)
            plant.step(u,state.v_ego,state.roll)
            ed += (w['road'][i]-plant.body)*.01
            e += ed*.01
            us.append(u)
          ys.append(c.measured_lataccel)
          es.append(e)
          ds.append(target)
        ys,es,us,ds = map(np.array,(ys,es,us,ds))
        for lo,hi in [(10,20),(20,35),(35,50),(50,200)]:
          mask = (w['vEgo']*2.23694>=lo)&(w['vEgo']*2.23694<hi)&(np.arange(len(r))>=100)
          if mask.sum()<100:
            continue
          # Native recorded torqueState output; matched 100 Hz timing and masks.
          t = np.arange(w['t'][0],w['t'][-1]+.001,.01)[:len(us)]
          recu = -align(t,body['torque'],3,.15)
          nm = np.repeat(mask,10)[:len(recu)]
          rec = stats(w['des'][mask],w['actual'][mask],recu,nm)
          sim = stats(ds[mask],ys[mask],us,np.repeat(mask,10))
          # Wander needs uninterrupted >=40 s within band; otherwise unavailable.
          runs = np.split(np.where(mask)[0],np.where(np.diff(np.where(mask)[0])>1)[0]+1)
          long = max(runs,key=len)
          if len(long)>=400:
            rec['wander_cm'] = wander(w['offset'][long])
            sim['wander_cm'] = wander(es[long])
          errors = {k:abs(sim[k]-rec[k])/max(abs(rec[k]),{'cost':.01,'reversals_s':.1,'saturation_pct':1.,'wander_cm':1.}[k]) for k in rec}
          row = {'route':route,'window':j,'band':f'{lo}-{hi}','mode':mode,'nnlc':is_nn,'recorded':rec,'simulated':sim,'relative_error':errors,
                 'pass':all(v<=.30 for v in errors.values()) and 'wander_cm' in rec,'n':int(mask.sum())}
          result['rows'].append(row)
          print(json.dumps(row),flush=True)
      (S/'calib'/output).write_text(json.dumps(result,indent=2))
  result['ranking_authorized'] = bool(result['rows']) and all(r['pass'] for r in result['rows'])
  (S/'calib'/output).write_text(json.dumps(result,indent=2))


if __name__ == '__main__':
  main()
