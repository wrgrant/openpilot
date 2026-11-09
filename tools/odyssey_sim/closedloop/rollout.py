"""100 Hz real control / 10 Hz plant, reusing comma's actual rollout for generic plant.

Physical plants use the same history, warmup and metric convention. The planned
trajectory is replayed open-loop; only actuator -> motion -> steering feedback closes.
"""
import argparse
import os
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.closedloop.data import SCRATCH, ROUTES
from tools.odyssey_sim.closedloop.controller import Controller, configs, model_from_array
from tools.odyssey_sim.replay.harness import metadata

BANDS=[(0,10),(10,20),(20,35),(35,50),(50,200)]


class ProvisionalPlant:
  def __init__(self,tau=.4):
    self.tau=tau
    self.y=0.

  def reset(self, y):
    self.y=y

  def step(self,u,v,a,roll,dt=.1):
    drive=np.sign(u)*max(abs(u)-.03,0.)
    eq=1.47*drive - .145*np.tanh(self.y/.08) - .055 + 9.81*roll
    self.y+= (1-np.exp(-dt/self.tau))*(eq-self.y)
    return self.y


def windows(d,seconds=30):
  """One maximum-curvature, eligible, contiguous window per route and speed band."""
  n=int(seconds*10)
  good=(d['active']>.5)&(d['pressed']<.5)&(d['vEgo']>1.)
  mph=d['vEgo']*2.23694
  for low,high in BANDS:
    valid=good&(mph>=low)&(mph<high)
    candidates=[]
    for i in range(0,len(good)-n,n//2):
      if np.sum(valid[i+100:i+n])>=30 and np.max(np.diff(d['t'][i:i+n]))<.2:
        score=np.mean(np.abs(d['targetLateralAcceleration'][i+100:i+n])[valid[i+100:i+n]])
        candidates.append((score,i))
    if candidates:
      _,start=max(candidates)
      w={k:v[start:start+n] for k,v in d.items()}
      w['score_mask']=valid[start:start+n]
      yield f'{low}-{high}',w


def metrics(target,pred,u,dt=.1,mask=None):
  target,pred,u=map(np.asarray,(target,pred,u))
  mask=np.ones(len(target),dtype=bool) if mask is None else mask
  adjacent=mask[1:]&mask[:-1]
  lat=float(np.mean((target[mask]-pred[mask])**2)*100)
  jerk=float(np.mean((np.diff(pred)[adjacent]/dt)**2)*100)
  # Matched crossings around meaningful curve entries/exits, +/- 2 s search.
  on=np.abs(target)>.3
  entries=np.where(np.diff(on.astype(int))==1)[0]+1
  exits=np.where(np.diff(on.astype(int))==-1)[0]+1
  def crossing(events,entry):
    out=[]
    for i in events:
      if not mask[i]:
        continue
      lo,hi=max(1,i-20),min(len(pred),i+21)
      cross_mask=(np.abs(pred)>.3) if entry else (np.abs(pred)<.3)
      hit=np.where(cross_mask[lo:hi]&~cross_mask[lo-1:hi-1])[0]+lo
      if len(hit):
        j=hit[np.argmin(abs(hit-i))]
        out.append(float((j-i)*dt))
    return (float(np.median(out)) if out else None),len(out)
  entry,ne=crossing(entries,True)
  exit_,nx=crossing(exits,False)
  return {'lataccel':lat,'jerk':jerk,'total':50*lat+jerk,'saturation_pct':float(np.mean(abs(u[mask])>=.99)*100),
          'flips_s':float(np.sum((u[1:]*u[:-1]<0)&(abs(u[1:]-u[:-1])>.01)&adjacent)/(sum(mask)*dt)),
          'entry_s':entry,'exit_s':exit_,'entries':ne,'exits':nx,'n':int(sum(mask))}


def run_physical(d,controller,plant):
  plant.reset(float(d['actual'][0]))
  y=float(d['actual'][0])
  pred=[]
  commands=[]
  native=[]
  for i in range(len(d['t'])):
    state=NS(v_ego=d['vEgo'][i],a_ego=d['aEgo'][i],roll=d['roll'][i],offset=d['angleOffset'][i],ratio=d['steerRatio'][i],stiffness=d['stiffness'][i])
    model=model_from_array(d['plans'][i])
    # 10 Hz logged plan/state held over ten native 100 Hz controller updates.
    for _ in range(10):
      u=controller.update(d['targetLateralAcceleration'][i],y,state,model)
      y=plant.step(u,state.v_ego,state.a_ego,state.roll,.01)
      native.append(u)
    pred.append(y)
    commands.append(u)
  controller.native_commands=np.asarray(native)
  return np.asarray(pred),np.asarray(commands)


class ChallengeAdapter:
  def __init__(self,controller,d):
    self.controller,self.d=controller,d
    self.i=20
  def update(self,target,current,state,future_plan=None):
    i=self.i
    self.i+=1
    s=NS(v_ego=state.v_ego,a_ego=state.a_ego,roll=self.d['roll'][i],offset=self.d['angleOffset'][i],ratio=self.d['steerRatio'][i],stiffness=self.d['stiffness'][i])
    model=model_from_array(self.d['plans'][i])
    for _ in range(10):
      u=self.controller.update(target,current,s,model)
    return u


def run_tiny(d,controller,tag):
  # Import comma's harness verbatim, rather than cloning its inference or loop.
  sys.path.insert(0,str(Path.home()/'Code/ext/controls_challenge'))
  from tinyphysics import TinyPhysicsModel,TinyPhysicsSimulator
  csv=SCRATCH/'closedloop_data'/f'window_{tag}.csv'
  cols=['roll','vEgo','aEgo','targetLateralAcceleration','steerCommand']
  np.savetxt(csv,np.column_stack([d[k] for k in cols]),delimiter=',',header=','.join(cols),comments='')
  model=TinyPhysicsModel(str(Path.home()/'Code/ext/controls_challenge/models/tinyphysics.onnx'),False)
  sim=TinyPhysicsSimulator(model,str(csv),ChallengeAdapter(controller,d))
  sim.rollout()
  return np.asarray(sim.current_lataccel_history),np.asarray(sim.action_history)


def main():
  p=argparse.ArgumentParser()
  p.add_argument('--plant',default='provisional',choices=['provisional','tiny','odyssey','public'])
  p.add_argument('--seconds',type=int,default=30)
  p.add_argument('--routes',nargs='*',default=ROUTES)
  p.add_argument('--config-limit',type=int,default=99)
  args=p.parse_args()
  os.environ["OPENPILOT_PREFIX"]="simclosed" if args.plant=="provisional" else "simclosed"+args.plant
  if args.plant in ('odyssey','public'):
    from tools.odyssey_sim.closedloop.worker_plant import FittedPlant, load_params
    FittedPlant.params=load_params()
    if args.plant=='public':
      FittedPlant.params.update(FittedPlant.params['public_model'])
    (SCRATCH/f'closedloop_{args.plant}_plant_snapshot.json').write_text(json.dumps(FittedPlant.params,indent=2))
  rows=[]
  traces={}
  for route in args.routes:
    path=SCRATCH/'closedloop_data'/f'{route}.npz'
    if not path.exists():
      print('MISSING',path,flush=True)
      continue
    d=dict(np.load(path))
    meta=metadata(route)
    for band,w in windows(d,args.seconds):
      for cfg in list(configs())[:args.config_limit]:
        controller=Controller(meta,cfg)
        if args.plant=='tiny':
          pred,u=run_tiny(w,controller,f'{route}_{band}')
        else:
          if args.plant in ('odyssey','public'):
            from tools.odyssey_sim.closedloop.worker_plant import FittedPlant
            plant=FittedPlant()
          else:
            plant=ProvisionalPlant()
          pred,u=run_physical(w,controller,plant)
        start=100
        m=metrics(w['targetLateralAcceleration'][start:],pred[start:],u[start:],mask=w['score_mask'][start:])
        if hasattr(controller,'native_commands'):
          native=controller.native_commands[start*10:]
          mask=np.repeat(w['score_mask'][start:],10)
          adjacent=mask[1:]&mask[:-1]
          m['saturation_pct']=float(np.mean(abs(native[mask])>=.99)*100)
          m['flips_s']=float(np.sum((native[1:]*native[:-1]<0)&(abs(native[1:]-native[:-1])>.01)&adjacent)/(sum(mask)*.01))
        m.update(plant=args.plant,route=route,band=band,config=cfg.name,nn_active=controller.nn_active,
                 logged_lataccel=float(np.mean((w['targetLateralAcceleration'][start:]-w['actual'][start:])[w['score_mask'][start:]]**2)*100),
                 start_t=float(w['t'][0]))
        rows.append(m)
        traces[f'{route}_{band}_{cfg.name}_pred']=pred
        traces[f'{route}_{band}_{cfg.name}_u']=u
        print(json.dumps(m),flush=True)
        (SCRATCH/f'closedloop_{args.plant}.json').write_text(json.dumps(rows,indent=2))
  np.savez_compressed(SCRATCH/f'closedloop_{args.plant}_traces.npz',**traces)


if __name__=='__main__':
  main()
