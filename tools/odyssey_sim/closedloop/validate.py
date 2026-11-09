"""Check real logged-motion command reproduction and simulator conventions."""
import json
import os
import numpy as np
from tools.odyssey_sim.closedloop.data import SCRATCH
from tools.odyssey_sim.replay.harness import replay_configs, ControllerConfig
from openpilot.common.params import Params


def main():
  # Worker 1's adapter defaults to async put; make setup deterministic in this
  # validation process without changing worker 1's files or the real Params tree.
  os.environ['OPENPILOT_PREFIX']='simclosedvalidation'
  orig_put,orig_bool=Params.put,Params.put_bool
  Params.put=lambda self,key,value,**kw:orig_put(self,key,value,block=True)
  Params.put_bool=lambda self,key,value,**kw:orig_bool(self,key,value,block=True)
  cfgs=[ControllerConfig(nnlc=True,version=v) for v in (0,1)]
  q=[]
  frames=0
  for row in replay_configs('000001a7--a7f62f4b3f',cfgs):
    frames+=1
    if row['active'] and not row['pressed']:
      q.append(row)
    if len(q)>=3000:
      break
  out={'frames':frames,'eligible':len(q),'logged_version':sorted({r['version'] for r in q})}
  for cfg in cfgs:
    e=np.array([r[cfg.name]-r['logged'] for r in q])
    out[cfg.name]={'command_mae':float(np.mean(abs(e))),'command_rmse':float(np.sqrt(np.mean(e**2))),
                   'correlation':float(np.corrcoef([r[cfg.name] for r in q],[r['logged'] for r in q])[0,1])}
  (SCRATCH/'closedloop_validation.json').write_text(json.dumps(out,indent=2))
  print(out)


if __name__=='__main__':
  main()
