"""Executable invariants for namespace/configuration, signs, feedback and costs."""
import os
import math
import json
from types import SimpleNamespace as NS
from tools.odyssey_sim.closedloop.controller import Controller, configs, model_from_array
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_sim.closedloop.rollout import metrics, ProvisionalPlant
from tools.odyssey_sim.closedloop.data import SCRATCH
import numpy as np


def main():
  os.environ['OPENPILOT_PREFIX']='simclosedcheck'
  meta=metadata('000001a7--a7f62f4b3f')
  checked=[]
  for cfg in configs():
    c=Controller(meta,cfg)
    assert c.lac.pid._k_p[1][0] == (8. if cfg.angle else 250.)
    state=NS(v_ego=25.,a_ego=0.,roll=.01,offset=.2,ratio=17.8,stiffness=1.)
    target,current=.3,.2
    c.update(target,current,state,model_from_array(np.zeros((3,33))))
    angle=c.last_angle
    reconstructed=-c.vm.calc_curvature(math.radians(angle-.2),25.,.01)*25.**2
    assert abs(reconstructed-current)<1e-9
    assert c.nn_active==int(cfg.nnlc)
    assert abs(c.deadband_comp.amount-cfg.kick)<1e-6
    checked.append(cfg.name)
  plant=ProvisionalPlant()
  plant.reset(0.)
  for _ in range(100):
    y=plant.step(.5,25.,0.,0.,.01)
  assert y>0
  m=metrics([0.,0.,0.],[1.,1.,1.],[.1,.1,.1])
  assert m['lataccel']==100 and m['jerk']==0 and m['total']==5000
  out={'config_checks':checked,'vehicle_model_inverse_error':'<1e-9 m/s²','positive_torque_positive_accel':True,'challenge_cost_identity':True}
  (SCRATCH/'closedloop_checks.json').write_text(json.dumps(out,indent=2))
  print('PASS',len(checked),'configurations; inverse, plant sign and costs')


if __name__=='__main__':
  main()
