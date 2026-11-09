"""Adapter to worker 2's documented schema-v1 API. Positive torque raises accel."""
from tools.odyssey_sim.plant.plant import new_state, step, load_params as load_params


class FittedPlant:
  params = None

  def reset(self,y):
    self.state=new_state(params=self.params,lat_accel=y,steering_accel=y)

  def step(self,u,v,a,roll,dt=.1):
    self.state['roll']=float(roll)
    self.state=step(self.state,float(u),float(v),float(dt))
    # LaC measures acceleration inferred from wheel angle, not inertial lateral accel.
    return self.state['steering_accel']
