"""Schema-v2 scalar step API, acceleration-positive applied CAN command.

The steering state is the controller's VehicleModel-equivalent acceleration;
body is v*yaw rate. No additional transport delay is resolved at 10 Hz.
"""
import json
import math
from tools.odyssey_sim.calib.identify import S


def interp(v, p):
  if v <= 10:
    return p[0]
  if v >= 30:
    return p[2]
  i = 0 if v < 20 else 1
  return p[i] + (p[i+1]-p[i])*(v-(10+10*i))/10


class Plant:
  def __init__(self, params=None):
    self.params = params or json.loads((S/'calib'/'plant_v2.json').read_text())
    self.y = self.body = self.u = 0.

  def reset(self, y, body=None, u=0):
    self.y = float(y)
    self.body = float(y if body is None else body)
    self.u = float(u)

  def step(self, u, v, roll, dt=.01):
    p,b = self.params['steering'],self.params['body']
    # Honda's 0.33-second full-scale slew; normalized max delta ~0.03/tick.
    self.u += max(-3*dt, min(3*dt, u-self.u))
    eff = math.copysign(max(abs(self.u)-.028,0),self.u)
    push = interp(v,p[:3])*eff+p[7]*roll+p[8]-self.y
    self.y += math.copysign(max(abs(push)-interp(v,p[4:7]),0),push)*(-math.expm1(-dt/p[3]))
    self.body += (interp(v,b[:3])*self.y+b[4]*roll+b[5]-self.body)*(-math.expm1(-dt/b[3]))
    return self.y
