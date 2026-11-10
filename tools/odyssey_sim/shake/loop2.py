"""Free-running noisy plant, production controllers, gates and full factorial.

Seeds are paired across factors. Lane factor toggles the measured fast component
of the reconstructed historical lane correction, leaving its slow part fixed.
"""
import itertools
import json
import hashlib
from types import SimpleNamespace as NS
from pathlib import Path
import numpy as np
from scipy import signal
from numba import njit
from tools.odyssey_sim.shake.identify2 import save, manifest
from tools.odyssey_sim.shake.analyze import OUT, windows, band, metrics
from tools.odyssey_sim.shake.extract import SCRATCH
from tools.odyssey_sim.shake.physics import MASS, RATIO, A, B
from tools.odyssey_sim.shake.closed import Controller, Config, model_from_array, metadata, SafeNNTorqueModel, NnlcResponseShaper, stats
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.tools.lib.logreader import LogReader


@njit(cache=True)
def step(command, v, rack, state, yp, cp, ep, kind, d, vs, measured_motor=np.nan):
  theta, omega, vy, yaw, ff, fr, motor, applied, z = state
  cf, cr, iz, length = yp
  inertia, viscous, fc, trail, offset = cp
  dead, scale, cap, tau, fade = ep
  for _ in range(10):
    applied += min(.003, max(-.003, command-applied))
    target = np.sign(applied)*min(cap, scale*max(abs(applied)-dead, 0))*min(1., max(.15, v/fade))*.05
    motor = measured_motor if np.isfinite(measured_motor) else motor+(-np.expm1(-.001/tau))*(target-motor)
    if kind == 0:
      friction = fc*np.tanh(omega/.002)
    else:
      g = d*(1.+(np.exp(-(omega/vs)**2) if kind == 2 else 0.))
      zdot = omega-abs(omega)*z/g
      z = z*np.exp(-.001*abs(omega)/g)+np.sign(omega)*g*(-np.expm1(-.001*abs(omega)/g))
      friction = fc*(z/d+(.02*zdot/d if kind == 2 else 0.))
    omega += .001*(motor+rack-viscous*omega-friction-trail*ff/RATIO-offset)/inertia
    theta += .001*omega
    ff += (cf*(theta/RATIO-(vy+A*yaw)/v)-ff)*(-np.expm1(-.001*v/length))
    fr += (cr*(-(vy-B*yaw)/v)-fr)*(-np.expm1(-.001*v/length))
    vy += .001*((ff+fr)/MASS-v*yaw)
    yaw += .001*(A*ff-B*fr)/iz
  return np.array([theta, omega, vy, yaw, ff, fr, motor, applied, z])


def plan_cache(name):
  dest = SCRATCH/f'{name}_plan2.npz'
  if dest.exists():
    return dict(np.load(dest))
  route, = Path.home().joinpath('comma-logs').glob(f'04b*_{name}--*')
  rows = []
  v = 0.
  for f in sorted(route.glob('*/rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    for m in LogReader(str(f)):
      if m.which() == 'carState':
        v = m.carState.vEgo
      elif m.which() == 'modelV2':
        md = m.modelV2
        raw = 0.
        if len(md.laneLines) == 4 and min(md.laneLineProbs[1], md.laneLineProbs[2]) > .5:
          x = np.asarray(md.laneLines[1].x)
          y = (np.asarray(md.laneLines[1].y)+np.asarray(md.laneLines[2].y))/2
          keep = x <= 25
          c = np.polynomial.polynomial.polyfit(x[keep], y[keep], 3)
          off, head = -float(c[0]), -np.arctan(c[1])
          if abs(off) <= 1.5 and abs(head) <= .1:
            raw = np.clip(-.3*off-1.2*v*head, -.15, .15)
        rows.append([m.logMonoTime*1e-9, md.action.desiredCurvature, raw])
    print('plan', name, f.parent.name, flush=True)
  rows = np.array(rows)
  result = {'t': rows[:, 0], 'curvature': rows[:, 1], 'lane': rows[:, 2]}
  np.savez_compressed(dest, **result)
  return result


class Campaign:
  def __init__(self):
    self.yaw = [np.array(f['parameters']) for f in json.loads((OUT/'yaw2.json').read_text())['fits']]
    self.ep = np.array(json.loads((OUT/'eps2.json').read_text())['parameters'])
    col = json.loads((OUT/'column2.json').read_text())
    self.col, self.noise = col['selected'], col['noise']
    self.cp = np.array(self.col['parameters'])
    self.data, self.meta, self.plans = {}, {}, {}
    for name in ('000001ac', '000001ad', '000001ae'):
      self.data[name] = dict(np.load(SCRATCH/f'{name}.npz'))
      self.meta[name] = metadata(name)
      self.plans[name] = plan_cache(name)

  def run(self, w, model=None, gain=None, noise=True, disturbance=True, lane='historical', seed=0):
    model = w['model'] if model is None else model
    c = Controller(self.meta[w['name']], Config(nnlc=True))
    ext = c.lac.extension
    stock = Path.cwd()/'openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json'
    model_dir = Path.cwd()/'openpilot/sunnypilot/selfdrive/controls/lib/nnlc/odyssey_models'
    path = stock if model == 0 else model_dir/f'HONDA_ODYSSEY_WILLIAM_S{3 if model == 1 else 5}.json'
    ext.model = SafeNNTorqueModel(path, stock)
    ext.response_shaper = NnlcResponseShaper.load(path)
    if gain is not None:
      if ext.response_shaper is None:
        ext.response_shaper = NnlcResponseShaper()
      ext.response_shaper.error_gain = gain
    ext.model_ab = None
    c.lac.reset()
    state = np.zeros(9)
    state[0], state[3] = w['truth'][0, 0], w['yaw'][0]
    d = self.data[w['name']]
    ix = np.clip(np.searchsorted(d['t'], w['t']), 0, len(d['t'])-1)
    plan = self.plans[w['name']]
    rawlane = np.interp(w['t'], plan['t'], plan['lane'])
    smoothlane = signal.lfilter([.01/.51], [1, -.5/.51], rawlane)
    if lane == 'historical':
      desired = w['des_curv']
    else:
      # Preserve historical clipped/delayed base plan; replace only lane component.
      historical = rawlane if w['lane'] else np.zeros(len(rawlane))
      add = rawlane if lane == 'on' else smoothlane if lane == 'slow' else np.zeros(len(rawlane))
      desired = w['des_curv']+(add-historical)/w['v']**2
    rng = np.random.default_rng(seed)
    n = self.noise[w['speed']]
    sensors = rng.normal(size=(len(ix), 4))*np.array(n['sensor_std']) if noise else np.zeros((len(ix), 4))
    rack = signal.lfilter([n['innovation_std']], [1, -n['rack_rho']], rng.normal(size=len(ix))) if disturbance else np.zeros(len(ix))
    us, angles, bodies = [], [], []
    for i, k in enumerate(ix):
      v = max(5., float(w['v'][i]+sensors[i, 3]))
      vehicle, tuning = d['vehicle'][k], d['tuning'][k]
      lp = NS(roll=float(vehicle[0]), angleOffsetDeg=float(vehicle[1]))
      c.vm.update_params(max(float(vehicle[3]), .1), max(float(vehicle[2]), .1))
      # Log angles include sensor zero; plant theta is absolute sensor angle in pass 1 convention.
      angle = round((-np.degrees(state[0])+sensors[i, 0])*10)/10
      cs = NS(vEgo=v, aEgo=0., steeringAngleDeg=angle, steeringRateDeg=-np.degrees(state[1])+sensors[i, 1],
              yawRate=state[3]+sensors[i, 2], steeringPressed=False, steeringRateLimited=False, leftBlinker=False, rightBlinker=False)
      ext.update_model_v2(model_from_array(d['plans'][k].reshape(3, -1)))
      delay = float(d['delay'][k, 0])
      ext.update_lateral_lag(delay)
      ext.update_limits()
      c.lac.update_torque_parameters(*map(float, tuning))
      pose = NS(orientation=NS(pitch=float(d['orientation'][k, 1])))
      limited = bool(us and abs(state[7]-us[-1]) > .01)
      torque, _, _ = c.lac.update(True, cs, c.vm, lp, limited, float(desired[i]), pose, False, delay+LAT_SMOOTH_SECONDS)
      torque = c.odyssey.apply(float(torque), True, cs, c.vm, lp, float(desired[i]), 1.)
      command = -float(torque)
      state = step(command, float(w['v'][i]), rack[i], state, self.yaw[w['speed']], self.cp, self.ep,
                   self.col['kind'], self.col['d'], self.col['vs'])
      us.append(command)
      angles.append(np.degrees(state[0]))
      bodies.append(w['v'][i]*state[3])
    return stats(np.array(us), np.array(angles), np.array(bodies))

  def open_loop(self, w):
    state = np.zeros(9)
    state[0], state[3] = w['truth'][0, 0], w['yaw'][0]
    y = []
    for i, u in enumerate(w['u']):
      state = step(u, w['v'][i], 0., state, self.yaw[w['speed']], self.cp, self.ep,
                   self.col['kind'], self.col['d'], self.col['vs'])
      y.append([state[0], w['v'][i]*state[3]])
    return metrics(w, np.array(y))


def main():
  start = manifest()
  save('loop_manifest_start2.json', start)
  campaign = Campaign()
  allw = [w for name in ('000001ac', '000001ad', '000001ae') for w in windows(name)]
  openrows, closedrows = [], []
  for w in allw:
    ms = campaign.open_loop(w)
    row = {k: w[k] for k in ('name', 'model', 'speed', 'lane')}
    row['start'] = float(w['t'][0])
    openrows.append(dict(**row, metrics=ms))
    road = stats(w['u'], -w['angle'], w['truth'][:, 1])
    sims = [campaign.run(w, seed=seed) for seed in range(3)]
    sim = {k: float(np.median([s[k] for s in sims])) for k in road}
    ratios = {k: sim[k]/road[k] if road[k] else None for k in road if k != 'peak_per_second'}
    closedrows.append(dict(**row, road=road, sim=sim, seed_stats=sims, ratios=ratios,
                           peak_error=abs(sim['peak_per_second']-road['peak_per_second'])))
    print('GATE', row, ratios, flush=True)
  save('gate12.json', {'rows': openrows, 'coherence_threshold': .7})
  save('gate22.json', {'rows': closedrows, 'seeds': 3})
  # Paired 32-cell factorial on two high-speed development windows, not one
  # different logged plan per factor. Replicate each cell with three noise seeds.
  chosen = [w for w in allw if w['name'] == '000001ac' and w['speed'] == 1]
  chosen = [chosen[0], max(chosen, key=lambda w: np.std(band(w['truth'])[:, 1]))]
  factorial = []
  for model, gain, noise, disturbance, lane in itertools.product((0, 2), (1, 2), (False, True), (False, True), (False, True)):
    for wi, w in enumerate(chosen):
      for seed in range(3):
        s = campaign.run(w, model=model, gain=gain, noise=noise, disturbance=disturbance, lane='on' if lane else 'slow', seed=seed)
        factorial.append({'model': model, 'gain': gain, 'noise': noise, 'disturbance': disturbance, 'lane': lane, 'window': wi, 'seed': seed, 'stats': s})
    save('factorial2.json', {'rows': factorial})
    print('FACTORIAL', model, gain, noise, disturbance, lane, flush=True)
  forecasts = []
  for lane in ('off', 'slow'):
    runs = [campaign.run(w, model=0, lane=lane, seed=seed) for w in allw if w['name'] in ('000001ac', '000001ad') and w['speed'] == 1 for seed in range(5)]
    forecasts.append({'controller': 'stock_NNLC', 'smoothed_lane_correction': lane == 'slow', 'windows_seeds': len(runs),
                     'metrics': {k: {'median': float(np.median([r[k] for r in runs])), 'interval95': np.percentile([r[k] for r in runs], [2.5, 97.5]).tolist()}
                              for k in runs[0] if k != 'peak_per_second'}})
  save('forecast_locked2.json', {'target': 'next highway drive after creation; >45 mph; 20s eligible windows; 5s warmup/1s trim',
       'locked_before_target_seen': True, 'validated': False,
       'interval_kind': 'conditional empirical 95% window/seed range; excludes parameter/model uncertainty',
       'forecasts': forecasts})
  save('loop_manifest_end2.json', manifest())
  save('forecast_lock2.json', {'sha256': hashlib.sha256((OUT/'forecast_locked2.json').read_bytes()).hexdigest()})


if __name__ == '__main__':
  main()
