"""Staged offline identification. No runtime writes; all quantities curvature-positive.

Route-held-out spectral tyre fit, measured CAN EPS fit, inverse column comparison.
Intervals bootstrap route/window clusters; they remain conditional on the model.
"""
import hashlib
import json
import subprocess
from pathlib import Path
import numpy as np
from scipy import signal, optimize
from numba import njit
from tools.odyssey_sim.shake.extract import SCRATCH, DATA
from tools.odyssey_sim.shake.physics import MASS, RATIO, A, B
from tools.odyssey_sim.shake.analyze import OUT

EVIDENCE = Path.home()/'comma-logs/odyssey-evidence'


def save(name, obj):
  (OUT/name).write_text(json.dumps(obj, indent=2, allow_nan=False))


def manifest():
  files = list(Path('openpilot/sunnypilot/selfdrive/controls').rglob('*.py'))
  files += [Path('openpilot/selfdrive/controls/controlsd.py')]
  return {'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
          'branch': subprocess.check_output(['git', 'branch', '--show-current'], text=True).strip(),
          'status': subprocess.check_output(['git', 'status', '--short'], text=True),
          'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}


def route(name):
  if (DATA/f'{name}.npz').exists():
    p = SCRATCH/f'{name}.npz'
    d = dict(np.load(p if p.exists() else DATA/f'{name}.npz'))
    d['active'] = d['lat']
    older = EVIDENCE/'plant_data'/f'{name}.npz'
    if 'motor' not in d and older.exists():
      old = np.load(older)
      d['motor'] = np.interp(d['t'], old['t'], old['motor'])[:, None]
      d['can_cmd'] = (np.interp(d['t'], old['t'], old['cmd'])*4096)[:, None]
    d['cmd'] = d['can_cmd'][:, 0]/4096 if 'can_cmd' in d else -d['torque']
    if 'motor' in d:
      d['motor'] = d['motor'][:, 0]
  else:
    d = dict(np.load(EVIDENCE/'plant_data'/f'{name}.npz'))
    cc = np.load(EVIDENCE/'calib'/f'{name}_body.npz')['cc']
    d['yaw'] = np.interp(d['t'], cc[:, 0], cc[:, 3])
  return d


def pieces(d, manual=False, min_speed=25*.44704):
  t = d['t']
  ok = (d['v'] > min_speed) & (d['blinker'] == 0)
  if not manual:
    ok &= d['active'] > .5
  speed = (d['v'] > 45*.44704).astype(int)
  edges = np.r_[0, np.flatnonzero((np.diff(t) <= 0) | (np.diff(t) > .04) | (np.diff(d['seg']) != 0) |
              (np.diff(ok.astype(int)) != 0) | (np.diff(speed) != 0))+1, len(t)]
  for s, e in zip(edges[:-1], edges[1:], strict=True):
    if not ok[s]:
      continue
    for start in np.arange(t[s], t[e-1]-20, 20):
      ts = start+np.arange(2000)*.01
      w = {k: np.interp(ts, t, d[k]) for k in ('angle', 'rate', 'yaw', 'v', 'cmd', 'drv')}
      if 'motor' in d:
        w['motor'] = np.interp(ts, t, d['motor'])
      w.update(t=ts, speed=int(speed[s]), theta=-np.radians(w['angle']))
      yield w


def spectra(w):
  kw = {'fs': 100, 'nperseg': 500, 'noverlap': 250}
  f, px = signal.welch(w['theta'], **kw)
  _, xy = signal.csd(w['theta'], w['yaw'], **kw)
  _, coh = signal.coherence(w['theta'], w['yaw'], **kw)
  mask = (f >= .3) & (f <= 3)
  return f[mask], xy[mask]/np.maximum(px[mask], 1e-20), coh[mask]


def yaw_tf(p, v, f):
  cf, cr, iz, length = p
  mat = np.array([[-(cf+cr)/(MASS*v), (-A*cf+B*cr)/(MASS*v)-v],
                  [(-A*cf+B*cr)/(iz*v), -(A*A*cf+B*B*cr)/(iz*v)]])
  # Full four-state tyre force relaxation, not an extra scalar yaw delay.
  m = np.zeros((4, 4))
  m[0, 1] = -v
  m[0, 2:] = 1/MASS
  m[1, 2:] = [A/iz, -B/iz]
  m[2] = [-cf/length, -cf*A/length, -v/length, 0]
  m[3] = [-cr/length, cr*B/length, 0, -v/length]
  rhs = np.array([0, 0, cf*v/(length*RATIO), 0])
  del mat
  return np.linalg.solve(2j*np.pi*f[:, None, None]*np.eye(4)-m, np.broadcast_to(rhs, (len(f), 4))[..., None])[..., 1, 0]


def yaw_fit():
  rng = np.random.default_rng(20261109)
  names = sorted(p.stem for p in (EVIDENCE/'plant_data').glob('*.npz'))
  names += ['000001ac', '000001ad', '000001ae']
  held = {'000001a5', '0000019c', '000001ad'}
  rows = []
  for name in names:
    for w in pieces(route(name), manual=name in ('000001a4', '000001a5')):
      f, h, coh = spectra(w)
      rows.append({'route': name, 'speed': w['speed'], 'v': float(np.mean(w['v'])), 'f': f, 'h': h, 'coh': coh})
  bounds = np.log([[20000, 20000, 1000, .02], [600000, 600000, 16000, 6]])

  def residual(x, group):
    p = np.exp(x)
    out = []
    for r in group:
      mask = r['coh'] >= .7
      if mask.any():
        z = yaw_tf(p, r['v'], r['f'][mask])/r['h'][mask]-1
        out.extend(np.r_[z.real, z.imag])
    return np.array(out)

  fits = []
  for speed in range(2):
    train = [r for r in rows if r['speed'] == speed and r['route'] not in held and r['route'] != '000001ae']
    def loss(x, train=train):
      return np.mean(residual(x, train)**2)
    de = optimize.differential_evolution(loss, list(zip(*bounds, strict=True)), seed=12+speed, popsize=10, maxiter=90, tol=.002)
    best = optimize.least_squares(lambda x, train=train: residual(x, train), de.x, bounds=bounds)
    # Route-cluster bootstrap preserves within-route dependence.
    routes = sorted({r['route'] for r in train})
    boots = []
    for _ in range(40):
      sample = [r for name in rng.choice(routes, len(routes)) for r in train if r['route'] == name]
      fit = optimize.least_squares(lambda x, sample=sample: residual(x, sample), best.x, bounds=bounds, max_nfev=60)
      boots.append(np.exp(fit.x))
    p = np.exp(best.x)
    tests = []
    for r in rows:
      if r['speed'] != speed or r['route'] not in held | {'000001ae'}:
        continue
      ratio = yaw_tf(p, r['v'], r['f'])/r['h']
      for f, z, c in zip(r['f'], ratio, r['coh'], strict=True):
        tests.append({'route': r['route'], 'frequency': float(f), 'coherence': float(c), 'covered': bool(c >= .7),
                          'gain': float(abs(z)), 'phase': float(np.angle(z, deg=True)),
                          'passed': bool(c >= .7 and .85 <= abs(z) <= 1.15 and abs(np.angle(z, deg=True)) <= 15)})
    fits.append({'speed': speed, 'parameters': p.tolist(), 'interval95': np.percentile(boots, [2.5, 97.5], axis=0).tolist(),
                     'bounds': np.exp(bounds).tolist(), 'cost': float(np.mean(residual(best.x, train)**2)),
                     'training_routes': routes, 'training_windows': len(train), 'test': tests})
    print('YAW', speed, p, 'covered/pass', sum(t['covered'] for t in tests), sum(t['passed'] for t in tests), flush=True)
  save('yaw2.json', {'parameter_names': ['Cf_N_rad', 'Cr_N_rad', 'Iz_kgm2', 'relaxation_m'], 'fits': fits,
                      'heldout_routes': sorted(held), 'check_only': '000001ae', 'route_windows': {n: sum(r['route'] == n for r in rows) for n in names}})
  return fits


@njit(cache=True)
def tyre(theta, v, p):
  cf, cr, iz, length = p
  vy = r = ff = fr = 0.
  out = np.zeros((len(theta), 2))
  for i in range(len(theta)):
    speed = max(v[i], 5.)
    for _ in range(10):
      ff += (cf*(theta[i]/RATIO-(vy+A*r)/speed)-ff)*(-np.expm1(-.001*speed/length))
      fr += (cr*(-(vy-B*r)/speed)-fr)*(-np.expm1(-.001*speed/length))
      vy += .001*((ff+fr)/MASS-speed*r)
      r += .001*(A*ff-B*fr)/iz
    out[i] = ff, r
  return out


@njit(cache=True)
def eps_predict(u, v, p):
  dead, scale, cap, tau, fade = p
  y = np.zeros(len(u))
  for i in range(1, len(u)):
    target = min(cap, scale*max(abs(u[i])-dead, 0.))*min(1., max(.15, v[i]/fade))
    y[i] = y[i-1]+(-np.expm1(-.01/tau))*(target-y[i-1])
  return y


def eps_fit():
  rng = np.random.default_rng(20261109)
  train, test = [], []
  for name in ('000001a7', '000001aa', '000001ab', '000001ac', '000001ad', '000001ae'):
    d = route(name)
    for w in pieces(d, min_speed=1.):
      if 'motor' not in w:
        continue
      # Straight self-presses remain eligible. Reject curves with substantial driver torque.
      valid = (abs(w['drv']) < 300) | ((abs(w['v']*w['yaw']) < .3) & (w['v'] > 11.17))
      rev = np.r_[False, w['cmd'][1:]*w['cmd'][:-1] < 0]
      valid &= signal.convolve(rev.astype(float), np.ones(21), mode='same') < .5
      w['valid'] = valid & (np.arange(2000) > 100)
      (test if name in ('000001ad', '000001ae') else train).append(w)
  bounds = ([.005, 50, 55, .003, 3], [.07, 180, 100, .15, 12])
  def res(p, group):
    return np.concatenate([(eps_predict(w['cmd'], w['v'], p)-w['motor'])[w['valid']][::5] for w in group])
  de = optimize.differential_evolution(lambda p: np.mean(np.minimum(abs(res(p, train)), 20)**2), list(zip(*bounds, strict=True)), seed=5, maxiter=80)
  best = optimize.least_squares(lambda p: res(p, train), de.x, bounds=bounds, loss='soft_l1')
  boots = []
  # Fixed bootstrap samples within each optimizer, never resample inside its residual.
  for _ in range(40):
    sample = [train[i] for i in rng.integers(len(train), size=len(train))]
    boots.append(optimize.least_squares(lambda p, sample=sample: res(p, sample), best.x, bounds=bounds, loss='soft_l1', max_nfev=60).x)
  result = {'parameters': best.x.tolist(), 'names': ['deadband', 'raw_per_command', 'cap_raw', 'tau_s', 'fade_full_m_s'],
                'interval95': np.percentile(boots, [2.5, 97.5], axis=0).tolist(), 'bounds': bounds,
                'train_rmse': float(np.sqrt(np.mean(res(best.x, train)**2))), 'test_rmse': float(np.sqrt(np.mean(res(best.x, test)**2))),
                'train_windows': len(train), 'test_windows': len(test),
                'limitation': ('Low-speed fade confounded by driver assistance; absolute torque scale is unknown; '
                               + 'reversal +/-0.10 s excluded. Signed motor uses sign(CAN command).')}
  save('eps2.json', result)
  print('EPS', result, flush=True)
  return result


@njit(cache=True)
def friction_basis(omega, kind, d, vs):
  z = 0.
  out = np.zeros(len(omega))
  for i, vel in enumerate(omega):
    if kind == 0:
      out[i] = np.tanh(vel/.002)
    else:
      g = d*(1.+(np.exp(-(vel/vs)**2) if kind == 2 else 0.))
      zdot = vel-abs(vel)*z/g
      z = z*np.exp(-.01*abs(vel)/g)+np.sign(vel)*g*(-np.expm1(-.01*abs(vel)/g))
      out[i] = z/d + (.02*zdot/d if kind == 2 else 0.)
  return out


def column_fit(yaw):
  rng = np.random.default_rng(20261109)
  groups = {'train': [], 'test': [], 'check': []}
  for name in ('000001a7', '000001aa', '000001ab', '000001ac', '000001ad', '000001ae'):
    d = route(name)
    for w in pieces(d):
      if 'motor' not in w:
        continue
      w['route'] = name
      w['omega'] = signal.savgol_filter(w['theta'], 31, 3, deriv=1, delta=.01)
      w['alpha'] = signal.savgol_filter(w['theta'], 31, 3, deriv=2, delta=.01)
      w['ff'] = tyre(w['theta'], w['v'], np.array(yaw[w['speed']]['parameters']))[:, 0]/RATIO
      w['signed_motor'] = np.sign(w['cmd'])*w['motor']*.05
      rev = np.r_[False, w['cmd'][1:]*w['cmd'][:-1] < 0]
      w['valid'] = (np.arange(2000) > 500) & (np.arange(2000) < 1900)
      w['valid'] &= (signal.convolve(rev.astype(float), np.ones(21), mode='same') < .5)
      w['valid'] &= (abs(w['drv']) < 400) | (abs(w['v']*w['yaw']) < .3)
      groups['check' if name.endswith('ae') else 'test' if name.endswith('ad') else 'train'].append(w)
  fits = []
  for kind, label in enumerate(('Coulomb', 'Dahl', 'LuGre')):
    def design(w, d, vs, kind=kind):
      return np.c_[w['alpha'], w['omega'], friction_basis(w['omega'], kind, d, vs), w['ff'], np.ones(2000)]
    def fit(group, d, vs):
      x = np.concatenate([design(w, d, vs)[w['valid']][::5] for w in group])
      y = np.concatenate([w['signed_motor'][w['valid']][::5] for w in group])
      result = optimize.lsq_linear(x, y, bounds=([.001, 0, 0, 0, -5], [2, 20, 4, .3, 5]))
      return result.x, float(np.mean((x@result.x-y)**2))
    def obj(x):
      return fit(groups['train'], *np.exp(x))[1]
    de = optimize.differential_evolution(obj, [(-8, -3), (-7, 0)], maxiter=35, popsize=8, seed=kind)
    d, vs = np.exp(de.x)
    p, cost = fit(groups['train'], d, vs)
    boots = []
    for _ in range(40):
      sample = [groups['train'][i] for i in rng.integers(len(groups['train']), size=len(groups['train']))]
      boots.append(fit(sample, d, vs)[0])
    rms = {}
    for group, ws in groups.items():
      err = np.concatenate([(w['signed_motor']-design(w, d, vs)@p)[w['valid']] for w in ws])
      rms[group] = float(np.sqrt(np.mean(err**2)))
    fits.append({'kind': kind, 'label': label, 'parameters': p.tolist(), 'd': float(d), 'vs': float(vs), 'cost': cost,
                     'interval95': np.percentile(boots, [2.5, 97.5], axis=0).tolist(), 'torque_residual_rmse': rms})
  best = min(fits, key=lambda r: r['torque_residual_rmse']['test'])
  noises = []
  for speed in range(2):
    residuals, sensors = [], []
    for w in groups['train']:
      if w['speed'] != speed or (w['route'].endswith('ac') and int(w['t'][0]//180)%3 != 0):
        continue
      x = np.c_[w['alpha'], w['omega'], friction_basis(w['omega'], best['kind'], best['d'], best['vs']), w['ff'], np.ones(2000)]
      # Continuous scored middle, no concatenation across windows for AR coefficient.
      r = (x@np.array(best['parameters'])-w['signed_motor'])[500:1900]
      residuals.append(r-np.mean(r))
      sensors.append([np.std(w[k]-signal.savgol_filter(w[k], 11, 2)) for k in ('angle', 'rate', 'yaw', 'v')])
    rho = sum(np.dot(r[:-1], r[1:]) for r in residuals)/sum(np.dot(r[:-1], r[:-1]) for r in residuals)
    rho = float(np.clip(rho, 0, .999))
    sigma = float(np.std(np.concatenate(residuals)))
    noises.append({'speed': speed, 'rack_rho': rho, 'rack_std_Nm_gauge': sigma, 'innovation_std': sigma*np.sqrt(1-rho*rho),
                       'sensor_std': np.median(sensors, axis=0).tolist(), 'windows': len(residuals)})
  result = {'names': ['J_gauge', 'B_gauge', 'Fc_gauge', 'trail_gauge', 'offset_Nm_gauge'], 'fits': fits, 'selected': best, 'noise': noises,
                'limitation': ('Inverse dynamics; derivatives smoothed 0.31 s. Gauge 0.05 Nm/raw assumed. '
                               + 'Residual includes driver/model/measurement error, not uniquely road torque. '
                               + 'Friction state parameters not bootstrapped.')}
  save('column2.json', result)
  print('COLUMN', result, flush=True)
  return result


def main():
  save('manifest_start2.json', manifest())
  yaw = yaw_fit()
  eps_fit()
  column_fit(yaw)


if __name__ == '__main__':
  main()
