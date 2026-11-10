"""Timing-first wheel-to-yaw identification; stop before downstream failed links."""

import json
from pathlib import Path
import numpy as np
from scipy import signal, optimize
from tools.odyssey_sim.shake.extract import SCRATCH
from tools.odyssey_sim.shake.identify2 import route, pieces, spectra, manifest
from tools.odyssey_sim.shake.timing3 import extract3, vehicle_vectors, band, lag_seconds

OUT = Path(__file__).parent
PLOTS = Path.home() / 'comma-logs/odyssey-evidence/shake_plots'
NAMES = [
  '00000174',
  '0000018c',
  '0000019b',
  '0000019c',
  '000001a3',
  '000001a4',
  '000001a5',
  '000001a7',
  '000001aa',
  '000001ab',
  '000001ac',
  '000001ad',
  '000001ae',
]
HELD = {'0000019c', '000001a5', '000001ad', '000001ae'}


def save(name, obj):
  (OUT / name).write_text(json.dumps(obj, indent=2, allow_nan=False))


def quant(x):
  return np.percentile(x, [2.5, 50, 97.5]).tolist() if len(x) else []


def native_series(n, key):
  a = n[key]
  return a[np.argsort(a[:, 0])]


def calibrated(n, key):
  a = native_series(n, key)
  sources = json.loads(str(n['sources_json']))
  allowed = [v for k, v in sources.items() if k != 'bmx055']
  counts = [(np.sum(a[:, 2] == v), v) for v in allowed]
  source = max(counts)[1]
  a = a[(a[:, 2] == source) & (a[:, 0] > 0) & (abs(a[:, 1] - a[:, 0]) < 0.1)]
  c = native_series(n, 'calib')
  idx = np.maximum(0, np.searchsorted(c[:, 0], a[:, 0], side='right') - 1)
  return a, vehicle_vectors(a[:, 4:7], c[idx, 3:6]), sources


def timing_rows(name, n, d):
  g, gv, sources = calibrated(n, 'gyro')
  a, av, _ = calibrated(n, 'accel')
  stat = np.interp(a[:, 0], d['t'], d['v']) < 0.1
  static = av[stat]
  stationary_gyro = np.interp(g[:, 0], d['t'], d['v']) < 0.1
  pose = native_series(n, 'pose')
  bias = float(np.median(gv[stationary_gyro, 2])) if stationary_gyro.sum() >= 500 else float(np.median(gv[:, 2] - np.interp(g[:, 0], pose[:, 0], pose[:, 4])))
  gv[:, 2] -= bias
  result = {
    'gyro_yaw_bias_rad_s': bias,
    'bias_method': 'stationary' if stationary_gyro.sum() >= 500 else 'route-median raw minus pose DC only',
    'sources': sources,
    'gyro_samples': len(g),
    'accel_samples': len(a),
    'gyro_publication_delay_s': quant(g[:, 1] - g[:, 0]),
    'accel_publication_delay_s': quant(a[:, 1] - a[:, 0]),
    'stationary_samples': len(static),
    'gravity_vehicle_m_s2': np.median(static, axis=0).tolist() if len(static) else [],
    'gravity_norm_m_s2': quant(np.linalg.norm(static, axis=1)),
    'calibration_rpy_median': np.median(n['calib'][:, 3:6], axis=0).tolist(),
  }
  # Resample each eligible continuous window, and reject gaps in native sensor coverage.
  windows = []
  pose_lags, ar_lags, motor_lags, low_agree, curves = [], [], [], [], []
  for w in pieces(d, manual=name in ('000001a4', '000001a5')):
    ts = w['t']
    inside = (g[:, 0] >= ts[0] - 0.05) & (g[:, 0] <= ts[-1] + 0.05)
    gg = g[inside]
    if len(gg) < 1800 or np.max(np.diff(gg[:, 0])) > 0.05:
      continue
    y = np.interp(ts, g[:, 0], gv[:, 2])
    yp = w['yaw'].copy()
    if np.std(band(y)) < 0.0002:
      continue
    lag, corr = lag_seconds(band(y)[100:-100], band(yp)[100:-100])
    if corr > 0.7:
      pose_lags.append([lag, corr])
    slow = signal.sosfiltfilt(signal.butter(3, 0.2, fs=100, output='sos'), np.c_[y, yp], axis=0)[200:-200]
    if np.std(slow[:, 0]) > 0.001:
      low_agree.append([float(np.corrcoef(slow.T)[0, 1]), float(np.polyfit(slow[:, 0], slow[:, 1], 1)[0])])
    # Native angle versus integrated native rate: transport + internal filter, not absolute ECU latency.
    angle = np.interp(ts, n['angle'][:, 0], n['angle'][:, 2])
    rate = np.interp(ts, n['rate'][:, 0], n['rate'][:, 2])
    integrated = np.cumsum(rate) * 0.01
    l, c = lag_seconds(band(integrated)[100:-100], band(angle)[100:-100], limit=0.15)
    if c > 0.7:
      ar_lags.append([l, c])
    cmd = np.interp(ts, n['command'][:, 0], n['command'][:, 2]) / 4096
    motor = np.interp(ts, n['motor'][:, 0], n['motor'][:, 2])
    l, c = lag_seconds(band(abs(cmd))[100:-100], band(motor)[100:-100], limit=0.15)
    if c > 0.7:
      motor_lags.append([l, c])
    w['theta'] = -np.radians(angle)
    # Do not use CAN wheel curvature as independent yaw truth. Fit steady understeer separately.
    for s in (0, 1000):
      sl = slice(s, s + 1000)
      if abs(np.mean(y[sl])) > 0.006 and np.std(y[sl]) < 0.2 * abs(np.mean(y[sl])) and np.std(w['v'][sl]) < 0.5:
        curves.append([float(np.mean(w['v'][sl])), float(np.mean(w['theta'][sl])), float(np.mean(y[sl]))])
    w['yaw'] = y
    f, h, coh = spectra(w)
    windows.append(
      {
        'route': name,
        'speed': w['speed'],
        'v': float(np.mean(w['v'])),
        'f': f,
        'h': h,
        'coh': coh,
        'angle_rms_deg': float(np.std(band(angle))),
        'start': float(ts[0]),
        'yaw_rms': float(np.std(band(y))),
      }
    )
  result.update(
    pose_lag_s=quant([x[0] for x in pose_lags]),
    pose_lag_windows=pose_lags,
    angle_after_integrated_rate_s=quant([x[0] for x in ar_lags]),
    angle_rate_windows=ar_lags,
    motor_after_abs_command_s=quant([x[0] for x in motor_lags]),
    motor_windows=motor_lags,
    low_frequency_pose_vs_raw=low_agree,
    steady_curves=curves,
    windows=len(windows),
  )
  return result, windows


def fit3(rows):
  rng = np.random.default_rng(31109)
  fits = []
  # Relative raw-yaw minus angle timing uncertainty: gyro publication eliminated;
  # CAN arrival/ECU unknown, conservatively allow +/-100 ms explicitly, not a physical inertia.
  bounds = (np.r_[np.log([20000, 20000, 1000, 0.02]), -0.1], np.r_[np.log([600000, 600000, 16000, 6]), 0.1])
  for speed in range(2):
    train = [r for r in rows if r['route'] not in HELD and r['speed'] == speed]

    def pack(group):
      fs, vs, hs = [], [], []
      for r in group:
        m = r['coh'] >= 0.7
        fs.extend(r['f'][m])
        vs.extend(np.full(m.sum(), r['v']))
        hs.extend(r['h'][m])
      return np.array(fs), np.array(vs), np.array(hs)

    def prediction(x, f, v):
      cf, cr, iz, length = np.exp(x[:4])
      m = np.zeros((len(f), 4, 4))
      m[:, 0, 1] = -v
      m[:, 0, 2:] = 1 / 2163.35
      m[:, 1, 2:] = [1.35 / iz, -1.65 / iz]
      m[:, 2, 0] = -cf / length
      m[:, 2, 1] = -cf * 1.35 / length
      m[:, 2, 2] = -v / length
      m[:, 3, 0] = -cr / length
      m[:, 3, 1] = cr * 1.65 / length
      m[:, 3, 3] = -v / length
      rhs = np.zeros((len(f), 4, 1))
      rhs[:, 2, 0] = cf * v / (length * 14.35)
      return np.linalg.solve(2j * np.pi * f[:, None, None] * np.eye(4) - m, rhs)[:, 1, 0] * np.exp(-2j * np.pi * f * x[4])

    data = pack(train)

    def residual(x, data=data):
      f, v, h = data
      z = prediction(x, f, v) / h - 1
      return np.r_[z.real, z.imag]

    de = optimize.differential_evolution(
      lambda x: np.mean(residual(x) ** 2), list(zip(*bounds, strict=True)), seed=3 + speed, popsize=12, maxiter=120, tol=0.001
    )
    best = optimize.least_squares(residual, de.x, bounds=bounds, max_nfev=300)
    boots = []
    routes = sorted({r['route'] for r in train})
    for _ in range(60):
      sample = [r for name in rng.choice(routes, len(routes)) for r in train if r['route'] == name]
      d = pack(sample)
      b = optimize.least_squares(lambda x, d=d: residual(x, d), best.x, bounds=bounds, max_nfev=100)
      boots.append(np.r_[np.exp(b.x[:4]), b.x[4]])
    tests = []
    for r in rows:
      if r['speed'] != speed:
        continue
      z = prediction(best.x, r['f'], np.full(len(r['f']), r['v'])) / r['h']
      for f, q, c in zip(r['f'], z, r['coh'], strict=True):
        tests.append(
          {
            'route': r['route'],
            'held': r['route'] in HELD,
            'v_m_s': r['v'],
            'angle_rms_deg': r['angle_rms_deg'],
            'start': r['start'],
            'frequency': float(f),
            'coherence': float(c),
            'gain': float(abs(q)),
            'phase_deg': float(np.angle(q, deg=True)),
            'covered': bool(c >= 0.7),
            'passed': bool(c >= 0.7 and 0.85 <= abs(q) <= 1.15 and abs(np.angle(q, deg=True)) <= 15),
          }
        )
    p = np.r_[np.exp(best.x[:4]), best.x[4]]
    fit = {
      'speed': speed,
      'parameters': p.tolist(),
      'interval95': np.percentile(boots, [2.5, 97.5], axis=0).tolist(),
      'training_routes': routes,
      'training_windows': len(train),
      'cost': float(np.mean(residual(best.x) ** 2)),
      'test': tests,
      'understeer_s2_m': float(2163.35 / 3 * (1.65 / p[0] - 1.35 / p[1])),
    }
    bp = np.array(boots)
    fit['understeer_interval95_s2_m'] = np.percentile(2163.35 / 3 * (1.65 / bp[:, 0] - 1.35 / bp[:, 1]), [2.5, 97.5]).tolist()
    fits.append(fit)
    held = [t for t in tests if t['held'] and t['covered']]
    print('FIT', speed, p, 'PASS', sum(t['passed'] for t in held), '/', len(held), flush=True)
  return fits


def main():
  save('manifest_start3.json', manifest())
  rows, timing = [], {}
  for name in NAMES:
    n = extract3(name)
    d = route(name)
    tr, ws = timing_rows(name, n, d)
    timing[name] = tr
    rows.extend(ws)
    print('TIMING', name, tr['pose_lag_s'], 'windows', len(ws), flush=True)
  save('timing3.json', timing)
  # Cache complex spectra without lossy serialization for independent follow-up.
  np.savez_compressed(SCRATCH / 'spectra3.npz', rows=np.array(rows, dtype=object))
  fits = fit3(rows)
  save('yaw3.json', {'names': ['Cf_N_rad', 'Cr_N_rad', 'Iz_kgm2', 'relaxation_m', 'relative_delay_s'], 'fits': fits})
  save('manifest_end3.json', manifest())
  print('DONE', flush=True)


if __name__ == '__main__':
  main()
