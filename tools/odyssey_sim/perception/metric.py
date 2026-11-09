"""Offline, output-based Odyssey smoothness screening; see README for calibration limits.

Stable API (keyword arguments; NumPy only):
  perceptible_roughness(dt=seconds, angle_deg=array,
    angle_rate_deg_s=None, lat_accel=None, hand_torque_nm=None, command=None,
    valid=None, thresholds=Thresholds(), window_s=10., hop_s=5.) -> dict
  from_simulator(states, dt=seconds, command=None, **kwargs) -> same dict
  sluggishness(dt=seconds, actual=array, reference=array, threshold=units,
    max_lag_s=4.) -> dict
  Thresholds: frozen dataclass with explicit units; see README.
Primary keys: roughness, p95_window, scored_windows, coverage_channels, windows.
Empty eligible data returns None, never a good score of zero. The 0.1-0.5 Hz
band is diagnostic only; the score uses 0.5-10 Hz. Compare identical coverage,
thresholds, speed, road, and valid windows. Thresholds are provisional, not human
certification. No driver sensor counts-to-Nm conversion is assumed.

Only NumPy is required. Commands never contribute to the roughness score. Missing
hand torque is unknown, not zero. All thresholds are configurable hypotheses.
"""
from dataclasses import dataclass

import numpy as np

BANDS_HZ = ((0.1, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 5.0), (5.0, 10.0))


@dataclass(frozen=True)
class Thresholds:
  angle_deg: float = 0.1
  angle_weber: float = 0.14
  hand_nm: float = 0.1
  hand_weber: float = 0.15
  rim_accel_mps2: float = 0.2
  wheel_radius_m: float = 0.19
  body_accel_mps2: float = 0.05
  body_jerk_mps3: float = 0.3


def _array(x, n, name):
  a = np.asarray(x, dtype=float)
  if a.shape != (n,) or not np.all(np.isfinite(a)):
    raise ValueError(f'{name} must contain {n} finite samples')
  return a


def _spectrum(x, dt):
  """One-sided Hann periodogram, density per Hz, with linear trend removed."""
  n = len(x)
  z = np.arange(n) - (n - 1) / 2
  x = x - np.mean(x)
  x = x - z * np.dot(z, x) / np.dot(z, z)
  w = np.hanning(n)
  p = abs(np.fft.rfft(x * w)) ** 2 * dt / np.sum(w * w)
  p[1:-1] *= 2
  return np.fft.rfftfreq(n, dt), p


def perceptible_roughness(*, dt, angle_deg, angle_rate_deg_s=None, lat_accel=None,
                         hand_torque_nm=None, command=None, valid=None,
                         thresholds=None, window_s=10.0, hop_s=5.0):
  """Score continuous uniformly sampled outputs; units are degrees, SI, N·m.

  valid is an eligibility mask; windows crossing excluded samples are skipped.
  Return dimensionless excess energy, channel coverage and per-window/band data.
  Use angle_rate_deg_s when available: its PSD / (2*pi*f)^2 gives displacement
  without differentiating 0.1-degree CAN angle steps. At least one full window
  is required. This is a screening index, NOT a detection probability.
  """
  th = thresholds or Thresholds()
  if not np.isfinite(dt) or dt <= 0 or window_s < 2 or hop_s <= 0:
    raise ValueError('positive dt/hop_s and window_s >= 2 required')
  if any(not np.isfinite(v) or v <= 0 for v in vars(th).values()):
    raise ValueError('thresholds must be finite and positive')
  angle = np.asarray(angle_deg, dtype=float)
  if angle.ndim != 1:
    raise ValueError('angle_deg must be one dimensional')
  n = len(angle)
  angle = _array(angle, n, 'angle_deg')
  inputs = {'wheel': angle}
  for name, value in [('rate', angle_rate_deg_s), ('body', lat_accel), ('hand', hand_torque_nm), ('command', command)]:
    if value is not None:
      inputs[name] = _array(value, n, name)
  mask = np.ones(n, dtype=bool) if valid is None else np.asarray(valid, dtype=bool)
  if mask.shape != (n,):
    raise ValueError('valid must match angle')
  size, hop = round(window_s / dt), max(1, round(hop_s / dt))
  if size < 4:
    raise ValueError('window needs at least four samples')
  rows = []
  for start in range(0, n - size + 1, hop):
    sl = slice(start, start + size)
    if not mask[sl].all():
      continue
    spectra = {k: _spectrum(v[sl], dt) for k, v in inputs.items()}
    f = spectra['wheel'][0]
    omega = 2 * np.pi * f
    df = 1 / (size * dt)
    angle_floor = max(th.angle_deg, th.angle_weber * abs(float(np.mean(angle[sl]))))
    wheel_threshold = np.full_like(f, angle_floor)
    # Rotational tactile proxy above 3 Hz: tangential rim acceleration.
    high = f >= 3
    wheel_threshold[high] = np.minimum(wheel_threshold[high],
      np.degrees(th.rim_accel_mps2 / (th.wheel_radius_m * omega[high] ** 2)))
    wp = spectra['wheel'][1]
    if 'rate' in spectra:
      wp = spectra['rate'][1] / np.maximum(omega ** 2, 1e-20)
    ps = {'wheel': (wp, wheel_threshold)}
    if 'body' in spectra:
      ps['body'] = (spectra['body'][1], np.maximum(th.body_accel_mps2,
        th.body_jerk_mps3 / np.maximum(omega, 1e-10)))
    if 'hand' in spectra:
      floor = max(th.hand_nm, th.hand_weber * abs(float(np.mean(inputs['hand'][sl]))))
      ps['hand'] = (spectra['hand'][1], np.full_like(f, floor))
    bands = []
    for lo, hi in BANDS_HZ:
      if hi > 0.5 / dt:
        continue  # no silent partial-band comparisons
      k = (f >= lo) & (f < hi)
      channels = {}
      for name, (p, floor) in ps.items():
        ratio2 = float(np.sum(p[k] / floor[k] ** 2) * df)
        channels[name] = {'rms': float(np.sqrt(np.sum(p[k]) * df)),
                          'threshold_ratio': float(np.sqrt(ratio2)),
                          'excess_energy': max(0.0, ratio2 - 1.0)}
      bands.append({'hz': [lo, hi], 'channels': channels})
    # Avoid counting the same physical motion as independent wheel/body cues.
    channel_energy = {name: sum(b['channels'][name]['excess_energy'] for b in bands if b['hz'][0] >= 0.5) for name in ps}
    row = {'start_s': start * dt, 'energy': max(channel_energy.values(), default=0.0),
           'channel_energy': channel_energy, 'bands': bands}
    if command is not None:
      c = inputs['command'][sl]
      signs = np.sign(c[c != 0])
      row['command_reversals_hz'] = float(np.sum(signs[1:] * signs[:-1] < 0) / (size * dt))
    rows.append(row)
  energy = np.array([r['energy'] for r in rows])
  return {'version': 'odyssey-perception-v2', 'roughness': float(np.sqrt(np.mean(energy))) if len(energy) else None,
          'p95_window': float(np.sqrt(np.percentile(energy, 95))) if len(energy) else None,
          'scored_windows': len(rows), 'coverage_channels': [k for k in ('wheel', 'body', 'hand') if k in inputs],
          'thresholds': vars(th), 'bands_hz': [list(b) for b in BANDS_HZ if b[1] <= 0.5 / dt], 'windows': rows}


def sluggishness(*, dt, actual, reference, threshold, max_lag_s=4.0):
  """Reference-conditioned lag and supra-threshold mismatch, separate from roughness.

  actual/reference must share units and signs (angle OR lateral acceleration).
  Includes planner delay if reference is road demand; not if it is model demand.
  This is a global lag diagnostic; callers select individual exit/event windows.
  """
  a = np.asarray(actual, dtype=float)
  a = _array(a, len(a), 'actual')
  r = _array(reference, len(a), 'reference')
  if not all(np.isfinite(x) for x in (dt, threshold, max_lag_s)) or dt <= 0 or threshold <= 0 or max_lag_s < 0 or len(a) < 4:
    raise ValueError('positive dt/threshold, nonnegative max_lag, >=4 samples required')
  rms = float(np.std(r))
  if rms < threshold:
    return {'lag_s': None, 'reason': 'insufficient reference excitation'}
  max_shift = min(round(max_lag_s / dt), len(a) // 3)
  errors = [np.mean((a[k:] - r[:len(a) - k]) ** 2) for k in range(max_shift + 1)]
  excess = np.maximum(abs(a - r) - threshold, 0)
  return {'lag_s': int(np.argmin(errors)) * dt,
          'excess_error_rms': float(np.sqrt(np.mean(excess ** 2))),
          'excess_error_integral': float(np.sum(excess) * dt)}


def from_simulator(states, *, dt, command=None, **kwargs):
  """Plant dictionaries: steering_accel is m/s² proxy, NEVER deg/s²."""
  states = list(states)
  return perceptible_roughness(dt=dt, angle_deg=[s['angle_deg'] for s in states],
    angle_rate_deg_s=[s['angle_rate_deg_s'] for s in states],
    lat_accel=[s['lat_accel'] for s in states], command=command, **kwargs)
