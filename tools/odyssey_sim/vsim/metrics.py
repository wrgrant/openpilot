"""Recorded/reprojected wander diagnostics using the existing 20Hz metric.

These describe model perception and requests, not closed-loop vehicle tracking.
Curve/takeover/body metrics cannot be fabricated from open-loop camera replay.
"""
import numpy as np
from tools.odyssey_wander import bandpass, peak_period


def perception_metrics(rows):
  result = {'curve_width_m': None, 'curve_exit_release_s': None, 'takeover_proxy': None,
            'body_roughness': None, 'tracking_error': None, 'follow': None, 'lag_s': None,
            'unavailable_reason': 'requires validated closed-loop pose, controller and body motion'}
  if len(rows) < 900 or np.max(np.diff([r['t'] for r in rows])) > .1:
    result['wander'] = {'available': False, 'reason': 'needs 45 continuous seconds at 20Hz'}
    return result
  time = np.array([r['t'] for r in rows])
  uniform = np.arange(time[0], time[-1], .05)
  speed = np.array([r['speed'] for r in rows])
  stats = {}
  for prefix in ('', 'logged_'):
    offset = -np.array([np.mean(np.array(r[prefix + 'lane'])[:, 0]) for r in rows])
    accel = speed**2 * np.array([r[prefix + 'curvature'] for r in rows])
    offset, accel = [np.interp(uniform, time, a) for a in (offset, accel)]
    stats[prefix or 'replayed'] = {'wander_cm': float(np.std(bandpass(offset)) * 100),
                                 'period_s': peak_period(accel), 'desired_accel_rms': float(np.std(bandpass(accel)))}
  result['wander'] = dict(available=True, **stats, note='unmasked perception diagnostic; not autonomous tracking score')
  return result


def closure_metrics(rows, motion):
  """Per-window tracking evidence; held-out and curve coverage are separate gates."""
  from tools.odyssey_sim.vsim.transfer import perceive
  if len(rows) < 900:
    return {'available': False, 'reason': 'less than 45 seconds of continuous scoring'}
  t = np.array([r['t'] for r in rows])
  if np.max(np.diff(t)) > .1:
    return {'available': False, 'reason': 'video discontinuity'}
  grid = np.arange(t[0], t[-1], .05)
  result = {}
  for prefix in ('', 'logged_'):
    offset = perceive(rows, prefix)[0]
    offset = np.interp(grid, t, offset)
    band = bandpass(offset)
    frequencies = np.fft.rfftfreq(len(band), .05)
    keep = (frequencies >= 1/20) & (frequencies <= 1/7)
    power = abs(np.fft.rfft(band))**2
    result[prefix or 'simulated'] = {'wander_rms_m':float(np.std(band)), 'wander_period_s':peak_period(offset),
                                   'frequency_hz':frequencies[keep].tolist(), 'power':power[keep].tolist()}
  a,b=result['simulated'],result['logged_']
  result['relative_rms_error'] = abs(a['wander_rms_m']/b['wander_rms_m'] - 1)
  result['relative_period_error'] = abs(a['wander_period_s']/b['wander_period_s'] - 1)
  result['normalized_spectrum_l1'] = float(np.sum(abs(np.array(a['power'])-b['power']))/np.sum(b['power']))
  result['max_relative_pose_m'] = max(abs(r['dy']) for r in motion) if motion else None
  result['curve_width_m'] = result['curve_exit_release_s'] = None
  result['curve_reason'] = 'requires matched curve event coverage; this score does not assert curve closure'
  result['heldout_closure_pass'] = False
  return result
