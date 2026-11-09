"""Recorded/reprojected wander diagnostics using the existing 20Hz metric.

These describe model perception and requests, not closed-loop vehicle tracking.
Curve/takeover/body metrics cannot be fabricated from open-loop camera replay.
"""

import numpy as np
from tools.odyssey_wander import bandpass, peak_period


def perception_metrics(rows):
  result = {
    'curve_width_m': None,
    'curve_exit_release_s': None,
    'takeover_proxy': None,
    'body_roughness': None,
    'tracking_error': None,
    'follow': None,
    'lag_s': None,
    'unavailable_reason': 'requires validated closed-loop pose, controller and body motion',
  }
  if len(rows) < 900 or np.max(np.diff([r['t'] for r in rows])) > 0.1:
    result['wander'] = {'available': False, 'reason': 'needs 45 continuous seconds at 20Hz'}
    return result
  time = np.array([r['t'] for r in rows])
  uniform = np.arange(time[0], time[-1], 0.05)
  speed = np.array([r['speed'] for r in rows])
  stats = {}
  for prefix in ('', 'logged_'):
    offset = -np.array([np.mean(np.array(r[prefix + 'lane'])[:, 0]) for r in rows])
    accel = speed**2 * np.array([r[prefix + 'curvature'] for r in rows])
    offset, accel = [np.interp(uniform, time, a) for a in (offset, accel)]
    stats[prefix or 'replayed'] = {
      'wander_cm': float(np.std(bandpass(offset)) * 100),
      'period_s': peak_period(accel),
      'desired_accel_rms': float(np.std(bandpass(accel))),
    }
  result['wander'] = dict(available=True, **stats, note='unmasked perception diagnostic; not autonomous tracking score')
  return result


def closure_metrics(rows, motion):
  """Per-window tracking evidence; held-out and curve coverage are separate gates."""
  from tools.odyssey_sim.vsim.transfer import perceive

  if len(rows) < 900:
    return {'available': False, 'reason': 'less than 45 seconds of continuous scoring'}
  t = np.array([r['t'] for r in rows])
  if np.max(np.diff(t)) > 0.1:
    return {'available': False, 'reason': 'video discontinuity'}
  grid = np.arange(t[0], t[-1], 0.05)
  result = {}
  for prefix in ('', 'logged_'):
    offset = perceive(rows, prefix)[0]
    offset = np.interp(grid, t, offset)
    band = bandpass(offset)
    frequencies = np.fft.rfftfreq(len(band), 0.05)
    keep = (frequencies >= 1 / 20) & (frequencies <= 1 / 7)
    power = abs(np.fft.rfft(band)) ** 2
    result[prefix or 'simulated'] = {
      'wander_rms_m': float(np.std(band)),
      'wander_period_s': peak_period(offset),
      'frequency_hz': frequencies[keep].tolist(),
      'power': power[keep].tolist(),
    }
  a, b = result['simulated'], result['logged_']
  result['relative_rms_error'] = abs(a['wander_rms_m'] / b['wander_rms_m'] - 1)
  result['relative_period_error'] = abs(a['wander_period_s'] / b['wander_period_s'] - 1)
  result['normalized_spectrum_l1'] = float(np.sum(abs(np.array(a['power']) - b['power'])) / np.sum(b['power']))
  result['max_relative_pose_m'] = max(abs(r['dy']) for r in motion) if motion else None
  result['curve_width_m'] = result['curve_exit_release_s'] = None
  result['curve_reason'] = 'requires matched curve event coverage; this score does not assert curve closure'
  result['heldout_closure_pass'] = False
  return result


def curve_scores(rows, motion, native, origin, confidence=None):
  if len(rows) < 3 or len(motion) < 2:
    return {
      'available': False,
      'reason': 'insufficient video/body samples',
      'events': [],
      'simulated_body_jerk_rms_mps3': None,
      'logged_body_jerk_rms_mps3': None,
    }
  from tools.odyssey_sim.vsim.transfer import perceive
  from tools.odyssey_sim.curves.characterize import events

  confidence = (
    np.array([r['logged_lane_prob'][1] > 0.5 and r['logged_lane_prob'][2] > 0.5 for r in rows])
    if confidence is None and all('logged_lane_prob' in r for r in rows)
    else confidence
  )
  confidence = np.ones(len(rows), bool) if confidence is None else np.asarray(confidence, bool)
  t = np.array([r['t'] for r in rows])
  grid = np.arange(t[0], t[-1], 0.1)
  mt = np.array([r['t'] - origin for r in motion])
  nt = native['t'] - origin
  v = np.interp(grid, t, [r['speed'] for r in rows])
  logged_des = np.interp(grid, nt, native['des'])
  logged_off = perceive(rows, 'logged_', 25)[0]
  sim_off = perceive(rows, '', 25)[0]
  _, _, road = perceive(rows, 'logged_', 15)
  # Event boundaries come exclusively from the logged plan for pairing.
  base = {
    't': grid,
    'ok': np.ones(len(grid), bool),
    'des': logged_des,
    'act': np.interp(grid, nt, native['act']),
    'body': np.interp(grid, nt, native['body']),
    'v': v,
    'cmd': np.interp(grid, nt, native['cmd']),
    'geometry_road': np.interp(grid, t, road) * v * v,
  }
  logged_events = events(base)
  result = []
  for ev in logged_events:
    mask = (grid >= ev['start_t']) & (grid <= ev['end_t'])
    sign = np.sign(np.mean(logged_des[mask]))
    middle = (grid >= ev['start_t'] + 0.25 * (ev['end_t'] - ev['start_t'])) & (grid <= ev['end_t'] - 0.25 * (ev['end_t'] - ev['start_t']))
    middle_confidence = np.interp(grid[middle], t, confidence.astype(float)) > 0.5
    width_available = np.mean(middle_confidence) > 0.8
    row = {
      'width_available': bool(width_available),
      'lane_confident_fraction': float(np.mean(middle_confidence)),
      'start_t': ev['start_t'],
      'end_t': ev['end_t'],
      'logged': ev,
      'unmasked_logged_run_wide_m': float(np.median(-sign * np.interp(grid[middle], t, logged_off))),
      'logged_run_wide_m': float(np.median(-sign * np.interp(grid[middle][middle_confidence], t, logged_off))) if width_available else None,
      'unmasked_simulated_run_wide_m': float(np.median(-sign * np.interp(grid[middle], t, sim_off))),
      'simulated_run_wide_m': float(np.median(-sign * np.interp(grid[middle][middle_confidence], t, sim_off))) if width_available else None,
    }
    # Same logged threshold and sign; compare actual body release to logged plan.
    peak = np.argmax(np.where(mask, sign * logged_des, -np.inf))
    amp = sign * logged_des[peak]
    after = np.arange(peak, min(np.searchsorted(grid, ev['end_t'] + 6), len(grid)))
    body = np.interp(grid, mt, [r['body'] for r in motion])
    release = after[sign * body[after] <= 0.2 * amp]
    plan_exit = ev['des']['0.2'][1]
    row['simulated_exit_body_minus_plan_s'] = None if not len(release) or plan_exit is None else float(grid[release[0]] - plan_exit)
    result.append(row)
  # Body jerk is defined on 20Hz body acceleration, without derivative noise from variable dt.
  g20 = np.arange(t[0], t[-1], 0.05)
  return {
    'events': result,
    'simulated_body_jerk_rms_mps3': float(np.sqrt(np.mean(np.gradient(np.interp(g20, mt, [r['body'] for r in motion]), 0.05) ** 2))),
    'logged_body_jerk_rms_mps3': float(np.sqrt(np.mean(np.gradient(np.interp(g20, nt, native['body']), 0.05) ** 2))),
  }


def logged_lane_mask(rows, key, origin):
  """Fixed logged visibility mask keeps candidate-dependent selection out of scores."""
  if all('logged_lane_prob' in r for r in rows):
    return np.array([r['logged_lane_prob'][1] > 0.5 and r['logged_lane_prob'][2] > 0.5 for r in rows])
  from tools.odyssey_sim.calib.identify import S

  root = S if (S / 'calib3' / f'{key}.npz').exists() else S / 'vsim3/cache'
  with np.load(root / 'calib3' / f'{key}.npz') as data:
    t, probabilities = data['t'], data['probs']
  at = np.array([r['t'] + origin for r in rows])
  right = np.clip(np.searchsorted(t, at), 0, len(t) - 1)
  left = np.maximum(right - 1, 0)
  ix = np.where(abs(t[right] - at) < abs(t[left] - at), right, left)
  return (probabilities[ix, 1] > 0.5) & (probabilities[ix, 2] > 0.5) & (abs(t[ix] - at) < 0.01)


def lane_scores(rows, motion, confidence=None):
  from tools.odyssey_sim.vsim.transfer import perceive

  t = np.array([r['t'] for r in rows])
  grid = np.arange(t[0], t[-1], 0.05)
  confidence = np.ones(len(rows), bool) if confidence is None else np.asarray(confidence, bool)
  output = {}
  for prefix in ('', 'logged_'):
    offset, _, _ = perceive(rows, prefix, horizon=25)
    near = -np.array([np.mean(np.array(r[prefix + 'lane'])[:, 0]) for r in rows])
    mask = confidence & np.isfinite(near)
    available = mask.mean() > 0.8 and len(rows) >= 900 and np.max(np.diff(t)) <= 0.1
    unmasked = np.interp(grid, t, near)
    supported = np.interp(grid, t[mask], near[mask]) if available else None
    bp = bandpass(supported) if available else None
    frequencies = np.fft.rfftfreq(len(grid), 0.05)
    keep = (frequencies >= 1 / 20) & (frequencies <= 1 / 7)
    output[prefix or 'simulated'] = {
      'wander_available': bool(available),
      'lane_confident_fraction': float(mask.mean()),
      'wander_cm': float(np.std(bp) * 100) if available else None,
      'unmasked_wander_cm': float(np.std(bandpass(unmasked)) * 100),
      'near_fit_wander_cm': float(np.std(bandpass(np.interp(grid, t, offset))) * 100),
      'max_offset_m': float(np.max(abs(near[mask]))) if mask.any() else None,
      'unmasked_max_offset_m': float(np.max(abs(near))),
      'period_s': peak_period(supported) if available else None,
      'frequencies_hz': frequencies[keep].tolist() if available else [],
      'power_m2': (abs(np.fft.rfft(bp)[keep]) ** 2 / len(grid) ** 2).tolist() if available else [],
      'reason': None if available else 'requires 900 continuous 20Hz frames and both logged lane probabilities >.5 in >80% of the window',
    }
  output['max_pose_m'] = float(max(abs(r['dy']) for r in motion))
  output['rms_pose_m'] = float(np.sqrt(np.mean(np.array([r['dy'] for r in motion]) ** 2)))
  return output
