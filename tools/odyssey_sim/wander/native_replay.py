"""20 Hz model-origin replay, calibrated geometry and 100 Hz body/torque channels.

Candidate commands are counterfactual intentions, not observed outcomes.
"""

import json
import numpy as np
from tools.odyssey_sim.calib.identify import S, align
from tools.odyssey_sim.wander.runtime import LaneCentering, FrictionAssist
from tools.odyssey_sim.wander.research import ROUTES


def main():
  out = []
  selected = json.loads((S / 'calib3/closure.json').read_text())['selection']['variant']
  for route in ROUTES:
    d = np.load(S / 'calib3' / f'{route}.npz')
    b = np.load(S / 'calib' / f'{route}_body.npz')
    t = d['t']
    n = d['native']
    g = d['geometry'][:, selected]
    v = align(t, n, 1, 0.15)
    active = align(t, n, 7, 0.15)
    pressed = align(t, n, 5, 0.15)
    blinker = align(t, n, 6, 0.15)
    des = align(t, n, 8, 0.15)
    actual = align(t, n, 9, 0.15)
    command = -align(t, n, 10, 0.15)
    rate = align(t, n, 4, 0.15)
    yaw = align(t, b['cc'], 3, 0.15)
    ok = (v > 22) & (active > 0.5) & (pressed == 0) & (blinker == 0) & (d['probs'][:, 1] > 0.6) & (d['probs'][:, 2] > 0.6)
    # Existing cache records nonzero maneuver injection but not full mode state.
    # Exclude injected samples; shadow metrics do not model all runtime gates.
    cached = np.load(S / 'calib' / f'{route}_aligned.npz')
    probe = align(t, np.c_[cached['t'], cached['probe']], 1, 0.15)
    ok &= np.isfinite(probe) & (abs(probe) < 0.001)
    ok &= np.isfinite(np.c_[g, v, yaw, des, actual, command, rate]).all(axis=1)
    for heading in [False, True]:
      lane = LaneCentering()
      assist = FrictionAssist()
      data = []
      previous = False
      for i in range(1, len(t)):
        if not ok[i] or not ok[i - 1] or not 0.01 < t[i] - t[i - 1] < 0.12:
          lane.reset()
          assist.reset()
          previous = False
          continue
        dt = t[i] - t[i - 1]
        a = lane.update(
          dt=dt,
          speed=v[i],
          yaw=yaw[i],
          road_curvature=g[i, 2],
          camera_offset=g[i, 0],
          camera_heading=g[i, 1] if heading else None,
          camera_new=True,
          valid=True,
          enabled=True,
        )
        u = assist.update(command[i], target=des[i] + a, actual=actual[i], speed=v[i], rate=rate[i], dt=dt, enabled=True)
        # Smooth offset over 0.5 s backward only; no future camera frames.
        lo = max(0, i - 10)
        e = float(np.mean(g[lo : i + 1, 0]))
        ep = float(np.mean(g[max(0, lo - 10) : max(1, lo), 0]))
        outward = previous and 0.02 < abs(e) < 0.08 and e * (e - ep) > 0 and abs(e - ep) > 0.0025
        data.append([t[i], g[i, 0], a, u - command[i], outward, a * e > 0, des[i], actual[i]])
        previous = True
      a = np.array(data)
      m = a[:, 4] > 0.5
      out.append(
        {
          'route': route,
          'heading_used': heading,
          'eligible_minutes': len(a) * 0.05 / 60,
          'early_samples': int(m.sum()),
          'early_direction_pct': float(100 * np.mean(a[m, 5])) if m.any() else None,
          'addon_rms': float(np.sqrt(np.mean(a[:, 2] ** 2))),
          'friction_add_rms': float(np.sqrt(np.mean(a[:, 3] ** 2))),
          'addon_clipped_pct': float(100 * np.mean(abs(a[:, 2]) >= 0.1199)),
        }
      )
      np.savez_compressed(S / 'wander' / f'native_replay_{route}_{int(heading)}.npz', trace=a)
  (S / 'wander/native_replay.json').write_text(json.dumps(out, indent=2))
  print(json.dumps(out, indent=2))


if __name__ == '__main__':
  main()
