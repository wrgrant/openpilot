"""Apply return primitive to validated native harness output (no motion feedback).

Preview reports an altered target, not a regenerated native LaC command. Full
LaC preview command replay is in experiment.py at calib's 10 Hz motion cadence.
"""

import json
import math
import numpy as np
from opendbc.car.vehicle_model import VehicleModel
from tools.odyssey_sim.calib.identify import S, align
from tools.odyssey_sim.calib.evaluate import records
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_sim.curves.transition import CurveTransitions
from tools.odyssey_sim.curves.characterize import summary
from openpilot.selfdrive.modeld.constants import ModelConstants


def main():
  out = []
  events = json.loads((S / 'curves' / 'characterization.json').read_text())['events']
  for route, w in records():
    key = route[:8]
    if key not in ['000001a7', '000001aa', '000001ab']:
      continue
    d = dict(np.load(S / f'nnlc_{key[-3:]}.npz'))
    t = d['t']
    raw = np.load(S / 'plant_data' / f'{key}.npz')
    vp = np.load(S / 'calib' / f'{key}_body.npz')['vp']
    vm = VehicleModel(metadata(route)['carParams'].as_reader())
    ix = np.searchsorted(w['t'], t, side='right') - 1
    angle = align(t, np.c_[raw['t'], raw['angle']], 1, 0.05)
    rate = align(t, np.c_[raw['t'], raw['rate']], 1, 0.05)
    drv = align(t, np.c_[raw['t'], raw['drv']], 1, 0.05)
    params = np.array([align(t, vp, k, 0.5) for k in range(1, 5)]).T
    # Visit event-local slices including 3 s context. Never carry state over gaps.
    for j, event in enumerate(events[key]['rows']):
      sel = np.flatnonzero((t >= event['start_t'] - 3) & (t <= event['end_t'] + 5))
      if len(sel) < 100:
        continue
      sign = np.sign(w['des'][np.argmin(abs(w['t'] - event['start_t']))])
      for mode in ['off', 'release', 'active']:
        c = CurveTransitions(enabled=True, preview=True, return_mode=mode)
        times = []
        targets = []
        commands = []
        base = []
        for i in sel:
          z = max(0, ix[i])
          roll, offset, ratio, stiff = params[i]
          if not np.isfinite(params[i]).all():
            c.reset()
            continue
          vm.update_params(max(stiff, 0.1), max(ratio, 0.1))
          speed = float(d['speed'][i])
          target = float(d['desired'][i])
          planned = math.degrees(vm.get_steer_from_curvature(-target / max(speed * speed, 0.1), speed, roll))
          u = -float(d['nnlc_v0_logged'][i])
          target, u2 = c.update(
            command=u,
            target_accel=target,
            speed=speed,
            angle_deg=angle[i] - offset,
            rate_deg_s=rate[i],
            planned_angle_deg=planned,
            plan_t=ModelConstants.T_IDXS,
            plan_accel=w['plans'][z, 0],
            active=bool(d['active'][i]),
            driver_pressed=bool(d['pressed'][i]),
            driver_torque=drv[i],
            plan_age=float(t[i] - w['t'][z]),
            dt=0.01,
          )
          times.append(t[i])
          targets.append(target)
          commands.append(u2)
          base.append(u)
        times = np.array(times)
        targets = np.array(targets)
        commands = np.array(commands)
        base = np.array(base)

        def first(a, entry, times=times, event=event, sign=sign):
          mask = (times <= event['start_t'] + 2) if entry else (times >= event['end_t'] - 1)
          ids = np.flatnonzero(mask & ((sign * a > 0.8 * event['peak']) if entry else (sign * a < 0.2 * event['peak'])))
          return float(times[ids[0]]) if len(ids) and ids[0] > 0 else None

        original = np.interp(times, t, d['desired'])
        en0, en1 = first(original, True), first(targets, True)
        ex0, ex1 = first(original, False), first(targets, False)
        out.append(
          {
            'route': key,
            'event': j,
            'mode': mode,
            'n': len(times),
            'entry_target_lead_s': None if en0 is None or en1 is None else en0 - en1,
            'exit_target_lead_s': None if ex0 is None or ex1 is None else ex0 - ex1,
            'command_changed_pct': float(100 * np.mean(abs(commands - base) > 0.01)),
            'command_delta_peak': float(np.max(abs(commands - base))),
          }
        )
    print(key, flush=True)
  (S / 'curves' / 'native_replay.json').write_text(json.dumps(out, indent=2))
  for mode in ['off', 'release', 'active']:
    rr = [r for r in out if r['mode'] == mode]
    print(
      mode,
      {k: summary([r[k] for r in rr if r[k] is not None]) for k in ['entry_target_lead_s', 'exit_target_lead_s', 'command_changed_pct', 'command_delta_peak']},
    )


if __name__ == '__main__':
  main()
