"""Return-to-straight rejection grid on plant v2, plus adversarial input checks."""

import json
import numpy as np
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.calib.identify import S
from tools.odyssey_sim.curves.transition import CurveTransitions


def main():
  rows = []
  for v in [5.0, 10.0, 20.0, 30.0, 35.0]:
    for sign in [-1, 1]:
      for mode in ['off', 'release', 'active']:
        p = Plant()
        p.reset(sign * 0.8)
        c = CurveTransitions(enabled=True, return_mode=mode)
        yy = []
        uu = []
        for _ in range(1000):
          # Approximate bicycle wheel conversion: used only for the return grid.
          factor = -np.degrees(17.8 * 2.9 / v**2)
          angle = p.y * factor
          before = p.y
          rate = 0.0 if not yy else (p.y - yy[-1]) * factor / 0.01
          # Plain proportional feedback plus van-like steady hold during decay.
          command = float(np.clip(-0.5 * p.y, -1, 1))
          _, u = c.update(
            command=command,
            target_accel=0.0,
            speed=v,
            angle_deg=angle,
            rate_deg_s=rate,
            planned_angle_deg=0.0,
            plan_t=[0.0, 1.0, 3.0],
            plan_accel=[0.0, 0.0, 0.0],
            active=True,
            driver_pressed=False,
            driver_torque=0.0,
            plan_age=0.0,
          )
          uu.append(u)
          yy.append(before)
          p.step(u, v, 0.0)
        y = np.array(yy)
        ix = np.flatnonzero(abs(y) < 0.08)
        rows.append(
          {
            'speed': v,
            'sign': sign,
            'mode': mode,
            'settle_10pct_s': float(ix[0] * 0.01) if len(ix) else None,
            'overshoot_accel': float(max(0.0, np.max(-sign * y))),
            'final_accel': float(y[-1]),
            'peak_command': float(np.max(np.abs(uu))),
          }
        )
  (S / 'curves' / 'stress.json').write_text(json.dumps(rows, indent=2))
  print('rows', len(rows), 'max overshoot', max(r['overshoot_accel'] for r in rows))


if __name__ == '__main__':
  main()
