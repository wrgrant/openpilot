"""Real-controller logged-plan evaluation with output-based perception scoring."""

import json
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib.evaluate import records, pieces
from tools.odyssey_sim.calib.identify import S, TEST
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.calib.controller import Controller
from tools.odyssey_sim.closedloop.controller import Config, model_from_array
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_sim.wander.adapter import AssistedController
from tools.odyssey_sim.perception.metric import perceptible_roughness


def main():
  rows = []
  for route, d in records():
    if route[:8] not in TEST:
      continue
    meta = metadata(route)
    cfg = Config(nnlc=meta['params'].get('NeuralNetworkLateralControl', b'0') == b'1')
    selections = list(pieces(d)) or list(pieces(d, 30))
    for j, r in enumerate(selections):
      if np.mean(d['vEgo'][r]) < 22:
        continue
      for mode in ['stock', 'inverse', 'pulse', 'dither']:
        c = Controller(meta, cfg) if mode == 'stock' else AssistedController(meta, cfg, mode)
        plant = Plant()
        plant.reset(d['actual'][r[0]], d['body'][r[0]], -d['steerCommand'][r[0]])
        output = []
        for i in r:
          state = NS(
            v_ego=d['vEgo'][i],
            a_ego=d['aEgo'][i],
            roll=d['roll'][i],
            offset=d['angleOffset'][i],
            ratio=d['steerRatio'][i],
            stiffness=d['stiffness'][i],
            limited=False,
          )
          for key in ['delay', 'factor', 'friction', 'torque_offset', 'pitch']:
            setattr(state, key, float(d[key][i]))
          model = model_from_array(d['plans'][i])
          for _ in range(10):
            u = c.update(d['des'][i], plant.y, state, model)
            plant.step(u, state.v_ego, state.roll)
            angle = np.degrees(c.vm.get_steer_from_curvature(-plant.y / state.v_ego**2, state.v_ego, state.roll))
            output.append([d['des'][i], plant.body, u, angle])
        a = np.array(output)[1000:]
        rough = perceptible_roughness(dt=0.01, angle_deg=a[:, 3], angle_rate_deg_s=np.gradient(a[:, 3], 0.01), lat_accel=a[:, 1], command=a[:, 2])
        rows.append(
          {
            'route': route,
            'window': j,
            'mode': mode,
            'seconds': len(a) * 0.01,
            'tracking_mse': float(np.mean((a[:, 0] - a[:, 1]) ** 2)),
            'roughness': rough['roughness'],
            'hand_channel': False,
            'ranking_authorized': False,
          }
        )
        print(route, j, mode, rows[-1]['roughness'], flush=True)
  (S / 'wander/logged_plan_perception.json').write_text(json.dumps(rows, indent=2))


if __name__ == '__main__':
  main()
