"""Mac timing, with an explicit 5-10x CPU slowdown estimate for the 3X."""

import json
import time
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.nnlc_train.closedloop import S, MODEL_DIR, VanController
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_sim.closedloop.controller import model_from_array
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel


def main():
  results = {}
  meta = metadata('000001aa--ba236ee054')
  state = NS(v_ego=25.0, a_ego=0.0, roll=0.01, offset=0.2, ratio=17.8, stiffness=1.0)
  plan = model_from_array(np.zeros((3, 33)))
  for name in ('stock', 'v1', 'v2'):
    c = VanController(meta, name)
    if name == 'v1':
      c.lac.extension.model = SafeNNTorqueModel(MODEL_DIR / 'HONDA_ODYSSEY_WILLIAM_V1.json', MODEL_DIR / 'HONDA_ODYSSEY.json')
    for _ in range(100):
      c.update(0.5, 0.3, state, plan)
    elapsed = []
    for _ in range(2000):
      start = time.perf_counter_ns()
      c.update(2.1, 0.3, state, plan)
      elapsed.append((time.perf_counter_ns() - start) / 1e6)
    results[name] = {
      'mac_median_ms': float(np.median(elapsed)),
      'mac_p95_ms': float(np.percentile(elapsed, 95)),
      'estimated_3x_median_ms_5to10x': [float(np.median(elapsed) * s) for s in (5, 10)],
    }
  (S / 'nnlc_benchmark.json').write_text(json.dumps(results, indent=2))
  print(results)


if __name__ == '__main__':
  main()
