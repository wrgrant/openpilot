"""Full native controller update timing; host-only, no device access."""

import json
import os
import time
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib.evaluate import records
from tools.odyssey_sim.closedloop.controller import Config, model_from_array
from tools.odyssey_sim.nnlc_smooth.experiment import runtime_factory, OUT
from tools.odyssey_sim.replay.harness import metadata


def main():
  route, d = next(records())
  state = NS(
    v_ego=25.0,
    a_ego=0.0,
    roll=0.01,
    offset=0.0,
    ratio=15.0,
    stiffness=1.0,
    delay=0.32,
    factor=1.47,
    friction=0.145,
    torque_offset=-0.055,
    pitch=0.0,
    limited=False,
  )
  model = model_from_array(d['plans'][100])
  rows = []
  for name in ('V1', 'S3', 'S4', 'S5'):
    os.environ['NNLC_SMOOTH_MODEL'] = f'HONDA_ODYSSEY_WILLIAM_{name}.json'
    c = runtime_factory(metadata(route), Config(nnlc=True))
    times = []
    for i in range(2200):
      start = time.perf_counter_ns()
      value = c.update(2.0, 1.0, state, model)
      elapsed = (time.perf_counter_ns() - start) / 1e6
      assert np.isfinite(value) and abs(value) <= 1
      if i >= 200:
        times.append(elapsed)
    rows.append({'model': name, 'median_ms': float(np.median(times)), 'p95_ms': float(np.percentile(times, 95)), 'max_ms': float(max(times))})
  (OUT / 'benchmark.json').write_text(json.dumps(rows, indent=2))
  print(json.dumps(rows, indent=2))


if __name__ == '__main__':
  main()
