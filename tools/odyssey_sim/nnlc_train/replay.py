"""Bounded logged-motion V0 replay, reusing worker 1's validated timing adapter."""

import json
from dataclasses import dataclass
from unittest.mock import patch
import numpy as np
from tools.odyssey_sim.replay import harness
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel
from tools.odyssey_sim.nnlc_train.closedloop import S, MODEL_DIR


@dataclass(frozen=True)
class Config(harness.ControllerConfig):
  model: str = 'stock'

  @property
  def name(self):
    return self.model


def main():
  params = Params()
  params.remove('NnlcModelOverride')
  # Preserve worker 1's adapter; only replace the constructed model for one arm.
  configs = [Config(model=m) for m in ('stock', 'v1')]
  rows = []
  for route in ('000001aa--ba236ee054', '000001ab--9191c0d55d'):
    pending = iter(configs)
    original = harness.LatControlTorqueV0

    def create(*args, original=original, pending=pending, **kwargs):
      controller = original(*args, **kwargs)
      cfg = next(pending)
      if cfg.model == 'v1':
        controller.extension.model = SafeNNTorqueModel(MODEL_DIR / 'HONDA_ODYSSEY_WILLIAM_V1.json', MODEL_DIR / 'HONDA_ODYSSEY.json')
      return controller

    put, put_bool = Params.put, Params.put_bool
    with (
      patch.object(harness, 'LatControlTorqueV0', create),
      patch.object(Params, 'put', lambda self, key, value, put=put, **kw: put(self, key, value, block=True)),
      patch.object(Params, 'put_bool', lambda self, key, value, put_bool=put_bool, **kw: put_bool(self, key, value, block=True)),
    ):
      samples = list(harness.replay_configs(route, configs, max_frames=20000))
    for lo, hi in ((0, 10), (10, 20), (20, 35), (35, 50), (50, 999)):
      for cfg in configs:
        selected = [r for r in samples if r['active'] and not r['pressed'] and lo <= r['speed'] * 2.23694 < hi]
        if not selected:
          continue
        u = np.array([r[cfg.name] for r in selected])
        logged = np.array([r['logged'] for r in selected])
        t = np.array([r['t'] for r in selected])
        adjacent = np.diff(t) < 0.025
        seconds = float(np.sum(np.diff(t)[adjacent]))
        rows.append(
          {
            'route': route,
            'model': cfg.name,
            'band': f'{lo}-{hi}',
            'n': len(u),
            'command_mae': float(np.mean(abs(u - logged))),
            'flips_s': float(np.sum((u[:-1] * u[1:] < 0) & (abs(np.diff(u)) > 0.01) & adjacent) / max(seconds, 0.01)),
          }
        )
    print(route, 'DONE', flush=True)
  (S / 'nnlc_replay.json').write_text(json.dumps(rows, indent=2))


if __name__ == '__main__':
  main()
