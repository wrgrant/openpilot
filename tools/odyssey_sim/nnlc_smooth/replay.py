"""Validated V0 logged-motion replay; motion is fixed, commands are counterfactual."""

import json
import os
from dataclasses import dataclass
from unittest.mock import patch
import numpy as np
from tools.odyssey_sim.replay import harness
from tools.odyssey_sim.nnlc_smooth.experiment import OUT
from tools.odyssey_sim.nnlc_train import MODEL_DIR
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NnlcResponseShaper


@dataclass(frozen=True)
class Config(harness.ControllerConfig):
  model: str = 'stock'

  @property
  def name(self):
    return self.model


def main():
  os.environ['OPENPILOT_PREFIX'] = 'simnnlcsmoothreplay'
  params = Params()
  params.remove('NnlcModelOverride')
  configs = [Config(model=x) for x in ('stock', 'V1', 'S3', 'S4', 'S5', 'V2')]
  rows = []
  for route in ('000001aa--ba236ee054', '000001ab--9191c0d55d'):
    pending = iter(configs)
    original = harness.LatControlTorqueV0

    def create(*args, original=original, pending=pending, **kwargs):
      c = original(*args, **kwargs)
      cfg = next(pending)
      if cfg.model == 'V2':
        from tools.odyssey_sim.nnlc_smooth.experiment import install, Trial

        install(c.extension, Trial(model='v2'))
      elif cfg.model != 'stock':
        path = MODEL_DIR / f'HONDA_ODYSSEY_WILLIAM_{cfg.model}.json'
        c.extension.model = SafeNNTorqueModel(path, MODEL_DIR / 'HONDA_ODYSSEY.json')
        c.extension.response_shaper = NnlcResponseShaper.load(path)
      return c

    put, put_bool = Params.put, Params.put_bool
    with (
      patch.object(harness, 'LatControlTorqueV0', create),
      patch.object(Params, 'put', lambda self, key, value, put=put, **kw: put(self, key, value, block=True)),
      patch.object(Params, 'put_bool', lambda self, key, value, put_bool=put_bool, **kw: put_bool(self, key, value, block=True)),
    ):
      samples = list(harness.replay_configs(route, configs, max_frames=20000))
    for lo, hi in ((0, 10), (10, 20), (20, 35), (35, 50), (50, 200)):
      selected = [r for r in samples if r['active'] and not r['pressed'] and lo <= r['speed'] * 2.23694 < hi]
      if len(selected) < 2:
        continue
      t = np.array([r['t'] for r in selected])
      logged = np.array([r['logged'] for r in selected])
      adjacent = np.diff(t) < 0.025
      seconds = np.sum(np.diff(t)[adjacent])
      for cfg in configs:
        u = np.array([r[cfg.name] for r in selected])
        rows.append(
          {
            'route': route,
            'band': f'{lo}-{hi}',
            'model': cfg.model,
            'n': len(u),
            'command_mae': float(np.mean(abs(u - logged))),
            'r': float(np.corrcoef(u, logged)[0, 1]),
            'flips': float(np.sum((u[:-1] * u[1:] < 0) & (abs(np.diff(u)) > 0.01) & adjacent) / max(seconds, 0.01)),
            'rate_rms': float(np.sqrt(np.mean((np.diff(u)[adjacent] / np.diff(t)[adjacent]) ** 2))),
          }
        )
    (OUT / 'replay.json').write_text(json.dumps(rows, indent=2))
    print(route, 'done', flush=True)


if __name__ == '__main__':
  main()
