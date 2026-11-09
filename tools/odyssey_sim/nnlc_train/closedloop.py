"""Evaluate candidates with worker 3's plant/rollout, using the van's V0 controller.

Candidate V2 is deliberately offline until it clears closed-loop comparison.
"""

import json
import os
from collections import deque
import numpy as np
from tools.odyssey_sim.closedloop.controller import Controller, Config
from tools.odyssey_sim.closedloop.rollout import windows, metrics, run_physical
from tools.odyssey_sim.closedloop.worker_plant import FittedPlant, load_params
from tools.odyssey_sim.replay.harness import metadata
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque
from opendbc.car.car_helpers import interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.model import NNTorqueModel
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel

from tools.odyssey_sim.nnlc_train import SCRATCH as S, MODEL_DIR


class CandidateModel:
  """State-aware mapping tested with the same context in setpoint/measurement calls."""

  def __init__(self, ext):
    self.ext = ext
    self.base = ext.model
    self.model = NNTorqueModel(MODEL_DIR / 'HONDA_ODYSSEY_WILLIAM_V2.json')
    self.history = deque(maxlen=51)
    self.extra = None
    self.friction_override = self.model.friction_override

  def update(self, cs):
    self.history.append((self.ext._actual_lateral_accel, self.ext._output_torque))
    self.extra = [cs.steeringAngleDeg, cs.steeringRateDeg, self.ext._actual_lateral_accel]
    self.extra += [self.history[max(0, len(self.history) - 1 - i)][0] for i in (10, 30, 50)]
    self.extra += [self.history[max(0, len(self.history) - 1 - i)][1] for i in (30, 50)]

  def evaluate(self, x):
    if self.extra is None or len(self.history) < 51:
      return self.base.evaluate(x)
    return float(np.clip(self.model.evaluate(x + [0.0] * (18 - len(x)) + self.extra), -1, 1))


class VanController(Controller):
  def __init__(self, meta, name):
    super().__init__(meta, Config(nnlc=True))
    p = Params()
    p.remove('NnlcModelOverride')
    cp = meta['carParams'].as_reader()
    sp = meta['carParamsSP'].as_reader()
    ci = interfaces[cp.carFingerprint](cp, sp)
    self.lac = LatControlTorque(cp, sp, ci, 0.01)
    self.lac.update_torque_parameters(1.47, -0.055, 0.145)
    ext = self.lac.extension
    if name == 'v1':
      ext.model = SafeNNTorqueModel(MODEL_DIR / 'HONDA_ODYSSEY_WILLIAM_V1.json', MODEL_DIR / 'HONDA_ODYSSEY.json')
    if name == 'v2':
      candidate = CandidateModel(ext)
      original = ext.update_neural_network_feedforward

      def update(cs, params, pose):
        candidate.update(cs)
        return original(cs, params, pose)

      ext.model = candidate
      ext.update_neural_network_feedforward = update


def main():
  os.environ['OPENPILOT_PREFIX'] = 'simnnlctrain'
  FittedPlant.params = load_params()
  rows = []
  for route in ('000001aa--ba236ee054', '000001ab--9191c0d55d'):
    d = dict(np.load(S / 'closedloop_data' / f'{route}.npz'))
    meta = metadata(route)
    for band, w in windows(d):
      for name in ('stock', 'v1', 'v2'):
        c = VanController(meta, name)
        pred, u = run_physical(w, c, FittedPlant())
        m = metrics(w['targetLateralAcceleration'][100:], pred[100:], u[100:], mask=w['score_mask'][100:])
        native = c.native_commands[1000:]
        mask = np.repeat(w['score_mask'][100:], 10)
        adjacent = mask[1:] & mask[:-1]
        m['native_flips_s'] = float(np.sum((native[1:] * native[:-1] < 0) & (abs(native[1:] - native[:-1]) > 0.01) & adjacent) / (sum(mask) * 0.01))
        m['native_command_rate_rms'] = float(np.sqrt(np.mean((np.diff(native)[adjacent] / 0.01) ** 2)))
        m.update(route=route, band=band, model=name, baseline='V0_NNLC', plant='worker2_fitted')
        rows.append(m)
        print(json.dumps(m), flush=True)
        (S / 'nnlc_closedloop.json').write_text(json.dumps(rows, indent=2))


if __name__ == '__main__':
  main()
