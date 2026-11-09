"""Offline variants of the real native adapter; selected per isolated process."""

from dataclasses import replace
from pathlib import Path
import os
from tools.odyssey_sim.vsim.closed_loop import recorded

ARMS = ('stock', 'v0', 'kp_half', 'kp_double', 'V1', 'S3', 'S5', 'friction', 'bad_delay')


def candidate(meta, cfg):
  arm = os.environ.get('VSIM_ARM', 'stock')
  if arm not in ARMS:
    raise ValueError(arm)
  c = recorded(meta, replace(cfg, nnlc=arm not in ('v0', 'kp_half', 'kp_double')))
  if arm in ('kp_half', 'kp_double'):
    bp, values = c.lac.pid._k_p
    c.lac.pid._k_p = (bp, [v * (0.5 if arm == 'kp_half' else 2) for v in values])
  if arm in ('V1', 'S3', 'S5'):
    from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel
    from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NnlcResponseShaper

    path = Path.home() / 'comma-logs/odyssey-nnlc-models' / f'HONDA_ODYSSEY_WILLIAM_{arm}.json'
    stock = Path('openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json').resolve()
    c.lac.extension.model = SafeNNTorqueModel(str(path), str(stock))
    c.lac.extension.response_shaper = NnlcResponseShaper.load(str(path))
  update = c.update
  if arm == 'friction':
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_wander import FrictionAssist

    assist = FrictionAssist()

    def update_assist(target, actual, state, plan=None):
      command = update(target, actual, state, plan)
      return assist.update(command, target=target, actual=actual, speed=state.v_ego, rate=0.0, dt=0.01, enabled=True)

    c.update = update_assist
  if arm == 'bad_delay':
    from collections import deque

    queue = deque([0.0] * 30)

    def update_delay(target, actual, state, plan=None):
      queue.append(update(target, actual, state, plan))
      return queue.popleft()

    c.update = update_delay
  return c
