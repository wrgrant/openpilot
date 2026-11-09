import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.replay import ControllerConfig, replay_configs


class TestOdysseyReplay(OpenpilotTestCase):
  def test_logged_agreement_and_distinct_controller_modes(self):
    rows = list(replay_configs('000001aa', [ControllerConfig(True), ControllerConfig(False)], max_frames=25000))
    x = np.array([[r['logged'], r['nnlc_v0_logged'], r['stock_v0_logged']]
                  for r in rows if r['active'] and not r['pressed']])
    self.assertGreater(len(x), 1000)
    self.assertGreater(np.corrcoef(x[:, 0], x[:, 1])[0, 1], .999)
    self.assertLess(np.sqrt(np.mean((x[:, 0]-x[:, 1])**2)), .002)
    self.assertGreater(np.max(abs(x[:, 1]-x[:, 2])), .1)
