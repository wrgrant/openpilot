"""NnlcAB switches NNLC per block, resetting the PID and its limits only when the mode changes."""
from types import SimpleNamespace as NS

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import AB_BLOCK_S


class TestNnlcAB(OpenpilotTestCase):
  def test_switches_and_resets_only_on_change(self):
    calls = []
    fake = NS(enabled=True, _pid=NS(reset=lambda: calls.append("reset")),
              lac_torque=NS(update_limits=lambda: calls.append("lac_limits")), update_limits=lambda: calls.append("nn_limits"))
    NeuralNetworkLateralControl.update_ab(fake, 0.5 * AB_BLOCK_S)  # first block: on, already on
    assert fake.enabled and calls == []
    NeuralNetworkLateralControl.update_ab(fake, 1.5 * AB_BLOCK_S)  # second block: off
    assert not fake.enabled and calls == ["reset", "lac_limits", "nn_limits"]
    NeuralNetworkLateralControl.update_ab(fake, 1.7 * AB_BLOCK_S)  # same block: nothing
    assert len(calls) == 3
    NeuralNetworkLateralControl.update_ab(fake, 2.5 * AB_BLOCK_S)  # back on
    assert fake.enabled and len(calls) == 6
