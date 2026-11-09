"""Runtime shaping seam: step response, reset, bounded metadata and stock identity."""

import json
import tempfile
from pathlib import Path
import numpy as np
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NnlcResponseShaper


class TestResponseShaper(OpenpilotTestCase):
  def test_causal_response_and_reset(self):
    s = NnlcResponseShaper(error_gain=1.4, error_tau=0.09, ff_tau=0.09)
    self.assertEqual(s.update(0.0, 0.0), (0.0, 0.0))
    error, ff = s.update(1.0, 1.0)
    self.assertAlmostEqual(error, 0.14)
    self.assertAlmostEqual(ff, 0.1)
    for _ in range(100):
      error, ff = s.update(1.0, 1.0)
    self.assertAlmostEqual(error, 1.4, places=4)
    s.reset()
    self.assertEqual(s.update(0.0, 0.0), (0.0, 0.0))

  def test_metadata_validation_and_absence(self):
    with tempfile.TemporaryDirectory() as tmp:
      p = Path(tmp) / 'model.json'
      p.write_text('{}')
      self.assertIsNone(NnlcResponseShaper.load(p))
      for settings in ({'error_tau': float('nan')}, {'error_gain': 99}, {'ff_tau': -1}, {'typo': 1}):
        p.write_text(json.dumps({'odyssey_smoothing': settings}))
        with self.assertRaises(ValueError):
          NnlcResponseShaper.load(p)

  def test_bad_values_reset_state(self):
    s = NnlcResponseShaper(error_tau=0.1)
    s.update(1.0, 0.0)
    for value in (np.nan, np.inf, -np.inf):
      with self.assertRaises(ValueError):
        s.update(value, 0.0)
      self.assertEqual(s.update(0.0, 0.0), (0.0, 0.0))

  def test_selected_model_reaches_runtime_and_default_is_unchanged(self):
    from openpilot.common.params import Params
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.honda.values import CAR
    from openpilot.selfdrive.car.helpers import convert_to_capnp
    from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque
    from openpilot.sunnypilot.selfdrive.car import interfaces as sp_interfaces
    from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.model import NNTorqueModel

    p = Params()
    p.put_bool('NeuralNetworkLateralControl', True, block=True)
    ci_type = interfaces[CAR.HONDA_ODYSSEY]
    cp = ci_type.get_non_essential_params(CAR.HONDA_ODYSSEY)
    sp = ci_type.get_non_essential_params_sp(cp, CAR.HONDA_ODYSSEY)
    ci = ci_type(cp, sp)
    sp_interfaces.setup_interfaces(ci, p)
    sp = convert_to_capnp(sp)
    p.remove('NnlcModelOverride')
    baseline = LatControlTorque(cp.as_reader(), sp.as_reader(), ci, 0.01).extension
    self.assertIsNone(baseline.response_shaper)
    x = [20.0, 0.2, 0.0, 0.0] + [0.2] * 7 + [0.0] * 7
    self.assertEqual(baseline.model.evaluate(x), NNTorqueModel(sp.neuralNetworkLateralControl.model.path).evaluate(x))
    p.put('NnlcModelOverride', 'HONDA_ODYSSEY_WILLIAM_S3.json', block=True)
    candidate = LatControlTorque(cp.as_reader(), sp.as_reader(), ci, 0.01).extension
    self.assertIsNotNone(candidate.response_shaper)
    self.assertEqual(candidate.response_shaper.ff_tau, 0.02)
