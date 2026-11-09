import os
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.model import NNTorqueModel
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel
from openpilot.common.parameterized import parameterized
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc import helpers


class TestModelOverride(OpenpilotTestCase):
  @parameterized.expand([None, '', 'missing.json', '../HONDA_ODYSSEY.json', '/tmp/model.json', 'HONDA_ODYSSEY_WILLIAM_V2.json'])
  def test_invalid_override_falls_back(self, value):
    with patch.object(helpers, 'Params') as params:
      params.return_value.get.return_value = value
      self.assertEqual(helpers.get_nn_model_override('stock.json'), 'stock.json')

  def test_existing_drop_in_override(self):
    name = 'HONDA_ODYSSEY.json'
    with patch.object(helpers, 'Params') as params:
      params.return_value.get.return_value = name
      self.assertEqual(helpers.get_nn_model_override('stock.json'), os.path.join(helpers.TORQUE_NN_MODEL_PATH, name))

  def test_default_fingerprint_stays_stock(self):
    cp = SimpleNamespace(carFingerprint='HONDA_ODYSSEY', carFw=[], steerControlType=0)
    path, _, _ = helpers.get_nn_model_path(cp)
    self.assertEqual(os.path.basename(path), 'HONDA_ODYSSEY.json')


class TestVanModel(OpenpilotTestCase):
  def test_real_params_selection_and_missing_fallback(self):
    p = Params()
    p.remove("NnlcModelOverride")
    self.assertEqual(helpers.get_nn_model_override("stock.json"), "stock.json")
    name = "HONDA_ODYSSEY_WILLIAM_V1.json"
    p.put("NnlcModelOverride", name, block=True)
    self.assertEqual(os.path.basename(helpers.get_nn_model_override("stock.json")), name)
    p.put("NnlcModelOverride", "missing.json", block=True)
    self.assertEqual(helpers.get_nn_model_override("stock.json"), "stock.json")

  @parameterized.expand([2.0, 5.0, 10.0, 20.0, 30.0, 40.0])
  def test_model_finite_and_monotonic(self, speed):
    model = NNTorqueModel(os.path.join(helpers.ODYSSEY_MODEL_PATH, "HONDA_ODYSSEY_WILLIAM_V1.json"))
    for roll in (-0.08, 0.0, 0.08):
      outputs = [model.evaluate([speed, a, 0.0, roll] + [a] * 7 + [roll] * 7) for a in np.linspace(-3.0, 3.0, 121)]
      self.assertTrue(np.isfinite(outputs).all())
      self.assertTrue((np.diff(outputs) >= -1e-6).all())

  def test_experimental_output_bounded_and_missing_values_fall_back(self):
    stock = os.path.join(helpers.TORQUE_NN_MODEL_PATH, "HONDA_ODYSSEY.json")
    model = SafeNNTorqueModel(os.path.join(helpers.ODYSSEY_MODEL_PATH, "HONDA_ODYSSEY_WILLIAM_V1.json"), stock)
    rng = np.random.default_rng(42)
    for x in rng.uniform(-100.0, 100.0, (500, 18)):
      self.assertLessEqual(abs(model.evaluate(x.tolist())), 1.0)
    for invalid in (float('nan'), float('inf'), -float('inf')):
      self.assertTrue(np.isfinite(model.evaluate([20.0, invalid, 0.0, 0.0])))

  @parameterized.expand([0, 1])
  def test_override_reaches_both_controller_extensions(self, version):
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.honda.values import CAR
    from openpilot.selfdrive.car.helpers import convert_to_capnp
    from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
    from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as V0
    from openpilot.sunnypilot.selfdrive.car import interfaces as sp_interfaces

    p = Params()
    p.put_bool('NeuralNetworkLateralControl', True, block=True)
    ci_type = interfaces[CAR.HONDA_ODYSSEY]
    cp = ci_type.get_non_essential_params(CAR.HONDA_ODYSSEY)
    sp = ci_type.get_non_essential_params_sp(cp, CAR.HONDA_ODYSSEY)
    ci = ci_type(cp, sp)
    sp_interfaces.setup_interfaces(ci, p)
    sp = convert_to_capnp(sp)
    cls = V0 if version == 0 else LatControlTorque
    baseline = cls(cp.as_reader(), sp.as_reader(), ci, 0.01).extension.model
    x = [20.0, 0.5, 0.0, 0.0] + [0.5] * 7 + [0.0] * 7
    self.assertEqual(baseline.evaluate(x), NNTorqueModel(sp.neuralNetworkLateralControl.model.path).evaluate(x))
    p.put('NnlcModelOverride', 'HONDA_ODYSSEY_WILLIAM_V1.json', block=True)
    selected = cls(cp.as_reader(), sp.as_reader(), ci, 0.01).extension.model
    self.assertIsInstance(selected, SafeNNTorqueModel)
    self.assertNotEqual(selected.evaluate(x), baseline.evaluate(x))


class TestModelAB(OpenpilotTestCase):
  @parameterized.expand(["HONDA_ODYSSEY_WILLIAM_S3.json", "HONDA_ODYSSEY_WILLIAM_S5.json"])
  def test_candidates_ship_in_repo_and_validate(self, name):
    path = helpers.validated_model_path(name)
    self.assertIsNotNone(path)
    self.assertEqual(os.path.dirname(path), os.path.realpath(helpers.ODYSSEY_MODEL_PATH))

  def test_model_ab_switches_model_and_resets(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import WANDER_BLOCK_S
    calls = []
    shaper = SimpleNamespace(reset=lambda: calls.append("shaper"))
    fake = SimpleNamespace(model_ab={"stock": ("m0", None), "S3": ("m3", shaper), "S5": ("m5", shaper)}, model_arm="stock",
                           model="m0", response_shaper=None, _pid=SimpleNamespace(reset=lambda: calls.append("pid")))
    NeuralNetworkLateralControl.update_model_ab(fake, 0.5 * WANDER_BLOCK_S)
    self.assertEqual((fake.model, calls), ("m0", []))
    NeuralNetworkLateralControl.update_model_ab(fake, 1.5 * WANDER_BLOCK_S)
    self.assertEqual((fake.model, calls), ("m3", ["shaper", "pid"]))
    NeuralNetworkLateralControl.update_model_ab(fake, 2.5 * WANDER_BLOCK_S)
    self.assertEqual(fake.model, "m5")
    del fake.model_ab["S5"]  # a candidate that failed to load runs stock instead
    NeuralNetworkLateralControl.update_model_ab(fake, 2.6 * WANDER_BLOCK_S)
    self.assertEqual(fake.model, "m0")
