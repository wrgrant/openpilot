from openpilot.common.test import OpenpilotTestCase
import numpy as np
from tools.odyssey_sim.calib.identify import S
from tools.odyssey_sim.plan.models import PortableModel


class TestPortableModels(OpenpilotTestCase):
  def test_both_portable_models_execute_recurrently_with_finite_plans(self):
    for name in ('TSFDOM', 'CHEDDAR'):
      path = S/'plan'/f'{name}.onnx'
      if not path.exists():
        self.skipTest('hash-verified research ONNX artifacts are not present')
      model = PortableModel(path)
      images = np.full((2, 6, 128, 256), 128, np.uint8)
      first = model.run(images, 0, False, [.4, .875])['plan'].copy()
      for _ in range(24):
        output = model.run(images, 0, False, [.4, .875])
      self.assertEqual(output['plan'].shape, (1, 33, 15))
      self.assertTrue(np.isfinite(output['plan']).all())
      self.assertGreater(np.max(abs(first-output['plan'])), 1e-5)
