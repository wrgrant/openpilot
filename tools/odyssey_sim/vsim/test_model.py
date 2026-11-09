from pathlib import Path
import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.vsim.model import prepare, sample, DefaultModel


class TestFrameInput(OpenpilotTestCase):
  def test_nv12_is_packed_without_rgb_or_chroma_swap(self):
    y = np.empty((256, 512), dtype=np.uint8)
    y[::2, ::2], y[1::2, ::2], y[::2, 1::2], y[1::2, 1::2] = 11, 22, 33, 44
    uv = np.empty((128, 512), dtype=np.uint8)
    uv[:, ::2], uv[:, 1::2] = 55, 66
    out = prepare(np.r_[y.ravel(), uv.ravel()], 512, 256, np.eye(3))
    self.assertEqual(out.shape, (6, 128, 256))
    np.testing.assert_array_equal(out[:, 3, 7], [11, 22, 33, 44, 55, 66])

  def test_outside_view_clamps_instead_of_filling_black(self):
    image = np.array([[11, 22], [33, 44]], dtype=np.uint8)
    m = np.array([[1., 0, -100], [0, 1, 100], [0, 0, 1]])
    np.testing.assert_array_equal(sample(image, m, 2, 2), [[33, 33], [33, 33]])


class TestDefaultOnnx(OpenpilotTestCase):
  def test_recurrent_feature_buffer_affects_next_prediction(self):
    path = Path('openpilot/selfdrive/modeld/models/driving_supercombo.onnx')
    model, fresh = DefaultModel(path), DefaultModel(path)
    images = np.full((2, 6, 128, 256), 128, dtype=np.uint8)
    for _ in range(20):
      model.run(images, 0, False, [.4, .875])
    second = model.run(images, 0, False, [.4, .875])
    first = fresh.run(images, 0, False, [.4, .875])
    self.assertTrue(np.isfinite(second['plan']).all())
    self.assertGreater(np.max(abs(second['plan'] - first['plan'])), .001)
