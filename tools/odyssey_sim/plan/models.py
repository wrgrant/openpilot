"""Portable source ONNX adapters; external artifacts must be hash-verified first.

Legacy queues mirror modeld_v2/compile_modeld.py and stock_dependencies.py.
"""
import time
import numpy as np
from tools.odyssey_sim.vsim.model import DefaultModel


class PortableModel(DefaultModel):
  def __init__(self, path, threads=2):
    super().__init__(path, threads)
    self.legacy = 'new_img' not in {i.name for i in self.session.get_inputs()}
    if self.legacy:
      contract = {i.name: i.shape for i in self.session.get_inputs()}
      expected = {'img': [1, 12, 128, 256], 'big_img': [1, 12, 128, 256], 'desire_pulse': [1, 33, 8],
                  'traffic_convention': [1, 2], 'action_t': [1, 2], 'features_buffer': [1, 32, 512]}
      if contract != expected:
        raise ValueError(f'Unsupported portable input contract: {contract}')
      self.img = np.zeros((2, 5, 6, 128, 256), np.uint8)
      self.desire = np.zeros((132, 1, 8), np.float16)
      self.feat = np.zeros((128, 1, 512), np.float16)
      self.previous_feature = np.zeros((1, 512), np.float16)

  def run(self, images, desire, rhd, action_t):
    if not self.legacy:
      return super().run(images, desire, rhd, action_t)
    vec = np.zeros(8, np.float32)
    if 0 < desire < 8:
      vec[desire] = 1
    pulse = np.where(vec-self.last_desire > .99, vec, 0)
    self.last_desire = vec
    self.img[:, :-1] = self.img[:, 1:]
    self.img[:, -1] = images
    self.desire[:-1] = self.desire[1:]
    self.desire[-1] = pulse
    self.feat[:-1] = self.feat[1:]
    self.feat[-1] = self.previous_feature
    inputs = {'img': self.img[0, ::4].reshape(1, 12, 128, 256), 'big_img': self.img[1, ::4].reshape(1, 12, 128, 256),
              'desire_pulse': self.desire.reshape(33, 4, 1, 8).max(axis=1).reshape(1, 33, 8),
              'features_buffer': self.feat[::4].reshape(1, 32, 512),
              'traffic_convention': np.array([[not rhd, rhd]], np.float16), 'action_t': np.array([action_t], np.float16)}
    t = time.perf_counter()
    raw = self.session.run(None, inputs)[0].astype(np.float32)
    self.seconds += time.perf_counter()-t
    self.frames += 1
    self.previous_feature = raw[:, self.slices['hidden_state']].astype(np.float16)
    return self.parser.parse_outputs({k: raw[:, v].copy() for k, v in self.slices.items()})
