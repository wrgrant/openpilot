"""Portable default-model ONNX execution, recurrent state owned by the graph.

Sampling matches tinygrad examples/openpilot/compile_warp.py: nearest, clamped
NV12 planes, Y00/Y10/Y01/Y11/U/V packing. No RGB conversion or second resample.
"""
import base64
import pickle
import time
import numpy as np
import onnxruntime as ort
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


def sample(plane, matrix, width, height):
  yy, xx = np.indices((height, width), dtype=np.float32)
  m = matrix.astype(np.float32)
  z = m[2, 0] * xx + m[2, 1] * yy + m[2, 2]
  x = np.rint((m[0, 0] * xx + m[0, 1] * yy + m[0, 2]) / z)
  y = np.rint((m[1, 0] * xx + m[1, 1] * yy + m[1, 2]) / z)
  return plane[y.clip(0, plane.shape[0] - 1).astype(int), x.clip(0, plane.shape[1] - 1).astype(int)]


def prepare(nv12, width, height, matrix):
  y = nv12[:width * height].reshape(height, width)
  uv = nv12[width * height:].reshape(height // 2, width)
  y = sample(y, matrix, 512, 256)
  uv_matrix = matrix * np.array([[1, 1, .5], [1, 1, .5], [2, 2, 1]], dtype=np.float32)
  u = sample(uv[:, ::2], uv_matrix, 256, 128)
  v = sample(uv[:, 1::2], uv_matrix, 256, 128)
  return np.stack([y[::2, ::2], y[1::2, ::2], y[::2, 1::2], y[1::2, 1::2], u, v])


class DefaultModel:
  def __init__(self, path, threads=4):
    opt = ort.SessionOptions()
    opt.intra_op_num_threads = threads
    self.session = ort.InferenceSession(str(path), opt, providers=['CPUExecutionProvider'])
    self.metadata = self.session.get_modelmeta().custom_metadata_map
    # Trusted, repository-owned ONNX only; output_slices uses upstream pickle encoding.
    self.slices = pickle.loads(base64.b64decode(self.metadata['output_slices']))
    self.state = {i.name: np.zeros(i.shape, np.uint8 if i.type == 'tensor(uint8)' else np.float32)
                  for i in self.session.get_inputs() if i.name.startswith('state_')}
    self.parser = Parser()
    self.last_desire = np.zeros(8, dtype=np.float32)
    self.seconds = 0.
    self.frames = 0

  def run(self, images, desire, rhd, action_t):
    vec = np.zeros(8, dtype=np.float32)
    if 0 < desire < 8:
      vec[desire] = 1
    pulse = np.where(vec - self.last_desire > .99, vec, 0)
    self.last_desire = vec
    inputs = dict(self.state, new_img=images, desire=pulse, traffic_convention=np.array([[not rhd, rhd]], dtype=np.float32),
                  action_t=np.array([action_t], dtype=np.float32))
    t = time.perf_counter()
    outputs = self.session.run(None, inputs)
    self.seconds += time.perf_counter() - t
    self.frames += 1
    for name, value in zip(self.session.get_outputs()[1:], outputs[1:], strict=True):
      self.state[name.name.removeprefix('next_')] = value
    raw = outputs[0]
    return self.parser.parse_outputs({k: raw[:, v].copy() for k, v in self.slices.items()})
