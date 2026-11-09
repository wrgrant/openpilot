"""Bounded experimental inference; stock models retain their original evaluator."""

import numpy as np
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.model import NNTorqueModel


class SafeNNTorqueModel(NNTorqueModel):
  def __init__(self, path, stock_path):
    self.fallback = NNTorqueModel(stock_path)
    super().__init__(path)

  def evaluate(self, input_array):
    if not 2 <= len(input_array) <= self.input_size:
      return 0.0
    x = np.asarray(input_array, dtype=np.float32)
    if np.isfinite(x).all():
      value = super().evaluate(input_array)
      if np.isfinite(value):
        return float(np.clip(value, -1.0, 1.0))
    # Missing/invalid live values must not propagate NaN into the PID.
    clean = np.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0).tolist()
    value = self.fallback.evaluate(clean)
    return float(np.clip(value, -1.0, 1.0)) if np.isfinite(value) else 0.0
