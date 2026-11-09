"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os
import tomllib
from difflib import SequenceMatcher

from opendbc.car import structs
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.model import NNTorqueModel
import numpy as np

TORQUE_NN_MODEL_PATH = os.path.join(BASEDIR, "openpilot", "sunnypilot", "neural_network_data", "neural_network_lateral_control")
TORQUE_NN_MODEL_SUBSTITUTE_PATH = os.path.join(BASEDIR, "opendbc_repo", "opendbc", "car", "torque_data/substitute.toml")
MOCK_MODEL_PATH = os.path.join(TORQUE_NN_MODEL_PATH, "MOCK.json")
# Odyssey: van-specific models live in our own repo (the neural_network_data submodule is sunnypilot's)
ODYSSEY_MODEL_PATH = os.path.join(BASEDIR, "openpilot", "sunnypilot", "selfdrive", "controls", "lib", "nnlc", "odyssey_models")


def similarity(s1: str, s2: str) -> float:
  return SequenceMatcher(None, s1, s2).ratio()


def get_nn_model_path(CP: structs.CarParams) -> tuple[str, str, bool]:
  car_fingerprint = CP.carFingerprint
  eps_fw = str(next((fw.fwVersion for fw in CP.carFw if fw.ecu == "eps"), ""))

  def check_nn_path(_nn_candidate):
    _model_path = None
    _max_similarity = -1.0
    for f in os.listdir(TORQUE_NN_MODEL_PATH):
      if f.endswith(".json") and "_WILLIAM_" not in f:
        model = os.path.splitext(f)[0]
        similarity_score = similarity(model, _nn_candidate)
        if similarity_score > _max_similarity:
          _max_similarity = similarity_score
          _model_path = os.path.join(TORQUE_NN_MODEL_PATH, f)
    return _model_path, _max_similarity

  if len(eps_fw) > 3:
    eps_fw = eps_fw.replace("\\", "")
    nn_candidate = f"{car_fingerprint} {eps_fw}"
  else:
    nn_candidate = car_fingerprint

  model_path, max_similarity = check_nn_path(nn_candidate)
  exact_match = max_similarity >= 0.99

  if car_fingerprint not in model_path or 0.0 <= max_similarity < 0.9:
    nn_candidate = car_fingerprint
    model_path, max_similarity = check_nn_path(nn_candidate)
    exact_match = max_similarity >= 0.99

    if 0.0 <= max_similarity < 0.9:
      with open(TORQUE_NN_MODEL_SUBSTITUTE_PATH, 'rb') as f:
        sub = tomllib.load(f)
      sub_candidate = sub.get(car_fingerprint, car_fingerprint)

      for candidate in [car_fingerprint, sub_candidate]:
        model_path, max_similarity = check_nn_path(candidate)

      exact_match = False

  if CP.steerControlType == structs.CarParams.SteerControlType.angle:
    model_path = MOCK_MODEL_PATH

  model_name = os.path.splitext(os.path.basename(model_path))[0]

  return model_path, model_name, exact_match


def validated_model_path(name: str | None) -> str | None:
  """Path of a valid 18-input drop-in model named `name` (a bare .json filename) in our Odyssey model folder or
  sunnypilot's model folder, or None. Experimental files never take part in fuzzy fingerprint matching."""
  if not name or os.path.basename(name) != name or not name.endswith(".json"):
    return None
  for folder in (ODYSSEY_MODEL_PATH, TORQUE_NN_MODEL_PATH):
    path = os.path.realpath(os.path.join(folder, name))
    if os.path.dirname(path) != os.path.realpath(folder) or not os.path.isfile(path):
      continue
    try:
      model = NNTorqueModel(path)
      if model.input_size != 18 or model.output_size != 1:
        return None
      if not np.isfinite(model.input_mean).all() or not np.isfinite(model.input_std).all() or np.any(model.input_std <= 0):
        return None
      for accel in (-3.0, 0.0, 3.0):
        if not np.isfinite(model.evaluate([20.0, accel, 0.0, 0.0] + [accel] * 7 + [0.0] * 7)):
          return None
    except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError):
      return None
    return path
  return None


def get_nn_model_override(default_path: str) -> str:
  """Select a validated drop-in model at startup (NnlcModelOverride); unset/invalid keeps stock."""
  return validated_model_path(Params().get("NnlcModelOverride")) or default_path
