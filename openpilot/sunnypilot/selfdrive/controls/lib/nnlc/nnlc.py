"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import deque
import json
import math
import time
import numpy as np

from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from opendbc.sunnypilot.car.interfaces import LatControlInputs
from opendbc.sunnypilot.car.lateral_ext import get_friction as get_friction_in_torque_space
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_base import sign
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_jerk_aware import LatControlTorqueJerkAware
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import NNLC_ARMS, NNLC_MODEL_ARMS, NNLC_MODEL_FILES, WANDER_BLOCK_S, ab_arm
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.helpers import MOCK_MODEL_PATH, get_nn_model_override, validated_model_path
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.model import NNTorqueModel
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [12, 3, 1, 0]


# At a given roll, if pitch magnitude increases, the
# gravitational acceleration component starts pointing
# in the longitudinal direction, decreasing the lateral
# acceleration component. Here we do the same thing
# to the roll value itself, then passed to nnff.
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)


class NnlcResponseShaper:
  """Opt-in causal feedback/FF shaping stored with the experimental model.

  Keep torque feedback and planned feedforward separate: damping the error need
  not delay turn anticipation. Constants are seconds at the native 100 Hz rate.
  """
  def __init__(self, error_gain=1.0, error_tau=0.0, ff_tau=0.0):
    self.error_gain = error_gain
    self.error_tau = error_tau
    self.ff_tau = ff_tau
    self.reset()

  @classmethod
  def load(cls, path):
    with open(path) as f:
      settings = json.load(f).get('odyssey_smoothing')
    if settings is None:
      return None
    limits = {'error_gain': (0.1, 3.0), 'error_tau': (0.0, 0.4), 'ff_tau': (0.0, 0.3)}
    if not isinstance(settings, dict) or settings.keys() - limits.keys():
      raise ValueError('Invalid Odyssey smoothing schema')
    for key, value in settings.items():
      if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not limits[key][0] <= value <= limits[key][1]:
        raise ValueError('Invalid Odyssey smoothing value')
    return cls(**settings)

  def reset(self):
    self.error = self.ff = None
    self.last_update = None

  def update(self, error, ff):
    if not math.isfinite(error) or not math.isfinite(ff):
      self.reset()
      raise ValueError('Nonfinite NNLC response')
    now = time.monotonic()
    if self.last_update is not None and now - self.last_update > .1:
      self.reset()
    self.last_update = now
    error *= self.error_gain
    self.error = error if self.error is None else self.error + .01 / (self.error_tau + .01) * (error - self.error)
    self.ff = ff if self.ff is None else self.ff + .01 / (self.ff_tau + .01) * (ff - self.ff)
    return self.error, self.ff


class NeuralNetworkLateralControl(LatControlTorqueJerkAware):
  def __init__(self, lac_torque, CP, CP_SP, CI):
    super().__init__(lac_torque, CP, CP_SP, CI)
    self.params = Params()
    self.enabled = self.params.get_bool("NeuralNetworkLateralControl")
    # Odyssey: NnlcAB rotates NNLC on and off in odyssey_ab blocks to compare it with the stock torque controller
    self.ab = self.params.get_bool("NnlcAB")
    stock_path = CP_SP.neuralNetworkLateralControl.model.path
    model_path = get_nn_model_override(stock_path) if stock_path not in (MOCK_MODEL_PATH, '') else stock_path
    self.has_nn_model = model_path not in (MOCK_MODEL_PATH, '')

    # NN model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
    # of lat accel and roll
    # Past value is computed using previous desired lat accel and observed roll
    self.model = NNTorqueModel(model_path) if self.has_nn_model else None
    if self.has_nn_model and model_path != stock_path:
      self.model = SafeNNTorqueModel(model_path, stock_path)

    self.response_shaper = None
    if self.has_nn_model and model_path != stock_path:
      try:
        self.response_shaper = NnlcResponseShaper.load(model_path)
      except (OSError, ValueError, TypeError):
        # Invalid experimental metadata restores the original controller/model.
        self.model = NNTorqueModel(stock_path)

    # Odyssey: NnlcModelAB rotates stock / van-specific models in WANDER_BLOCK_S blocks (odyssey_ab.NNLC_MODEL_ARMS)
    self.model_ab = None
    if self.has_nn_model and self.params.get_bool("NnlcModelAB"):
      arms = {"stock": (self.model, self.response_shaper)}
      for arm, name in NNLC_MODEL_FILES.items():
        path = validated_model_path(name)
        if path is None:
          continue
        try:
          arms[arm] = (SafeNNTorqueModel(path, stock_path), NnlcResponseShaper.load(path))
        except (OSError, ValueError, TypeError):
          continue
      if len(arms) > 1:
        self.model_ab = arms
        self.model_arm = "stock"

    self.pitch = FirstOrderFilter(0.0, 0.5, 0.01)
    self.pitch_last = 0.0

    # setup future time offsets
    self.future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
    self.nn_future_times = [i + self.desired_lat_jerk_time for i in self.future_times]

    # setup past time offsets
    self.past_times = [-0.3, -0.2, -0.1]
    history_check_frames = [int(abs(i)*100) for i in self.past_times]
    self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
    self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
    self.roll_deque = deque(maxlen=history_check_frames[0])
    self.error_deque = deque(maxlen=history_check_frames[0])
    self.past_future_len = len(self.past_times) + len(self.nn_future_times)

  @property
  def _nnlc_enabled(self):
    return self.enabled and self.model_valid and self.has_nn_model

  def update_limits(self):
    super().update_limits()
    if not self._nnlc_enabled:
      return

    self._pid.set_limits(self.lac_torque.steer_max, -self.lac_torque.steer_max)

  def update_lateral_lag(self, lag):
    super().update_lateral_lag(lag)
    self.nn_future_times = [t + self.desired_lat_jerk_time for t in self.future_times]

  def update_feedforward_torque_space(self, CS):
    torque_from_setpoint = self.torque_from_lateral_accel_in_torque_space(LatControlInputs(self._setpoint, self._roll_compensation, CS.vEgo, CS.aEgo),
                                                                          self.torque_params, gravity_adjusted=False)
    torque_from_measurement = self.torque_from_lateral_accel_in_torque_space(LatControlInputs(self._measurement, self._roll_compensation, CS.vEgo, CS.aEgo),
                                                                             self.torque_params, gravity_adjusted=False)
    self._pid_log.error = float(torque_from_setpoint - torque_from_measurement)  # ty: ignore[invalid-assignment]
    self._ff = self.torque_from_lateral_accel_in_torque_space(LatControlInputs(self._gravity_adjusted_lateral_accel, self._roll_compensation,
                                                                               CS.vEgo, CS.aEgo), self.torque_params, gravity_adjusted=True)
    self._ff += get_friction_in_torque_space(self._desired_lateral_accel - self._actual_lateral_accel, self._lateral_accel_deadzone,
                                             FRICTION_THRESHOLD, self.torque_params)

  def update_ab(self, mono_s: float) -> None:
    """Switch NNLC for the current A/B block. The two modes run the PID in different units (torque vs lateral accel),
    so a switch resets it and sets its limits for the new mode."""
    want = ab_arm(mono_s, NNLC_ARMS) == "on"
    if want == self.enabled:
      return
    self.enabled = want
    if getattr(self, 'response_shaper', None) is not None:
      self.response_shaper.reset()
    self._pid.reset()
    self.lac_torque.update_limits()
    self.update_limits()

  def update_model_ab(self, mono_s: float) -> None:
    """Switch to this block's model (missing candidates fall back to stock); reset the PID and shaper on a switch."""
    arm = ab_arm(mono_s, NNLC_MODEL_ARMS, WANDER_BLOCK_S)
    arm = arm if arm in self.model_ab else "stock"
    if arm == self.model_arm:
      return
    self.model_arm = arm
    self.model, self.response_shaper = self.model_ab[arm]
    if self.response_shaper is not None:
      self.response_shaper.reset()
    self._pid.reset()

  def update_neural_network_feedforward(self, CS, params, calibrated_pose) -> None:
    if self.ab:
      self.update_ab(time.monotonic())
    if self.model_ab is not None:
      self.update_model_ab(time.monotonic())
    if not self._nnlc_enabled:
      return

    self.update_feedforward_torque_space(CS)

    low_speed_factor = float(np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)) ** 2
    self._setpoint = self._desired_lateral_accel + low_speed_factor * self._desired_curvature
    self._measurement = self._actual_lateral_accel + low_speed_factor * self._actual_curvature

    # update past data
    roll = params.roll
    if calibrated_pose is not None:
      pitch = self.pitch.update(calibrated_pose.orientation.pitch)
      roll = roll_pitch_adjust(roll, pitch)
      self.pitch_last = pitch
    self.roll_deque.append(roll)
    self.lateral_accel_desired_deque.append(self._desired_lateral_accel)

    # prepare past and future values
    # adjust future times to account for longitudinal acceleration
    adjusted_future_times = [t + 0.5 * CS.aEgo * (t / max(CS.vEgo, 1.0)) for t in self.nn_future_times]
    past_rolls = [self.roll_deque[min(len(self.roll_deque) - 1, i)] for i in self.history_frame_offsets]
    future_rolls = [roll_pitch_adjust(np.interp(t, ModelConstants.T_IDXS, self.model_v2.orientation.x) + roll,
                                      np.interp(t, ModelConstants.T_IDXS, self.model_v2.orientation.y) + self.pitch_last) for t in
                    adjusted_future_times]
    past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque) - 1, i)]
                                   for i in self.history_frame_offsets]
    future_planned_lateral_accels = [np.interp(t, ModelConstants.T_IDXS, self.model_v2.acceleration.y) for t in
                                     adjusted_future_times]

    # compute NNFF error response
    nnff_setpoint_input = [CS.vEgo, self._setpoint, self.lateral_jerk_setpoint, roll] \
                          + [self._setpoint] * self.past_future_len \
                          + past_rolls + future_rolls
    # past lateral accel error shouldn't count, so use past desired like the setpoint input
    nnff_measurement_input = [CS.vEgo, self._measurement, self.lateral_jerk_measurement, roll] \
                             + [self._measurement] * self.past_future_len \
                             + past_rolls + future_rolls
    torque_from_setpoint = self.model.evaluate(nnff_setpoint_input)
    torque_from_measurement = self.model.evaluate(nnff_measurement_input)
    self._pid_log.error = torque_from_setpoint - torque_from_measurement  # ty: ignore[invalid-assignment]

    # The "pure" NNLC error response can be too weak for cars whose models were trained
    # with a lack of high-magnitude lateral acceleration data, for which the NNLC model
    # torque response flattens out at high lateral accelerations.
    # This workaround blends in a guaranteed stronger error response only when the
    # desired lateral acceleration is high enough to warrant it, by using the lateral acceleration
    # error as the input to the NNLC model. This is not ideal, and potentially degrades the NNLC
    # accuracy for cars that don't have this issue, but it's necessary until a better NNLC model
    # structure is used that doesn't create this issue when high-magnitude data is missing.
    error_blend_factor = float(np.interp(abs(self._desired_lateral_accel), [1.0, 2.0], [0.0, 1.0]))
    if error_blend_factor > 0.0:  # blend in stronger error response when in high lat accel
      # NNFF inputs 5+ are optional, and if left out are replaced with 0.0 inside the NNFF class
      nnff_error_input = [CS.vEgo, self._setpoint - self._measurement, self.lateral_jerk_setpoint - self.lateral_jerk_measurement, 0.0]
      torque_from_error = self.model.evaluate(nnff_error_input)
      if sign(self._pid_log.error) == sign(torque_from_error) and abs(self._pid_log.error) < abs(torque_from_error):
        self._pid_log.error = self._pid_log.error * (1.0 - error_blend_factor) + torque_from_error * error_blend_factor  # ty: ignore[invalid-assignment]

    # compute feedforward (same as nn setpoint output)
    friction_input = self.update_friction_input(self._setpoint, self._measurement)
    nn_input = [CS.vEgo, self._desired_lateral_accel, friction_input, roll] \
               + past_lateral_accels_desired + future_planned_lateral_accels \
               + past_rolls + future_rolls
    self._ff = self.model.evaluate(nn_input)

    # apply friction override for cars with low NN friction response
    if self.model.friction_override:
      self._pid_log.error += get_friction(friction_input, self._lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if self.response_shaper is not None:
      if CS.steeringPressed or CS.vEgo < 5:
        self.response_shaper.reset()
      else:
        try:
          self._pid_log.error, self._ff = self.response_shaper.update(self._pid_log.error, self._ff)
        except ValueError:
          # Safe model already sanitizes inference inputs; avoid stale filter state.
          self._pid_log.error = 0.0
          self._ff = 0.0
    self.update_output_torque(CS)
