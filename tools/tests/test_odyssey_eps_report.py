"""The characterization report finds the torque at which a sticky wheel breaks away, even when it coasts first."""
import numpy as np

from tools.odyssey_eps_report import STEER_MAX, analyze_run


def test_breakaway_found_after_coasting():
  n, threshold = 1200, 0.05
  t = np.arange(n) / 100
  req = np.where(t < 4, 0.05, -0.05)  # request flips left -> right at 4 s
  cmd = np.clip(np.arange(n) * 0.0006, -0.2, 0.2)
  cmd[400:] = np.clip(cmd[399] - np.arange(1, n - 399) * 0.0012, -0.2, 0.2)  # command winds back down through zero
  ang, a = np.zeros(n), 0.0
  for i in range(n):  # the wheel only moves while the command is past the threshold, read in 0.1 degree steps
    if abs(cmd[i]) > threshold:
      a += np.sign(cmd[i]) * 0.004
    ang[i] = np.round(a, 1)
  d = {"t": t, "req_la": req, "cmd_tq": -cmd * STEER_MAX, "angle": ang, "motor_tq": np.abs(cmd) * 100,
       "vsa_lat": np.roll(ang * 0.05, 20)}
  (direction, _, torque), = analyze_run(d, np.arange(n), 0.0)["breakaway"]
  assert direction == -1
  assert threshold <= torque <= threshold + 0.01
