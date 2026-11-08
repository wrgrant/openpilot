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


def test_fine_angle_sees_motion_between_ticks():
  from tools.odyssey_eps_report import fine_angle
  rng = np.random.default_rng(0)
  t = np.arange(3000) / 100
  true = 0.15 * np.sin(2 * np.pi * t / 10)  # slow +-0.15 degree wheel motion
  rate = np.round((np.gradient(true) * 100 + rng.normal(0, 0.05, len(t))) / 0.1) * 0.1  # 0.1 deg/s steps, noisy
  reading = np.round(true / 0.1) * 0.1
  fine = fine_angle(reading, rate)
  assert np.std(fine - true) < 0.5 * np.std(reading - true)


def test_torque_pulses_find_the_deadband():
  from openpilot.common.realtime import DT_MDL
  from openpilot.tools.lateral_maneuvers.odyssey_suite import PULSE_LEVELS, pulse_train
  from tools.odyssey_eps_report import analyze_pulses
  (action,) = pulse_train(PULSE_LEVELS, 0.2, 1.0)
  t = np.arange(0, action.time_bp[-1], 0.01)
  probe = np.interp(np.floor(t / DT_MDL) * DT_MDL, action.time_bp, action.accel_bp)  # held at 20 Hz like the plan
  threshold, ang, a = 0.045, np.zeros(len(t)), 0.0
  for i in range(len(t)):  # the wheel turns only while the pulse is past the deadband
    if abs(probe[i]) > threshold:
      a += np.sign(probe[i]) * 0.003
    ang[i] = a
  d = {"probe": probe, "fine": ang, "cmd_tq": -probe * STEER_MAX, "vsa_lat": ang * 0.05}
  pulses = analyze_pulses(d, np.arange(len(t)), 0.0)
  assert len(pulses) == 2 * len(PULSE_LEVELS)
  moved = sorted({p["level"] for p in pulses if p["moved"]})
  assert moved == [lv for lv in PULSE_LEVELS if lv > threshold]
  assert {p["direction"] for p in pulses if p["moved"]} == {1.0, -1.0}
