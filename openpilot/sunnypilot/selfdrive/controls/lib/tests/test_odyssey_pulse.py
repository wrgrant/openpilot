"""Pulse test: the command during a shot is the plan's alone, every exit hands steering straight back, and the entry
rules are the ones replayed against recorded drives (times come from the module's constants, not literals)."""
from dataclasses import replace
import math
import time
import unittest

from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_pulse as op
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_pulse import Condition, Inputs, Pulse, PulseRuntime, plan

DT = .01
SETTLE_FRAMES = round(op.SETTLE_S / DT) + 1  # frames from the first ready one until a shot starts


def started(condition=None, normal=.02):
  p = Pulse()
  if condition is not None:
    p.plan = (('0:0', condition),)
  for n in range(SETTLE_FRAMES):
    p.step(n * DT, normal, Inputs(), True)
  assert p.in_shot, p.reason
  return p, (SETTLE_FRAMES - 1) * DT


class TestPulse(unittest.TestCase):
  def test_shot_starts_after_the_settling_time_at_zero_and_records_what_it_released(self):
    p = Pulse()
    for n in range(SETTLE_FRAMES - 1):
      self.assertEqual(p.step(n * DT, .04, Inputs(), True), .04, "lane keeping steers until the shot starts")
    self.assertFalse(p.in_shot)
    self.assertEqual(p.step((SETTLE_FRAMES - 1) * DT, .04, Inputs(), True), 0.)
    self.assertEqual(p.state, 'lead-in')
    self.assertAlmostEqual(p.record['released'], .04)
    self.assertEqual(p.record['baseline'], 0.)

  def test_no_shot_while_lane_keeping_holds_more_than_the_release_limit_or_the_van_is_not_settled(self):
    blocked = [{'normal': op.MAX_RELEASE + .01}, {'offset': op.START_OFFSET + .01}, {'heading': op.START_HEADING + .001},
               {'lane_width': 2 * (op.VAN_HALF_WIDTH + op.MIN_EDGE + op.MAX_DRIFT) - .02},  # no room to drift in this lane
               {'rate': .6}, {'driver_torque': op.DRIVER_TORQUE + 1}]
    for case in blocked:
      with self.subTest(case=case):
        p, normal = Pulse(), case.pop('normal', .02)
        for n in range(3 * SETTLE_FRAMES):
          self.assertEqual(p.step(n * DT, normal, replace(Inputs(), **case), True), normal)
        self.assertFalse(p.in_shot)

  def test_every_safety_exit_returns_normal_immediately_and_retries(self):
    cases = [('steering inactive', {'active': False}), ('invalid data', {'valid': False}),
             ('stale model', {'model_age': op.MAX_MODEL_AGE + .05}), ('blinker', {'blinker': True}), ('brake', {'brake': True}),
             ('lane change', {'lane_change': True}), ('speed band', {'speed': op.MAX_SPEED + 1}),
             ('lane lines lost', {'lane_prob': op.MIN_LANE_PROB - .01}), ('narrow lane', {'lane_width': op.MIN_LANE_WIDTH - .1}),
             ('offset', {'offset': op.ABORT_OFFSET + .01, 'lane_width': 6.}), ('heading', {'heading': op.ABORT_HEADING + .001}),
             ('near the line', {'lane_width': 3.3, 'offset': 3.3 / 2 - op.VAN_HALF_WIDTH - op.MIN_EDGE + .01}),
             ('curve ahead', {'curve_accel': op.MAX_CURVE_ACCEL + .01}), ("you're steering", {'driver_torque': op.HARD_DRIVER_TORQUE + 100}),
             ('speed changed', {'speed': 25. + op.MAX_SPEED_CHANGE + .1}), ('drifted', {'offset': op.MAX_DRIFT + .01}),
             ('invalid data', {'offset': float('nan')})]
    for reason, change in cases:
      with self.subTest(reason=reason):
        p, t = started()
        self.assertEqual(p.step(t + DT, .123, replace(Inputs(), **change), True), .123)
        self.assertEqual(p.events[-1]['abort'], reason)
        self.assertFalse(p.events[-1]['completed'])
        self.assertEqual(len(p.done['45-70']), 0)
        self.assertEqual(p.state, 'recover')

  def test_frame_gap_pause_deselect_and_sustained_driver(self):
    for gap, selected, paused, reason in ((op.MAX_FRAME_GAP + .05, True, False, 'frame gap'), (DT, False, False, 'not selected'),
                                          (DT, True, True, 'paused')):
      p, t = started()
      self.assertEqual(p.step(t + gap, .2, Inputs(), selected, paused), .2)
      self.assertEqual(p.events[-1]['abort'], reason)
    p, t = started()
    for n in range(1, round(op.DRIVER_SECONDS / DT) + 3):
      p.step(t + n * DT, .2, Inputs(driver_torque=op.DRIVER_TORQUE + 50), True)
    self.assertEqual(p.events[-1]['abort'], "you're steering")

  def test_phases_and_smooth_completion(self):
    c = Condition('single', .12, .2, 1)
    p, t = started(c)
    lead, pulse, total = (round(x / DT) for x in (op.LEAD_S, op.LEAD_S + c.duration, op.LEAD_S + c.duration + op.TAIL_S))
    phases, commands = [], []
    for n in range(1, total + 2):
      command = p.step(t + n * DT, .8, Inputs(), True)
      phases.append(p.state)
      commands.append(command)
      if n < total:
        self.assertEqual(command, .12 if lead <= n < pulse else 0., f"frame {n}: the shot's command is the plan's alone")
    self.assertEqual([s for i, s in enumerate(phases) if i == 0 or s != phases[i - 1]], ['lead-in', 'pulse', 'tail', 'recover'])
    self.assertLessEqual(commands[-1], 2 * op.HANDOVER_STEP + 1e-6, "lane keeping's 0.8 comes back in steps, not at once")
    self.assertTrue(p.events[-1]['completed'])
    self.assertEqual(p.done['45-70'], {'0:0'})

  def test_session_is_short_bounded_and_each_signal_is_what_it_says(self):
    shots, schedule = op.session(), plan()
    self.assertEqual(len(schedule), 3 * len(shots))
    self.assertEqual(len({i for i, _ in schedule}), len(schedule))
    self.assertLess(sum(op.LEAD_S + c.duration + op.TAIL_S for _, c in shots), 60, "one pass is under a minute of shots")
    self.assertEqual({name for name, _ in shots[:6]}, {'sham-a', 'square+', 'pair-6-0.1', 'ramp+', 'chirp-5', 'train-6-30+'},
                     "a drive cut short after six shots has one of each kind")
    for (_, a), (_, b) in zip(schedule[:len(shots)], schedule[len(shots):2 * len(shots)], strict=True):
      self.assertEqual(b.direction, -a.direction, "the second pass repeats the first with every first push reversed")
    for _, c in schedule:
      frames = round((op.LEAD_S + c.duration + op.TAIL_S) / DT)
      cmd = [c.command(n * DT) for n in range(frames + 5)]
      self.assertLessEqual(max(abs(x) for x in cmd), op.MAX_TORQUE)
      self.assertLess(op.LEAD_S + c.duration + op.TAIL_S, op.HARD_SHOT_S, "every planned shot ends before the hard time limit")
      lead, end = round(op.LEAD_S / DT), round((op.LEAD_S + c.duration) / DT)
      self.assertEqual(set(cmd[:lead]) | set(cmd[end:]), {0.}, "quiet before and after the signal")
    on = lambda c: [c.command(op.LEAD_S + n * DT) for n in range(round(c.duration / DT))]  # noqa: E731
    pair = on(Condition('pair', .06, 2 * .1 + op.PAIR_GAP_S, 1, period=.1))
    self.assertEqual((pair[:10], pair[10:60], pair[60:]), ([.06] * 10, [0.] * 50, [-.06] * 10))
    self.assertAlmostEqual(sum(pair), 0.)
    train = on(Condition('train', .08, op.TRAIN_S, -1, .25, .4))
    self.assertEqual(sum(x != 0 for x in train), round(op.TRAIN_S / DT * .4))
    ramp = on(dict(shots)['ramp-'])
    self.assertEqual(sorted({-x for x in ramp}), list(op.RAMP_LEVELS))
    self.assertEqual(ramp, sorted(ramp, reverse=True), "the ramp only ever grows")
    square = on(dict(shots)['square+'])
    self.assertEqual(sorted({abs(x) for x in square}), list(op.SQUARE_LEVELS))
    self.assertAlmostEqual(sum(square), 0., msg="each level is pushed both ways for the same time")
    chirp = on(dict(shots)['chirp-9'])
    self.assertLessEqual(max(abs(x) for x in chirp), .09)
    self.assertGreater(sum(a * b < 0 for a, b in zip(chirp, chirp[1:], strict=False)), 3, "the chirp reverses several times")
    self.assertEqual(set(on(dict(shots)['sham-a'])), {0.})

  def test_progress_round_trip_and_old_versions_are_ignored(self):
    schedule = plan()
    p = Pulse({'version': op.VERSION, 'done': {'45-70': [schedule[0][0], 'garbage']}})
    self.assertEqual(p.done['45-70'], {schedule[0][0]})
    self.assertEqual(Pulse(p.progress()).done, p.done)
    self.assertEqual(Pulse({'version': op.VERSION - 1, 'done': {'45-70': [schedule[0][0]]}}).done['45-70'], set())

  def test_ignoring_the_wheel_sensor_lets_a_shot_start_and_finish_with_torque_on_it(self):
    hands = Inputs(driver_torque=op.HARD_DRIVER_TORQUE + 500)
    p = Pulse()
    for n in range(3 * SETTLE_FRAMES):
      p.step(n * DT, .02, hands, True)
    self.assertFalse(p.in_shot, "obeyed: torque on the wheel keeps the test waiting")
    self.assertGreaterEqual(p.wheel_peak, op.HARD_DRIVER_TORQUE, "the panel can show the reading that blocked it")
    p = Pulse()
    p.ignore_wheel = True
    for n in range(SETTLE_FRAMES + round(2.5 / DT)):
      p.step(n * DT, .02, hands, True)
    done = [e for e in p.events if e.get('completed')]
    self.assertTrue(done, "ignored: shots run to the end")
    self.assertNotIn("you're steering", p.aborts)
    q, t = started()
    q.step(t + DT, .1, Inputs(driver_torque=op.HARD_DRIVER_TORQUE + 500), True)
    self.assertEqual(q.aborts, {"you're steering": 1}, "obeyed: the stop is counted for the panel")
    for change in ({'brake': True}, {'blinker': True}, {'active': False}):
      q, t = started()
      q.ignore_wheel = True
      self.assertEqual(q.step(t + DT, .1, replace(Inputs(), **change), True), .1, "the driver's other ways out still work")

  def test_fault_latches_and_inactive_is_passthrough(self):
    p = Pulse()
    for n in range(100):
      self.assertEqual(p.step(n * DT, .33, Inputs(), False), .33)
    self.assertEqual(p.step(1., .42, None, True), .42)
    self.assertTrue(p.broken)
    self.assertEqual(p.step(1.01, .42, Inputs(), True), .42)

  def test_timeout_invalid_clock_and_recovery_resume(self):
    p, t = started()
    p.start = t - op.HARD_SHOT_S
    self.assertEqual(p.step(t + DT, .17, Inputs(), True), .17)
    self.assertEqual(p.events[-1]['abort'], 'hard timeout')
    p, t = started()
    p.step(t + DT, .1, Inputs(blinker=True), True)
    for n in range(2, SETTLE_FRAMES + 12):  # lane keeping's command is slewed back first, then the settling time runs again
      p.step(t + n * DT, .02, Inputs(), True)
    self.assertEqual(p.state, 'lead-in')
    self.assertEqual(p.shot[0], p.plan[0][0], "an aborted shot is retried")
    p.step(float('nan'), .2, Inputs(), True)
    self.assertFalse(p.in_shot)

  def test_nonfinite_normal_fault_remains_passthrough(self):
    p, t = started()
    p.step(t + DT, float('nan'), Inputs(), True)
    self.assertFalse(p.in_shot)
    self.assertEqual(p.events[-1]['abort'], 'internal fault')


def runtime(controller=None, queue_size=512):
  """A PulseRuntime without its worker thread, selected and unpaused."""
  import queue
  r = PulseRuntime.__new__(PulseRuntime)
  r.controller = controller or Pulse()
  r.config = (True, False, None)
  r.heartbeat = time.monotonic()
  r.queue = queue.Queue(maxsize=queue_size)
  r.last_status = 0.
  r.pose, r.model_time, r.curve, r.width = None, -math.inf, math.inf, 0.
  return r


class TestPulseRuntime(unittest.TestCase):
  def test_runs_on_real_log_messages(self):
    """The first version sliced a capnp list, which raises: with real messages the test broke itself on its first frame
    and never fired. Drive the runtime with real message objects until a shot starts."""
    from openpilot.cereal import messaging
    from openpilot.selfdrive.modeld.constants import ModelConstants
    cs = messaging.new_message('carState').carState
    cs.vEgo = 27.
    msg = messaging.new_message('modelV2')
    md = msg.modelV2
    xs, ts = [float(x) for x in ModelConstants.X_IDXS], [float(t) for t in ModelConstants.T_IDXS]
    md.init('laneLines', 4)
    for line, y in zip(md.laneLines, (-5.55, -1.85, 1.85, 5.55), strict=True):
      line.x, line.y = xs, [y] * len(xs)
    md.laneLineProbs = [.9, .9, .9, .9]
    md.orientationRate.t, md.orientationRate.z = ts, [0.] * len(ts)
    md.velocity.t, md.velocity.x = ts, [27.] * len(ts)
    r = runtime()
    out = []
    for n in range(SETTLE_FRAMES + 30):
      now = 100. + n * DT
      out.append(r.update(now, .02, cs, md, True, n % 5 == 0, True, 0., 0.))
      r.heartbeat = time.monotonic()
    self.assertFalse(r.controller.broken)
    self.assertTrue(r.controller.in_shot, r.controller.reason)
    self.assertIn(0., out, "the lead-in sends zero")
    self.assertNotEqual(out[-1], .02, "lane keeping's command is replaced during the shot")
    self.assertLessEqual(abs(out[-1]), op.MAX_TORQUE)
    self.assertAlmostEqual(r.width, 3.7, places=5)

  def test_lane_pose_is_smoothed_and_restarts_after_a_gap(self):
    from types import SimpleNamespace as NS
    from unittest.mock import patch
    cs = NS(vEgo=27., steeringAngleDeg=0., steeringRateDeg=0., steeringTorque=0., leftBlinker=False, rightBlinker=False, brakePressed=False)
    md = NS(laneLineProbs=[1.] * 4, meta=NS(laneChangeState=0), laneLines=[NS(y=[y]) for y in (-5.5, -1.85, 1.85, 5.5)],
            orientationRate=NS(z=[0.] * 4, t=[0., 1., 2., 3.]), velocity=NS(x=[27.] * 4), action=NS(desiredCurvature=0.))
    r = runtime()
    poses = iter([(0., 0.), (.2, .01), (.2, .01), (.2, .01)])
    with patch('openpilot.sunnypilot.selfdrive.controls.lib.odyssey_lane.lane_pose', side_effect=lambda _: next(poses)):
      r.update(10., .0, cs, md, True, True, True, 0., 0.)
      r.update(10.05, .0, cs, md, True, True, True, 0., 0.)
      a = .05 / (op.POSE_SMOOTH_S + .05)
      self.assertEqual(r.pose[0], .2, "the offset is used as measured")
      self.assertAlmostEqual(r.pose[1], .01 * a, msg="the heading is low-passed")
      r.update(10.05 + op.MAX_MODEL_AGE + .1, .0, cs, md, True, True, True, 0., 0.)
      self.assertEqual(r.pose, (.2, .01))
    r = runtime()
    with patch('openpilot.sunnypilot.selfdrive.controls.lib.odyssey_lane.lane_pose', return_value=None):
      for n in range(3):
        r.update(20. + n * DT, .0, cs, md, True, True, True, 0., 0.)
      self.assertEqual(r.controller.reason, 'lane lines lost', "no usable lines is said in words the driver can act on")
      self.assertFalse(r.controller.broken)

  def test_queue_overflow_disables_experiment_without_losing_abort(self):
    from types import SimpleNamespace as NS
    p, t = started()
    r = runtime(p, queue_size=1)
    r.queue.put_nowait(('occupied', {}))
    r.pose, r.model_time, r.curve, r.width = (0., 0.), t, 0., 3.7
    cs = NS(vEgo=25., steeringAngleDeg=0., steeringRateDeg=0., steeringTorque=0.,
            leftBlinker=False, rightBlinker=False, brakePressed=False)
    md = NS(laneLineProbs=[1.] * 4, meta=NS(laneChangeState=0))
    self.assertEqual(r.update(t + DT, .17, cs, md, True, False, True, 0., 0.), .17)
    self.assertTrue(r.controller.broken)
    self.assertFalse(r.controller.in_shot)
    self.assertEqual(r.controller.events[-1]['abort'], 'runtime fault')
