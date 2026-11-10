"""Road A/B scoring preserves clock eligibility and the registered metric definitions."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.odyssey_wander import FS, ab_report, ab_split, shake, stretches
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import NNLC_MODEL_ARMS


def route(blocks=6):
  t = np.arange(blocks * 180 * int(FS)) / FS
  block = (t // 180).astype(int)
  arm = block % 3
  phase = t % 180
  sign = np.where(block % 2, -1, 1)
  width = np.choose(arm, [0.2, 0.3, 0.1])
  # Constant lanes give signed outside offsets; both left and right curves occur.
  lanes = np.stack([np.broadcast_to((sign * width - 1.8)[:, None], (len(t), 33)),
                    np.broadcast_to((sign * width + 1.8)[:, None], (len(t), 33))], axis=1)
  des = np.where((phase >= 70) & (phase < 90), sign, 0.0)
  d = {k: np.zeros(len(t)) for k in ('seg', 'pressed', 'blinker')}
  d.update(t=t, v=np.full(len(t), 20.0), lat_active=np.ones(len(t)), des_la=des, act_la=des,
           body=(arm + 1) * phase, lanes=lanes, l_y=lanes[:, 0, 0], r_y=lanes[:, 1, 0],
           l_p=np.ones(len(t)), r_p=np.ones(len(t)))
  return d


def report(d):
  output = io.StringIO()
  with contextlib.redirect_stdout(output):
    ab_report(d, stretches(d, 25, 45), 45, NNLC_MODEL_ARMS, 180)
  return output.getvalue()


class TestOdysseyWanderAB(unittest.TestCase):
  def test_shake_counts_time_with_the_wheel_pressed(self):
    d = route()
    arm = (d['t'] // 180).astype(int) % 3
    amplitude = np.choose(arm, [0.02, 0.05, 0.12])  # the third arm shakes the van at 1 Hz
    d['body'] = amplitude * np.sin(2 * np.pi * 1.0 * d['t'])
    d['pressed'] = (arm == 2).astype(float)  # and trips the press sensor the whole time
    got = shake(d, NNLC_MODEL_ARMS, 180, 25)
    self.assertEqual([len(got[a]) for a in NNLC_MODEL_ARMS], [2, 2, 2])
    for name, amp in zip(NNLC_MODEL_ARMS, (0.02, 0.05, 0.12), strict=True):
      np.testing.assert_allclose(got[name], amp / np.sqrt(2), rtol=0.05)
    self.assertEqual(ab_split(d, stretches(d, 25, 45), 45, NNLC_MODEL_ARMS, 180, 30.0)[-1][0], 'S3', "wander never scores the shaking arm")

  def test_clock_settling_and_autonomous_selection(self):
    d = route()
    d['pressed'][(d['t'] >= 235) & (d['t'] < 240)] = 1
    d['blinker'][(d['t'] >= 600) & (d['t'] < 605)] = 1
    d['lat_active'][(d['t'] >= 800) & (d['t'] < 805)] = 0
    pieces = ab_split(d, stretches(d, 25, 45), 45, NNLC_MODEL_ARMS, 180, 30)
    self.assertEqual([a for a, _ in pieces], ['stock', 'S3', 'S5', 'stock', 'S3', 'S3', 'S5'])
    for arm, r in pieces:
      self.assertTrue(np.all(d['t'][r] % 180 >= 30))
      self.assertTrue(np.all((d['t'][r] // 180).astype(int) % 3 == NNLC_MODEL_ARMS.index(arm)))
      self.assertTrue(np.all(d['pressed'][r] + d['blinker'][r] == 0))
      self.assertTrue(np.all(d['lat_active'][r] == 1))

  def test_known_signed_width_and_body_jerk_differences(self):
    text = report(route())
    self.assertIn('S3 - stock curve_run_wide_m: +0.100000 [+0.100000, +0.100000] m', text)
    self.assertIn('S5 - stock curve_run_wide_m: -0.100000 [-0.100000, -0.100000] m', text)
    self.assertIn('S3 - stock body_jerk_rms_mps3: +1.000000 [+1.000000, +1.000000] m/s3', text)
    self.assertIn('S5 - stock body_jerk_rms_mps3: +2.000000 [+2.000000, +2.000000] m/s3', text)

  def test_locked_primary_predictions_load_and_print(self):
    text = report(route())
    self.assertIn('Locked forecast v2', text)
    self.assertIn('predicted +0.100516 [-0.449826, +0.709858] cm RMS', text)
    self.assertIn('predicted -0.032979 [-0.071936, +0.005977] m', text)
    self.assertIn('predicted +0.036255 [+0.026273, +0.047385] m/s3', text)
    self.assertIn('inside predicted interval: yes', text)
    self.assertIn('inside predicted interval: no', text)
    self.assertIn('not calibrated prediction intervals', text)

  def test_no_lane_visibility_excludes_lane_scoring(self):
    d = route()
    d['l_p'][:] = 0.1
    text = report(d)
    self.assertIn('stock curve_run_wide_m: unavailable m (0 blocks)', text)
    self.assertIn('S3 - stock curve_run_wide_m: unavailable [CI unavailable] m;', text)
    self.assertIn('inside predicted interval: unavailable', text)
    self.assertIn('S3 - stock body_jerk_rms_mps3: unavailable', text)

  def test_single_blocks_have_difference_but_no_interval(self):
    text = report(route(3))
    self.assertIn('S3 - stock curve_run_wide_m: +0.100000 [CI unavailable] m', text)
    self.assertIn('S5 - stock body_jerk_rms_mps3: +2.000000 [CI unavailable] m/s3', text)

  def test_multiple_pieces_do_not_become_independent_blocks(self):
    d = route(3)
    # Two >=45s eligible pieces in each block, both past settling.
    d['pressed'][(d['t'] % 180 >= 100) & (d['t'] % 180 < 101)] = 1
    text = report(d)
    self.assertIn('stock body_jerk_rms_mps3: +1.000000 m/s3 (1 blocks)', text)
    self.assertIn('S3 - stock body_jerk_rms_mps3: +1.000000 [CI unavailable]', text)

  def test_empty_route_selection_is_unavailable(self):
    d = route(1)
    d['lat_active'][:] = 0
    text = report(d)
    self.assertIn('stock body_jerk_rms_mps3: unavailable m/s3 (0 blocks)', text)
    self.assertIn('inside predicted interval: unavailable', text)

  def test_block_interval_resamples_real_variation(self):
    from tools.odyssey_wander import block_difference

    delta, interval = block_difference(np.array([1., 3.]), np.array([0., 2.]))
    self.assertEqual(delta, 1)
    np.testing.assert_array_equal(interval, [-1, 3])

  def test_missing_body_does_not_silently_score_surviving_piece(self):
    d = route(3)
    d['pressed'][(d['t'] % 180 >= 100) & (d['t'] % 180 < 101)] = 1
    d['body'][d['t'] % 180 > 110] = np.nan
    text = report(d)
    self.assertIn('stock body_jerk_rms_mps3: unavailable m/s3 (0 blocks)', text)

  def pooled(self, routes, name, force=False):
    from tools.odyssey_wander import ab_report_routes
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
      ab_report_routes([f'/x/drive{i}' for i in range(len(routes))], routes, name, 25, 45, force)
    return output.getvalue()

  def test_drive_without_the_test_is_skipped_unless_forced(self):
    d = dict(route(), ab_params={'NnlcModelAB': True})
    text = self.pooled([d], 'lane')
    self.assertIn('drive0: skipped, OdysseyLaneCenterAB was not on (on: nnlc_model)', text)
    self.assertIn('off:  0.0 min, 0 blocks', text)
    self.assertIn('null test', self.pooled([d], 'lane', force=True))

  def test_drives_pool_and_blocks_stay_separate(self):
    a = dict(route(2), ab_params={'OdysseyLaneCenterAB': True})
    b = dict(route(2), ab_params={'OdysseyLaneCenterAB': True})  # same clock times as a: must not merge into its blocks
    text = self.pooled([a, b], 'lane')
    self.assertIn('off:  5.0 min, 2 blocks', text)
    self.assertIn(' on:  5.0 min, 2 blocks', text)
    self.assertIn('on - off wander:', text)

  def test_model_forecast_uses_lane_off_blocks_when_both_tests_ran(self):
    d = dict(route(6), ab_params={'NnlcModelAB': True, 'OdysseyLaneCenterAB': True})
    text = self.pooled([d], 'nnlc_model')
    self.assertIn('kept 3 of 6 pieces, those with the lane correction off', text)
    self.assertIn('stock body_jerk_rms_mps3: +1.000000 m/s3 (1 blocks)', text)
    text = self.pooled([d], 'lane')
    self.assertIn('pieces per model off/on: stock 1/1, S3 1/1, S5 1/1', text)

  def test_driver_presses_count_once_each_per_arm(self):
    d = dict(route(2), ab_params={'OdysseyLaneCenterAB': True})
    for start in (60, 100, 250):  # two presses in the off block, one in the on block
      d['pressed'][(d['t'] >= start) & (d['t'] < start + 2)] = 1
    d['pressed'][(d['t'] >= 10) & (d['t'] < 12)] = 1  # inside the settling time: not counted
    text = self.pooled([d], 'lane')
    self.assertIn('off 8.0 (2 in 2 min), on 4.0 (1 in 2 min)', text)

  def test_sway_measures_body_motion_without_lane_lines(self):
    from tools.odyssey_wander import analyze
    d = route(1)
    r = np.arange(len(d['t']))
    d['body'] = 0.2 * np.sin(2 * np.pi * d['t'] / 12)  # a 12 s swing of 0.2 m/s2
    d['l_p'][:] = 0.0
    x = analyze(d, r)
    self.assertTrue(np.isnan(x['wander']))
    self.assertAlmostEqual(x['sway'], 0.2 / np.sqrt(2), places=2)

  def test_load_reads_body_from_speed_times_logged_yaw(self):
    from openpilot.cereal import messaging
    from openpilot.tools.lib.logreader import save_log
    from tools.odyssey_wander import load

    messages = []
    for i, (speed, yaw) in enumerate(((20., 0.1), (10., -0.2), (20., None))):
      control = messaging.new_message('carControl')
      control.logMonoTime = 1_000_000_000 + i * 50_000_000
      control.carControl.latActive = True
      control.carControl.angularVelocity = [9., 8., yaw] if yaw is not None else []
      state = messaging.new_message('carState')
      state.logMonoTime = control.logMonoTime + 1
      state.carState.vEgo = speed
      messages.extend((control.as_reader(), state.as_reader()))
    with tempfile.TemporaryDirectory() as tmp:
      segment = Path(tmp) / 'route--0'
      segment.mkdir()
      save_log(str(segment / 'rlog.zst'), messages)
      d = load(tmp)
    np.testing.assert_allclose(d['body'][:2], [2., -2.])
    self.assertTrue(np.isnan(d['body'][2]))
    self.assertEqual(d['ab_params'], {})
