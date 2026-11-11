"""Source-rate history preserves short pulses independently of the render rate."""
import unittest

from openpilot.selfdrive.ui.sunnypilot.onroad.odyssey_strip_chart import StripHistory, FineAngle, scaled


class TestStripHistory(unittest.TestCase):
  def test_short_pulse_survives_wrap_and_time_window(self):
    history = StripHistory(300)
    for i in range(400):
      history.append(i * .01, .08 if 350 <= i < 355 else 0., 1.)
    rows = list(history.recent(3.99, 2.))
    self.assertEqual(len(rows), 201)
    self.assertEqual(sum(torque > 0 for _, torque, _ in rows), 5)
    self.assertAlmostEqual(rows[0][0], 3.99)
    self.assertAlmostEqual(rows[-1][0], 1.99)
    history.append(3.985, 1., 2.)
    self.assertEqual(len(list(history.recent(3.99, 2.))), 201)
    # A replay seek starts a fresh history, rather than joining unrelated times.
    history.append(1., -.02, 0.)
    self.assertEqual(list(history.recent(1., 2.)), [(1., -.02, 0.)])

  def test_fine_angle_integrates_and_stays_in_sensor_bin(self):
    angle = FineAngle()
    self.assertEqual(angle.update(1., 2., 1.), 2.)
    self.assertAlmostEqual(angle.update(1.01, 2., 1.), 2.01)
    self.assertAlmostEqual(angle.update(1.02, 2., 10.), 2.05)
    self.assertAlmostEqual(angle.update(1.03, 2.1, 0.), 2.05)
    self.assertEqual(angle.update(2., 3., 10.), 3.)
    self.assertEqual(angle.update(2.01, 3.1, None), 3.1)

  def test_signed_scale_and_clipping(self):
    self.assertEqual(scaled(0., .15), 0.)
    self.assertAlmostEqual(scaled(.075, .15), .5)
    self.assertEqual(scaled(-.3, .15), -1.)
    self.assertEqual(scaled(10., 5.), 1.)


class TestStripSources(unittest.TestCase):
  @staticmethod
  def message(service, t):
    from openpilot.cereal import messaging
    m = messaging.new_message(service)
    m.logMonoTime = int(t * 1e9)
    m.valid = True
    return m

  def test_sent_torque_wins_and_live_replay_clock_does_not_clear_history(self):
    from openpilot.selfdrive.ui.sunnypilot.onroad.odyssey_strip_chart import OdysseyStripChart
    chart = OdysseyStripChart()
    for i in range(100):
      t = 10 + i * .01
      output = self.message('carOutput', t)
      output.carOutput.actuatorsOutput.torque = .08 if 50 <= i < 55 else 0.
      chart.ingest(output)
      state = self.message('carState', t)
      state.carState.steeringAngleDeg = 1.
      chart.ingest(state)
      control = self.message('carControl', t + .001)
      control.carControl.actuators.torque = .5
      chart.ingest(control)
      other_clock = self.message('carControl', t + 1000)
      chart.ingest(other_clock)
    rows = list(chart.history.recent(10.99))
    self.assertEqual(sum(.079 < torque < .081 for _, torque, _ in rows), 10)  # state + output, five source ticks
    self.assertTrue(all(torque < .1 for _, torque, _ in rows))
    self.assertTrue(chart.output_source)
    fallback = self.message('carControl', 11.3)
    fallback.carControl.actuators.torque = -.04
    chart.ingest(fallback)
    self.assertAlmostEqual(chart.torque, -.04)
    self.assertFalse(chart.output_source)

  def test_honda_status_rate_refines_angle_without_new_message_fields(self):
    from openpilot.cereal import messaging
    from opendbc.can.packer import CANPacker
    from opendbc.can.parser import CANParser
    from openpilot.selfdrive.ui.sunnypilot.onroad.odyssey_strip_chart import OdysseyStripChart
    chart = OdysseyStripChart()
    chart.parser = CANParser('honda_odyssey_exl_2018_generated', [('STEER_STATUS', 0)], 0)
    packer = CANPacker('honda_odyssey_exl_2018_generated')
    for i in range(3):
      t = 10. + i * .01
      can = messaging.new_message('can', 1)
      can.valid, can.logMonoTime = True, int(t * 1e9)
      address, data, src = packer.make_can_msg('STEER_STATUS', 0, {'STEER_ANGLE_RATE': 1.})
      can.can[0].address, can.can[0].dat, can.can[0].src = address, data, src
      chart.ingest(can)
      state = self.message('carState', t)
      state.carState.steeringAngleDeg = 2.
      chart.ingest(state)
    self.assertTrue(chart.fine)
    self.assertAlmostEqual(chart.angle, 2.02)
    chart.ingest(self.message('carState', 10.2))
    self.assertFalse(chart.fine)
    self.assertEqual(chart.angle, 0.)
