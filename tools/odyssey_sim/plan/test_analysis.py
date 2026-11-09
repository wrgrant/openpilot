from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.plan.sign_audit import audit


class TestInterpretation(OpenpilotTestCase):
  def test_minus_120_degrees_can_be_stable_damping(self):
    # At w=1, a=-e-sqrt(3)*e_dot has gain 2, phase -120 degrees,
    # and poles (-sqrt(3) +/- i)/2: stable, despite the raw negative phase.
    r = audit([{'period': 6.283185307179586, 'plan_accel_per_offset': {'gain': 2, 'phase_deg': -120}}])
    self.assertAlmostEqual(r['k_per_s2'], 1)
    self.assertAlmostEqual(r['c_per_s'], 1.7320508075688772)
    self.assertAlmostEqual(r['phase_relative_to_restoring_deg'][0], 60)
    self.assertTrue(all(p[0] < 0 for p in r['ideal_poles']))

  def test_transported_pose_uses_shared_lane_and_preserves_missing_visibility(self):
    import numpy as np
    from tools.odyssey_sim.plan.summarize import transported
    rows = []
    for t in np.arange(0, 60, .05):
      e = .1*np.sin(2*np.pi*t/10)
      rows.append({'t': t, 'logged_lane': [[-e]*33, [-e]*33], 'dy': -e})
    self.assertAlmostEqual(transported(rows, np.ones(len(rows), bool)), 0)
    self.assertIsNone(transported(rows, np.zeros(len(rows), bool)))
