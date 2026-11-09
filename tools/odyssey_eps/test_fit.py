import json
import unittest
import numpy as np

from tools.odyssey_eps.fit import fit_map, TORQUE_BP
from opendbc.sunnypilot.car.honda.eps_mod import OdysseyEpsMap


class TestFit(unittest.TestCase):
  def test_recovers_the_table_inverse(self):
    # A plant built the way the EPS works: the table output drives a convex assist curve. The fit should then land on
    # the same commands as the default map, which inverts the table directly.
    from opendbc.sunnypilot.car.honda.eps_mod import ODYSSEY_FIRMWARE_OUT, ODYSSEY_STOCK_OUT, ODYSSEY_TABLE_IN
    for scale in (1.5, 2.0):
      def motor(command, out):
        return float(75 * np.interp(command, ODYSSEY_TABLE_IN, out) ** 3)
      stock = [[{'n': 5, 'motor': motor(t, ODYSSEY_STOCK_OUT)} for t in TORQUE_BP]] * 2
      modified = [[{'n': 5, 'motor': motor(t, ODYSSEY_FIRMWARE_OUT[scale])} for t in TORQUE_BP]] * 2
      fitted = OdysseyEpsMap(json.dumps(fit_map(stock, modified, scale)))
      default = OdysseyEpsMap(json.dumps({'firmware_scale': scale}))
      for t in TORQUE_BP:
        # within 5% of full scale: the fit joins nine measured command levels with straight lines
        self.assertLessEqual(abs(fitted.command(t, 5) - default.command(t, 5)), 0.05 * 4096)
        self.assertEqual(fitted.command(-t, 5), -fitted.command(t, 5))

  def test_refuses_a_response_the_tables_cannot_explain(self):
    stock = [[{'n': 5, 'motor': float(t * 75)} for t in TORQUE_BP]] * 2
    modified = [[{'n': 5, 'motor': float(t * 187.5)} for t in TORQUE_BP]] * 2  # linear in command: not how the EPS works
    with self.assertRaisesRegex(ValueError, 'insufficient'):
      fit_map(stock, modified, 2.5)

  def test_sparse_refuses(self):
    with self.assertRaisesRegex(ValueError, 'insufficient'):
      fit_map([], [], 2.5)
