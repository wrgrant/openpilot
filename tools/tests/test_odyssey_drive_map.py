import unittest
from tools.odyssey_drive_map import road_matched, render


class TestRoadMatched(unittest.TestCase):
  def test_paired_difference_and_null(self):
    points = [{'lat': 40+i*.001, 'lon': -94, 'heading': 0, 'arm': a, 'good': True, 'push': float(i+delta),
                   'route': f'test{i}', 't': i, 'speed': 30, 'reason': 'counting', 'kind': 'straight', 'changes': .5}
              for i in range(10) for a, delta in [('off', 0), ('on', 2)]]
    result = road_matched(points)[0]
    self.assertEqual(result['cells'], 10)
    self.assertEqual(result['difference_N'], 2)
    for p in points:
      p['push'] = 4.
    self.assertEqual(road_matched(points)[0]['interval_N'], [0., 0.])
    self.assertIn('const pts=[', render(points))
