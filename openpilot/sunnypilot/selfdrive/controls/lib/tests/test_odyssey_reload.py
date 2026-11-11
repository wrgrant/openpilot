import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_reload import ReloadManager


class TestReload(unittest.TestCase):
  def test_no_stamp_can_bypass_disengagement_or_pulse_gate(self):
    with tempfile.TemporaryDirectory() as d:
      root = Path(d)
      owner = SimpleNamespace(odyssey_pulse=SimpleNamespace(controller=SimpleNamespace(in_shot=True)))
      replacement = object()
      reports = []
      manager = ReloadManager(owner, root, prepare=lambda stamp: replacement, report=reports.append)
      manager.start()
      try:
        (root / 'request').write_text('abc:now')
        time.sleep(.15)
        self.assertFalse(manager.poll(True))
        self.assertFalse(manager.poll(False))
        owner.odyssey_pulse.controller.in_shot = False
        self.assertFalse(manager.poll(True))
        self.assertTrue(manager.poll(False))
        self.assertIs(manager.current, replacement)
        time.sleep(.15)
        self.assertEqual(reports[-1]['status'], 'ok')
      finally:
        manager.close()

  def test_error_retains_generation(self):
    with tempfile.TemporaryDirectory() as d:
      reports = []
      def fail(stamp):
        raise ValueError('bad module')
      manager = ReloadManager(SimpleNamespace(odyssey_pulse=None), Path(d), prepare=fail, report=reports.append)
      manager.start()
      try:
        (Path(d) / 'request').write_text('bad')
        time.sleep(.2)
        self.assertFalse(manager.poll(False))
        self.assertIsNone(manager.current)
        self.assertEqual(reports[-1]['status'], 'error')
      finally:
        manager.close()

  def test_default_waits_for_lateral_inactive(self):
    with tempfile.TemporaryDirectory() as d:
      manager = ReloadManager(SimpleNamespace(odyssey_pulse=None), Path(d), prepare=lambda stamp: object(), report=lambda result: None)
      manager.start()
      try:
        (Path(d) / 'request').write_text('wait')
        time.sleep(.15)
        self.assertFalse(manager.poll(True))
        self.assertTrue(manager.poll(False))
      finally:
        manager.close()

  def test_real_generation_and_state_handoff(self):
    from openpilot.common.params import Params
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import LiveAB
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_pulse import PulseRuntime
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_reload import Bundle
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_steer import OdysseySteer
    with tempfile.TemporaryDirectory() as d:
      params = Params(d)
      params.put('OdysseyABTotals', '{"lane":{"off":{"straight":12,"curve":3}}}', block=True)
      ab, pulse = LiveAB(params), PulseRuntime(params)
      owner = SimpleNamespace(params=params, odyssey_ab=ab, odyssey_pulse=pulse, odyssey_steer=OdysseySteer(params))
      bundle = None
      try:
        time.sleep(.15)
        params.put('OdysseyABExperiment', 'pulse')
        params.put_bool('OdysseyPulsePaused', False)
        time.sleep(.15)
        pulse.controller.done['25-45'] = {1, 2}
        pulse.controller.previous_command = .07
        ab.switched = 123.
        bundle = Bundle(owner)
        old_threads = (ab.thread, pulse.worker)
        old = bundle.swap(owner)
        bundle.retire(old)
        self.assertEqual(owner.odyssey_pulse.controller.done['25-45'], {1, 2})
        self.assertEqual(owner.odyssey_ab.switched, 123.)
        self.assertEqual(owner.odyssey_pulse.controller.previous_command, .07)
        self.assertFalse(params.get_bool('OdysseyPulsePaused'))
        import json
        self.assertEqual(json.loads(params.get('OdysseyABTotals'))['lane']['off']['straight'], 12)
        self.assertIsNot(owner.odyssey_pulse.__class__, PulseRuntime)
        self.assertTrue(owner.odyssey_pulse.worker.is_alive())
        self.assertTrue(all(not thread.is_alive() for thread in old_threads))
      finally:
        owner.odyssey_ab.close()
        owner.odyssey_pulse.close()
        if bundle:
          bundle.generation.close()

  def test_bad_generation_does_not_mutate_running_module(self):
    from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_lane
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_reload import Generation
    original = odyssey_lane.K_OFFSET
    generation = Generation()
    generation.sources['odyssey_lane'] = b'K_OFFSET = 999\nraise ValueError("broken source")\n'
    with self.assertRaisesRegex(ValueError, 'broken source'):
      generation.load()
    self.assertEqual(odyssey_lane.K_OFFSET, original)

  def test_new_generation_changes_behavior(self):
    from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_lane
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_reload import Generation
    generation = Generation()
    generation.sources['odyssey_lane'] = generation.sources['odyssey_lane'].replace(b'K_OFFSET = 0.3', b'K_OFFSET = 0.1')
    try:
      modules = generation.load()
      self.assertAlmostEqual(modules['odyssey_lane'].correction(.2, 0., 25.), -.02)
      self.assertAlmostEqual(odyssey_lane.correction(.2, 0., 25.), -.06)
    finally:
      generation.close()

  def test_external_helper_uses_active_generation(self):
    import sys
    from types import ModuleType
    from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_lane
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_reload import Generation
    external = ModuleType('openpilot._odyssey_reload_test_caller')
    external.correction = odyssey_lane.correction
    sys.modules[external.__name__] = external
    manager = ReloadManager(SimpleNamespace(odyssey_pulse=None))
    generation = Generation()
    generation.sources['odyssey_lane'] = generation.sources['odyssey_lane'].replace(b'K_OFFSET = 0.3', b'K_OFFSET = 0.1')
    try:
      self.assertAlmostEqual(external.correction(.2, 0., 25.), -.06)
      manager.modules = generation.load()
      self.assertAlmostEqual(external.correction(.2, 0., 25.), -.02)
    finally:
      generation.close()
      del sys.modules[external.__name__]
