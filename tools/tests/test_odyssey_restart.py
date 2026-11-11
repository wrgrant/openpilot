import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.system.manager import manager
from tools.odyssey_restart_controls import pid_command


class TestRestart(unittest.TestCase):
  def test_parse_names_logs_unknown_and_deduplicates(self):
    with patch.object(manager.cloudlog, 'event') as event:
      self.assertEqual(manager.parse_restart_processes(' controlsd\nui bogus controlsd\t', {'controlsd': None, 'ui': None}),
                       ['controlsd', 'ui'])
      event.assert_called_once_with('odyssey_restart_unknown', process='bogus')
      self.assertEqual(manager.parse_restart_processes('  ', {}), [])

  def test_both_flags_stop_each_process_once_and_are_removed(self):
    with tempfile.TemporaryDirectory() as tmp:
      flag, legacy = Path(tmp) / 'restart', Path(tmp) / 'legacy'
      flag.write_text('controlsd ui unknown')
      legacy.touch()
      processes = {'controlsd': Mock(), 'ui': Mock()}
      with patch.object(manager, 'RESTART_FLAG', str(flag)), patch.object(manager, 'LEGACY_RESTART_FLAG', str(legacy)), \
           patch.object(manager, 'managed_processes', processes):
        manager.restart_requested_processes()
        manager.restart_requested_processes()
      for process in processes.values():
        process.stop.assert_called_once_with()
      self.assertFalse(flag.exists())
      self.assertFalse(legacy.exists())

  def test_pid_pattern_cannot_match_its_own_command(self):
    for module in ('openpilot.selfdrive.controls.controlsd', 'openpilot.selfdrive.ui.ui'):
      command = pid_command(SimpleNamespace(module=module, name='unused'))
      self.assertIn("grep '[o]", command)
      self.assertNotIn(module, command)

  def test_native_pid_uses_executable_when_managed_name_differs(self):
    command = pid_command(SimpleNamespace(name='modeld_tinygrad', cmdline=['./modeld']))
    self.assertIn("grep '[.]", command)
    self.assertNotIn('modeld_tinygrad', command)
