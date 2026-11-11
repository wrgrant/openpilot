import json
import unittest
from unittest.mock import patch

from tools.odyssey_live_push import push, Remote


class TestLivePush(unittest.TestCase):
  def test_refuses_out_of_scope_differences_without_upload(self):
    class Device:
      calls = 0
      def python(self, script, data=None):
        self.calls += 1
        return json.dumps({'head': 'abc', 'dirty': ['openpilot/common/params_keys.h'], 'hashes': {}})
    device = Device()
    with patch('tools.odyssey_live_push.git', return_value=''):
      with self.assertRaisesRegex(ValueError, 'params_keys.h'):
        push(device)
    self.assertEqual(device.calls, 1)

  def test_refuses_shell_host(self):
    with self.assertRaises(ValueError):
      Remote('comma;reboot', '~/.ssh/id_ed25519_comma')

  def test_uploads_only_changed_scoped_file_and_matches_ack(self):
    import hashlib
    from tools.odyssey_live_push import FOLDER, ROOT, scope
    allowed = {str(FOLDER / f'{name}.py') for name in scope()}
    hashes = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in allowed}
    changed = str(FOLDER / 'odyssey_lane.py')
    hashes[changed] = 'old'
    class Device:
      payload = None
      def python(self, script, data=None):
        if self.payload is not None:
          return json.dumps({'stamp': self.payload['stamp'], 'status': 'ok'})
        if isinstance(data, dict):
          self.payload = data
          return ''
        return json.dumps({'head': 'abc', 'dirty': [], 'hashes': hashes})
    device = Device()
    with patch('tools.odyssey_live_push.git', side_effect=lambda *args: 'abcdef' if args[0] == 'rev-parse' else ''):
      self.assertTrue(push(device))
    self.assertEqual(set(device.payload['files']), {changed})
    self.assertFalse(device.payload['stamp'].endswith(':now'))
    self.assertEqual(device.payload['source']['hashes'][changed], hashlib.sha256((ROOT/changed).read_bytes()).hexdigest())
