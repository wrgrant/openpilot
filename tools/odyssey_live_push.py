#!/usr/bin/env python3
"""Atomic, allowlisted Odyssey Python upload. Installation of this bootstrap itself is parked work."""
import argparse
import hashlib
import json
from pathlib import Path
import py_compile
import shlex
import subprocess
import tempfile
import time
import uuid

from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_reload import LIB, scope

ROOT = Path(__file__).resolve().parents[1]
FOLDER = LIB.relative_to(ROOT)


class Remote:
  def __init__(self, host, identity):
    if host.startswith('-') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789@._-:' for c in host):
      raise ValueError('invalid host')
    self.command = ['ssh', '-oBatchMode=yes', '-oConnectTimeout=5', '-i', str(Path(identity).expanduser()), host]

  def python(self, script, data=None):
    return subprocess.run([*self.command, 'python3 -c ' + shlex.quote(script)], input=json.dumps(data) if data is not None else None,
                          text=True, capture_output=True, check=True, timeout=30).stdout


def git(*args):
  return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def push(remote):
  allowed = {str(FOLDER / f'{name}.py') for name in scope()}
  sources = {path: (ROOT/path).read_bytes() for path in sorted(allowed)}
  with tempfile.TemporaryDirectory() as d:
    for i, source in enumerate(sources.values()):
      snapshot = Path(d) / f'{i}.py'
      snapshot.write_bytes(source)
      py_compile.compile(str(snapshot), cfile=str(Path(d) / f'{i}.pyc'), doraise=True)
  inventory = json.loads(remote.python('''
import hashlib, json, pathlib, subprocess
root = pathlib.Path('/data/openpilot')
def git(*args):
  return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()
print(json.dumps({'head': git('rev-parse', 'HEAD'),
 'dirty': git('diff', '--name-only', 'HEAD').splitlines() + git('ls-files', '--others', '--exclude-standard').splitlines(),
 'hashes': {p: hashlib.sha256((root/p).read_bytes()).hexdigest() if (root/p).is_file() else None for p in json.loads(input())}}))
''', sorted(allowed)))
  # Compare both worktrees against the device commit; no assumption that matching branch names mean matching code.
  differences = set(git('diff', '--name-only', inventory['head']).splitlines())
  differences.update(git('ls-files', '--others', '--exclude-standard').splitlines())
  differences.update(inventory['dirty'])
  parked = sorted(differences - allowed)
  if parked:
    raise ValueError('Parked install required for:\n' + '\n'.join(parked))
  hashes = {p: hashlib.sha256(source).hexdigest() for p, source in sources.items()}
  files = {p: sources[p].decode() for p in allowed if inventory['hashes'].get(p) != hashes[p]}
  digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()[:12]
  stamp = f"{git('rev-parse', '--short', 'HEAD')}-{digest}-{uuid.uuid4().hex[:8]}"
  print(f'Uploading {len(files)} files; source {stamp}', flush=True)
  remote.python('''
import fcntl, hashlib, json, os, pathlib, tempfile
payload = json.loads(input())
root = pathlib.Path('/data/openpilot')
state = pathlib.Path('/dev/shm/odyssey_reload')
if not state.is_dir():
  raise RuntimeError('reload bootstrap is not running; perform a parked install')
with (state/'upload.lock').open('w') as lock:
  fcntl.flock(lock, fcntl.LOCK_EX)
  request, result = state/'request', state/'result'
  if request.exists() and (not result.exists() or json.loads(result.read_text()).get('stamp') != request.read_text().strip()
                           or json.loads(result.read_text()).get('status') not in ('ok', 'error')):
    raise RuntimeError('previous reload is still pending')
  staged = []
  try:
    for name, source in payload['files'].items():
      target = root/name
      if target.is_symlink() or target.parent.resolve() != (root/'openpilot/sunnypilot/selfdrive/controls/lib').resolve():
        raise ValueError('invalid upload path')
      compile(source, str(target), 'exec')
      fd, temporary = tempfile.mkstemp(dir=target.parent, prefix='.odyssey-')
      with os.fdopen(fd, 'w') as output:
        output.write(source)
      os.chmod(temporary, target.stat().st_mode if target.exists() else 0o644)
      staged.append((temporary, target))
    for temporary, target in staged:
      os.replace(temporary, target)
    (state/'source.json').write_text(json.dumps(payload['source']))
    temporary = state/'request.tmp'
    temporary.write_text(payload['stamp'])
    temporary.replace(request)
  finally:
    for temporary, target in staged:
      pathlib.Path(temporary).unlink(missing_ok=True)
''', {'files': files, 'stamp': stamp, 'source': {'commit': git('rev-parse', 'HEAD'), 'hashes': hashes}})
  deadline = time.monotonic() + 5
  while time.monotonic() < deadline:
    result = status(remote)
    if result.get('stamp') == stamp and result.get('status') in ('ok', 'error'):
      print(json.dumps(result, indent=2))
      return result['status'] == 'ok'
    time.sleep(.1)
  print('Reload pending: use --status. Activation waits for steering to be disengaged.')
  return False


def status(remote):
  return json.loads(remote.python(f"from pathlib import Path; p=Path({'/dev/shm/odyssey_reload/result'!r}); print(p.read_text() if p.exists() else '{{}}')"))


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--host', default='comma@100.66.59.37')
  parser.add_argument('--identity', default='~/.ssh/id_ed25519_comma')
  parser.add_argument('--status', action='store_true')
  parser.add_argument('--restart-openpilot', action='store_true')
  args = parser.parse_args()
  remote = Remote(args.host, args.identity)
  if args.restart_openpilot:
    print('WARNING: restarting openpilot can cause vehicle faults. Use only while parked with the van off.', flush=True)
    subprocess.run([*remote.command, 'sudo systemctl restart comma'], check=True, timeout=30)
  elif args.status:
    print(json.dumps(status(remote), indent=2))
  else:
    raise SystemExit(0 if push(remote) else 1)


if __name__ == '__main__':
  main()
