#!/usr/bin/env python3
"""Put new Python control code on the comma during a supervised drive by restarting controlsd alone.

Usage: tools/odyssey_restart_controls.py [--host comma@100.66.59.37] [--identity ~/.ssh/id_ed25519_comma] [--commit HEAD]

Pushes the commit to wrgrant/odyssey-agnos20, moves the comma's checkout to it, and asks the manager to restart
controlsd (manager.py, RESTART_FLAG). Everything else keeps running. Steering is off for the few seconds controlsd
takes to start, and openpilot shows a controls alert meanwhile. Refuses commits that change anything a build or a
full restart has to pick up (non-Python files, submodules)."""
import argparse
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FLAG = '/dev/shm/odyssey_restart_controlsd'
PID = "ps -eo pid,args | grep '[s]elfdrive.controls.controlsd' | awk '{print $1}' | head -1"  # [s]: never match this shell


def run(*cmd, **kw):
  return subprocess.run(cmd, check=True, text=True, capture_output=True, **kw).stdout.strip()


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--host', default='comma@100.66.59.37')
  ap.add_argument('--identity', default=os.path.expanduser('~/.ssh/id_ed25519_comma'))
  ap.add_argument('--commit', default='HEAD')
  a = ap.parse_args()
  ssh = ['ssh', '-o', 'ConnectTimeout=10', '-i', a.identity, a.host]

  sha = run('git', '-C', ROOT, 'rev-parse', a.commit)
  device = run(*ssh, 'cd /data/openpilot && git rev-parse HEAD')
  run('git', '-C', ROOT, 'fetch', '-q', 'wrgrant')
  changed = run('git', '-C', ROOT, 'diff', '--name-only', device, sha).split()
  blocked = [f for f in changed if not f.endswith('.py') or f in ('openpilot/system/manager/manager.py',)]
  submodules = run('git', '-C', ROOT, 'diff', '--submodule=short', '--name-only', device, sha, '--', *[
    line.split()[1] for line in run('git', '-C', ROOT, 'config', '-f', '.gitmodules', '--get-regexp', 'path').splitlines()])
  if blocked or submodules:
    sys.exit('Needs a parked install, not a controls restart: ' + ' '.join(blocked + submodules.split()))
  if not changed:
    print('No Python changes; restarting controlsd anyway.')

  env = {**os.environ, 'GIT_LFS_SKIP_SMUDGE': '1'}
  run('git', '-C', ROOT, 'push', '-q', '--no-verify', 'wrgrant', f'{sha}:refs/heads/odyssey-agnos20', env=env)
  before = run(*ssh, PID)
  run(*ssh, f'cd /data/openpilot && git fetch -q origin odyssey-agnos20 && git reset -q --hard {sha} && touch {FLAG}')
  print(f'comma on {sha[:9]} ({len(changed)} Python files changed); restarting controlsd', flush=True)
  for _ in range(40):
    time.sleep(.5)
    now = run(*ssh, PID)
    if now and now != before:
      print(f'controlsd restarted (pid {before or "-"} -> {now})')
      return
  sys.exit('controlsd did not come back within 20 s: check the comma')


if __name__ == '__main__':
  main()
