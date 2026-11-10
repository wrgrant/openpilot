#!/usr/bin/env python3
"""Follow a drive's pulse-test shots as its log segments close. All remote I/O is confined to Device.

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_drive_companion.py --host comma --out ~/comma-logs
  ... --host comma@100.66.59.37 --identity ~/.ssh/id_ed25519_comma --out ~/comma-logs   (over Tailscale, during a drive)
  ... --local /path/to/recorded/routes --out /tmp/local-companion

Start it before or during the drive: it ignores drives that ended before it started (see --since-minutes),
waits --wait-minutes for one to begin, follows a new route if the van is restarted, and stores logs in the usual
<out>/<dongle>_<route>/<segment>/rlog.zst layout.

Local mode releases one closed segment per snapshot; optional local scenario.json:
{"statuses": [{...}, ...], "drop_at": [2]} (one-based snapshot calls).
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time

from tools.odyssey_eps_report import analyze, note, report, suggest, summarize, write_summary

PARAMS = {'OdysseyAgentNote': '/dev/shm/params/d/OdysseyAgentNote',
          'OdysseyPulseQueue': '/data/params/d/OdysseyPulseQueue',
          'OdysseyPulsePaused': '/data/params/d/OdysseyPulsePaused'}


def validate_write(name, value):
  if name not in PARAMS:
    raise ValueError('Param is not companion-writable')
  if name == 'OdysseyPulsePaused' and value != '1':
    raise ValueError('Companion may only pause')
  if name == 'OdysseyAgentNote' and (len(value.splitlines()) > 2 or any(len(line) > 34 for line in value.splitlines())):
    raise ValueError('Note exceeds panel bounds')
  if name == 'OdysseyPulseQueue':
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_pulse import Condition, HARD_SHOT_S, LEAD_S, TAIL_S
    conditions = json.loads(value)
    if not isinstance(conditions, list) or len(conditions) > 6:
      raise ValueError('At most six queued shots')
    for c in conditions:
      shot = Condition(**c)
      if (shot.kind not in ('single', 'pair', 'train', 'sham', 'chirp') or not 0 <= shot.height <= .12 or
          not 0 < shot.duration < HARD_SHOT_S-LEAD_S-TAIL_S or shot.direction not in (-1, 1) or
          not 0 <= shot.duty <= 1 or shot.period < 0):
        raise ValueError('Unsafe queued condition')
      if shot.kind == 'pair' and (shot.period <= 0 or abs(shot.duration-(2*shot.period+.5)) > 1e-6):
        raise ValueError('Invalid pair timing')
      if shot.kind == 'train' and not 0 < shot.period <= shot.duration:
        raise ValueError('Invalid train period')


class Device:
  """Only remote boundary: bounded ssh/rsync, atomic allowlisted param writes."""
  def __init__(self, host, identity=None):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.@-]*', host):
      raise ValueError('Invalid ssh host')
    self.host = host
    # one shared connection for the polls, the pulls and the writes: each would otherwise pay a handshake over the hotspot
    self.ssh = ['ssh', '-oBatchMode=yes', '-oConnectTimeout=5', '-oServerAliveInterval=3', '-oServerAliveCountMax=2',
                '-oControlMaster=auto', '-oControlPersist=120', '-oControlPath=' + str(Path.home()/'.ssh'/'cm-companion-%C')]
    if identity:
      self.ssh += ['-i', str(Path(identity).expanduser())]

  def _remote(self, script, data=None):
    try:
      return subprocess.run([*self.ssh, self.host, 'python3 -c '+shlex.quote(script)], input=data,
                            capture_output=True, text=True, timeout=15, check=True).stdout
    except (subprocess.SubprocessError, OSError) as e:
      raise ConnectionError(f'ssh failed: {type(e).__name__}') from e

  def snapshot(self, since=None):
    """Routes with a segment written at or after `since` (device clock, seconds; None = every route on the device),
    their closed segments, the live status and queue, and the device's clock."""
    script = '''import json, time
from pathlib import Path
root = Path('/data/media/0/realdata')
dirs = [p for p in root.iterdir() if p.is_dir() and '--' in p.name]
since = SINCE
recent = sorted((p for p in dirs if since is None or p.stat().st_mtime >= since), key=lambda p: p.stat().st_mtime)
routes = list(dict.fromkeys(p.name.rsplit('--', 1)[0] for p in recent))
segments = [p.name for p in dirs if p.name.rsplit('--', 1)[0] in routes
            and (p/'rlog.zst').exists() and not (p/'rlog.lock').exists()]
def read(path, default):
  try:
    return json.loads(Path(path).read_text())
  except (OSError, ValueError):
    return default
dongle = Path('/data/params/d/DongleId')
print(json.dumps({'route': routes[-1] if routes else None, 'routes': routes, 'segments': segments, 'now': time.time(),
                  'dongle': dongle.read_text().strip() if dongle.exists() else 'unknown',
                  'status': read('/dev/shm/params/d/OdysseyABStatus', {}), 'queue': read('/data/params/d/OdysseyPulseQueue', [])}))
'''.replace('SINCE', repr(since))
    try:
      return json.loads(self._remote(script))
    except (ValueError, TypeError) as e:
      raise ConnectionError('Invalid device snapshot; retrying') from e

  def pull(self, segment, destination):
    if not re.fullmatch(r'[A-Za-z0-9_-]+--[0-9]+', segment):
      raise ValueError('Invalid segment path')
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_suffix('.partial')
    try:
      subprocess.run(['rsync', '-a', '--partial', '--timeout=10', '-e', shlex.join(self.ssh),
                      f'{self.host}:/data/media/0/realdata/{segment}/rlog.zst', str(partial)],
                     capture_output=True, timeout=20, check=True)
      partial.replace(destination)
    except (subprocess.SubprocessError, OSError) as e:
      raise ConnectionError(f'segment pull failed: {type(e).__name__}') from e

  def write(self, name, value):
    validate_write(name, value)
    path = PARAMS[name]
    # Temp file in the same filesystem, then rename: never expose a partial queue.
    script = f'''import os, sys, tempfile
from pathlib import Path
p = Path({path!r})
fd, tmp = tempfile.mkstemp(prefix='.'+p.name, dir=p.parent)
try:
  with os.fdopen(fd, 'w') as f:
    f.write(sys.stdin.read())
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, p)
finally:
  if os.path.exists(tmp): os.unlink(tmp)
'''
    self._remote(script, value)


class LocalDevice:
  """Same boundary, folder backed; deterministic releases and one-shot link failures."""
  def __init__(self, root, statuses=None, drop_at=()):
    self.root = Path(root)
    scenario = self.root/'scenario.json'
    config = json.loads(scenario.read_text()) if scenario.exists() else {}
    self.statuses = statuses if statuses is not None else config.get('statuses', [])
    self.drop_at = set(drop_at or config.get('drop_at', []))
    self.calls, self.released = 0, 0
    self.params, self.notes = {}, []
    self.files = sorted(self.root.glob('**/rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[-1]))
    self.files = [p for p in self.files if not (p.parent/'rlog.lock').exists()]

  def snapshot(self, since=None):
    self.calls += 1
    if self.calls in self.drop_at:
      raise ConnectionError('simulated dropped link')
    self.released = min(self.released+1, len(self.files))
    files = self.files[:self.released]
    routes = list(dict.fromkeys(p.parent.name.rsplit('--', 1)[0] for p in files))
    status = self.statuses[min(self.calls-1, len(self.statuses)-1)] if self.statuses else {}
    return {'route': routes[-1] if routes else None, 'routes': routes, 'segments': [p.parent.name for p in files], 'status': status,
            'queue': json.loads(self.params.get('OdysseyPulseQueue', '[]')), 'now': float(self.calls), 'dongle': 'local'}

  def pull(self, segment, destination):
    source = next(p for p in self.files[:self.released] if p.parent.name == segment)
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)

  def write(self, name, value):
    validate_write(name, value)
    self.params[name] = value
    if name == 'OdysseyAgentNote':
      self.notes.append(value)


def fault(summary, status, previous_aborts, consecutive):
  """Count stop events, not repeated polls of one status reason."""
  for reason, count in status.get('aborts', {}).items():
    delta = max(0, count-previous_aborts.get(reason, 0))
    if delta:
      consecutive = (reason, consecutive[1]+delta if consecutive[0] == reason else delta)
  previous_aborts = dict(status.get('aborts', {}))
  # Exact ordering comes from completed/aborted end records when available.
  ends = summary['shots']
  if ends:
    recent = ends[-5:]
    if len(recent) == 5 and all(not r.get('completed') for r in recent) and len({r.get('abort') for r in recent}) == 1:
      return 'fault pattern: '+recent[0].get('abort', 'unknown'), previous_aborts, consecutive
    if any(r.get('completed') and not r.get('samples') for r in ends):
      return 'fault pattern: completed shot without samples', previous_aborts, consecutive
  if consecutive[1] >= 5:
    return 'fault pattern: '+consecutive[0], previous_aborts, consecutive
  return None, previous_aborts, consecutive


def run(device, out, queue=False, max_minutes=None, poll_seconds=3., analyzer=analyze,
        stream=sys.stdout, clock=time.monotonic, sleep=time.sleep, since_minutes=None, wait_minutes=45.):
  out = Path(out).expanduser()
  out.mkdir(parents=True, exist_ok=True)
  started = last_new = clock()
  route, previous_aborts, consecutive = None, {}, ('', 0)
  since, first, last_note, folders = None, True, None, []
  summary, status = summarize([]), {}
  analyzed = set()
  state_path = out/'companion-state.json'
  state = json.loads(state_path.read_text()) if state_path.exists() else {'issued': []}
  issued = set(state['issued'])
  exit_reason, pause_pending = None, False
  while True:
    if max_minutes is not None and clock()-started >= max_minutes*60:
      exit_reason = exit_reason or 'max minutes reached'
      break
    if not analyzed and clock()-started >= wait_minutes*60:
      exit_reason = exit_reason or 'no drive started'
      break
    if analyzed and clock()-last_new >= 180:
      exit_reason = exit_reason or 'nothing new for three minutes'
      break
    try:
      snapshot = device.snapshot(since)
      if first and since_minutes is not None:
        # only drives with a segment written this recently, by the device's clock
        since, first = snapshot['now']-60*since_minutes, False
        snapshot = device.snapshot(since)
      first = False
      if snapshot['route'] != route and snapshot['route'] is not None:
        if route is not None:
          print(f"new route {snapshot['route']} (the van was restarted)", file=stream, flush=True)
        route = snapshot['route']
      status = snapshot['status']
      new_segments = []
      for segment in sorted(snapshot['segments'], key=lambda s: (s.rsplit('--', 1)[0], int(s.rsplit('--', 1)[-1]))):
        name = segment.rsplit('--', 1)[0]
        if name not in snapshot['routes'] or not re.fullmatch(r'[A-Za-z0-9_-]+', name) or not re.fullmatch(r'[A-Za-z0-9]+', snapshot['dongle']):
          continue
        folder = out/f"{snapshot['dongle']}_{name}"
        destination = folder/segment/'rlog.zst'
        if segment not in analyzed:
          if not destination.exists():
            device.pull(segment, destination)
          new_segments.append(segment)
          if folder not in folders:
            folders.append(folder)
      if new_segments:
        summary = analyzer(folders)
        analyzed.update(new_segments)
        last_new = clock()
        write_summary(summary, out/'companion-summary.json')
        print(f"{clock()-started:.0f}s {new_segments[-1]} {summary['clean']} clean; {note(summary).replace(chr(10), ' / ')}",
              file=stream, flush=True)
      if route and note(summary) != last_note:
        device.write('OdysseyAgentNote', note(summary))
        last_note = note(summary)
      detected, previous_aborts, consecutive = fault(summary, status, previous_aborts, consecutive)
      if detected:
        exit_reason, pause_pending = detected, True
      if pause_pending:
        device.write('OdysseyPulsePaused', '1')
        pause_pending = False
        summary['exit_reason'] = exit_reason
        device.write('OdysseyAgentNote', note(summary))
        break
      # A live 'done' status may precede closure of the current segment. Do not claim
      # those final shots were analyzed: the handoff reports both status and log coverage.
      complete = any(c['good'] == c['total'] for c in summary['coverage'])
      if complete or status.get('state') == 'done':
        exit_reason = 'pass complete'
        break
      if queue and route and not snapshot['queue'] and not status.get('queue', 0):
        candidates = suggest(summary)
        shots = []
        for candidate in candidates:
          key = json.dumps([route, status.get('band'), candidate['condition']], sort_keys=True)
          if key not in issued:
            issued.add(key)
            shots.append(candidate['condition'])
        if shots:
          # Record before write: an ambiguous disconnect must not queue duplicates.
          tmp = state_path.with_suffix('.tmp')
          tmp.write_text(json.dumps({'issued': sorted(issued)}))
          tmp.replace(state_path)
          device.write('OdysseyPulseQueue', json.dumps(shots[:6]))
    except (ConnectionError, OSError, ValueError) as e:
      print(f'link unavailable / read failed: {e}; retrying', file=stream, flush=True)
    sleep(poll_seconds)
  summary.update(exit_reason=exit_reason, live_status=status, pause_pending=pause_pending,
                 recommendation='Lead: review missing evidence; driver controls resuming. No automatic unpause.',
                 route=route, pulled_segments=len(analyzed))
  write_summary(summary, out/'companion-summary.json')
  print(f"Exit: {exit_reason}. Route: {route}. {len(analyzed)} closed segments analyzed. " +
        f"Live progress {status.get('finished', '?')}/{status.get('total', '?')}. Pause pending: {pause_pending}.", file=stream)
  print(report(summary), file=stream, end='')
  print(summary['recommendation'], file=stream, flush=True)
  return summary


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  source = ap.add_mutually_exclusive_group(required=True)
  source.add_argument('--host')
  source.add_argument('--local', type=Path)
  ap.add_argument('--out', type=Path, required=True)
  ap.add_argument('--queue', action='store_true')
  ap.add_argument('--max-minutes', type=float)
  ap.add_argument('--poll-seconds', type=float, default=5.)
  ap.add_argument('--identity', help='ssh identity file for --host')
  ap.add_argument('--since-minutes', type=float, default=1.5,
                  help='follow drives with a log segment written this recently (raise it to try the tool on an earlier drive)')
  ap.add_argument('--wait-minutes', type=float, default=45., help='how long to wait for a drive to begin')
  args = ap.parse_args()
  if args.poll_seconds < 0 or (args.max_minutes is not None and args.max_minutes <= 0):
    ap.error('poll seconds must be nonnegative and max minutes positive')
  device = LocalDevice(args.local) if args.local else Device(args.host, args.identity)
  run(device, args.out, args.queue, args.max_minutes, args.poll_seconds,
      since_minutes=None if args.local else args.since_minutes, wait_minutes=args.wait_minutes)


if __name__ == '__main__':
  main()
