#!/usr/bin/env python3
"""Local real-time replay proof: run real controlsd, change Python mid-minute, restore it on exit.

The recorded drive must disengage and then reengage between 6 and 20 seconds into the segment.
Uses comma's replay binary and local logs, without a vehicle or UI. Checks publication-time frame gaps around
reload against the pulse controller's MAX_FRAME_GAP. This measures the Mac scheduler, not the comma scheduler.
"""
import argparse
import json
import os
from pathlib import Path
import pty
import subprocess
import sys
import tempfile
import threading
import time

from openpilot.cereal import messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_pulse import MAX_FRAME_GAP
from tools.odyssey_sim.replay.ui_check import car_params, drain


def check(rlog):
  segment = Path(rlog).resolve().parent
  route_dir = segment.parent
  dongle, route = route_dir.name.split('_', 1)
  start = 60 * int(segment.name.rsplit('--', 1)[1])
  source = Path(BASEDIR) / 'openpilot/sunnypilot/selfdrive/controls/lib/odyssey_steer.py'
  original = source.read_text()
  sentinel = .123456
  original_return = 'return max(-steer_max, min(steer_max, steer))'
  assert original_return in original
  rows = []
  with tempfile.TemporaryDirectory() as directory, OpenpilotPrefix() as prefix:
    state = Path(directory)
    params = Params()
    cp, sp = car_params(segment)
    params.put('CarParams', cp, block=True)
    params.put('CarParamsSP', sp, block=True)
    for key in ('NeuralNetworkLateralControl', 'LagdToggle'):
      params.put_bool(key, True, block=True)
    env = {**os.environ, 'OPENPILOT_PREFIX': prefix.prefix, 'PYTHONPATH': BASEDIR, 'TERM': 'xterm-256color',
           'ODYSSEY_RELOAD_DIR': directory, 'PATH': str(Path(sys.executable).parent) + os.pathsep + os.environ.get('PATH', '')}
    master, slave = pty.openpty()
    threading.Thread(target=drain, args=(master,), daemon=True).start()
    socket = messaging.sub_sock('carControl', timeout=100)
    with (state/'controlsd.log').open('w') as output:
      processes = [subprocess.Popen([str(Path(BASEDIR)/'openpilot/tools/replay/replay'), f'{dongle}/{route}',
                                     '--data_dir', str(route_dir), '-s', str(start), '--no-vipc',
                                     '-b', 'carControl,controlsState,carControlSP'], env=env, stdin=slave, stdout=slave, stderr=slave),
                   subprocess.Popen([sys.executable, '-m', 'openpilot.selfdrive.controls.controlsd'], env=env,
                                    cwd=BASEDIR, stdout=output, stderr=subprocess.STDOUT)]
      begin, fired = time.monotonic(), False
      try:
        while time.monotonic() - begin < 20:
          if any(p.poll() is not None for p in processes):
            raise RuntimeError((state/'controlsd.log').read_text())
          if time.monotonic() - begin > 6 and not fired:
            source.write_text(original.replace(original_return, f'return {sentinel}'))
            (state/'request').write_text('replay-proof')
            fired = True
          for message in messaging.drain_sock(socket):
            rows.append((message.logMonoTime * 1e-9, message.carControl.actuators.torque, message.carControl.latActive))
        result = json.loads((state/'result').read_text()) if (state/'result').exists() else {}
      finally:
        source.write_text(original)
        for process in processes:
          process.terminate()
        for process in processes:
          try:
            process.wait(timeout=5)
          except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        os.close(slave)
        os.close(master)
  changed = [i for i, row in enumerate(rows) if abs(row[1]-sentinel) < 1e-7 and row[2]]
  # Exclude startup; measure a two-second window spanning the actual behavior change.
  gaps = []
  if changed:
    t = rows[changed[0]][0]
    gaps = [b[0]-a[0] for a, b in zip(rows, rows[1:], strict=False) if t-1 <= a[0] <= t+1]
  maximum = max(gaps, default=float('inf'))
  ok = result.get('status') == 'ok' and len(changed) > 0 and maximum <= MAX_FRAME_GAP
  print(json.dumps({'ok': ok, 'frames': len(rows), 'changed_frames': len(changed), 'max_frame_gap_around_reload': maximum,
                    'limit': MAX_FRAME_GAP, 'reload': result}, indent=2))
  return ok


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('rlog')
  args = parser.parse_args()
  raise SystemExit(0 if check(args.rlog) else 1)
