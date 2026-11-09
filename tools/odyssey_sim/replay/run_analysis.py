"""Persist reproducible replay arrays; run from the repository root."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from tools.odyssey_sim.replay.harness import ControllerConfig, replay_configs, metadata


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('route')
  parser.add_argument('--output', required=True)
  parser.add_argument('--max-frames', type=int)
  args = parser.parse_args()
  configs = [ControllerConfig(n, d) for d in (None, .32, .45) for n in (True, False)]
  rows = []
  start = time.monotonic()
  for row in replay_configs(args.route, configs, args.max_frames):
    rows.append(row)
    if len(rows) % 30000 == 0:
      print(args.route, len(rows), round(time.monotonic()-start, 1), flush=True)
  keys = list(rows[0])
  np.savez_compressed(args.output, **{k: np.array([r[k] for r in rows]) for k in keys})
  meta = metadata(args.route)
  Path(args.output + '.json').write_text(json.dumps({'route': args.route, 'commit': meta['initData'].gitCommit,
    'params': {k: v.decode(errors='replace') for k,v in meta['params'].items() if any(s in k for s in
      ['Neural', 'Lagd', 'Deadband', 'TorqueControl'])}}, indent=2))
  print('SAVED', args.output, len(rows), flush=True)


if __name__ == '__main__':
  main()
