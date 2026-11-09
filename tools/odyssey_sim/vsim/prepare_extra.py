"""Read stable local unseen-route rlogs into scratch caches, with no fitting."""

import argparse
import json
from pathlib import Path
from tools.odyssey_sim.plant import extract as eps
from tools.odyssey_sim.calib import extract as body
from tools.odyssey_sim.calib3 import extract as geometry


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--scratch', type=Path, required=True)
  args = ap.parse_args()
  root = args.scratch / 'cache'
  for name in ('plant_data', 'calib', 'calib3'):
    (root / name).mkdir(parents=True, exist_ok=True)
  eps.SCRATCH = body.SCRATCH = root
  geometry.OUT = root / 'calib3'
  state_path = root / 'sources.json'
  state = json.loads(state_path.read_text()) if state_path.exists() else {}
  for key in ('0000019f', '000001a0', '000001a2'):
    route = next((Path.home() / 'comma-logs').glob('*_' + key + '--*'), None)
    if route is None:
      print(key, 'not local yet', flush=True)
      continue
    signature = {str(p): [p.stat().st_size, p.stat().st_mtime_ns] for p in route.glob('*/rlog.zst')}
    prior = state.get(key, {})
    if not signature or signature != prior.get('observed'):
      state[key] = {'observed': signature, 'extracted': prior.get('extracted')}
      print(key, 'waiting for rlogs to remain stable across arrival checks', flush=True)
      continue
    if signature != prior.get('extracted'):
      # Only these worker-owned caches are invalidated when incoming rlogs change.
      for path in (root / 'plant_data' / f'{key}.npz', root / 'calib' / f'{key}_body.npz', root / 'calib3' / f'{key}.npz'):
        path.unlink(missing_ok=True)
      eps.extract(route)
      body.extract(route)
      geometry.extract(key)
      state[key]['extracted'] = signature
  state_path.write_text(json.dumps(state, indent=2))


if __name__ == '__main__':
  main()
