"""Execute frozen research arms on every recorded eligible window."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
from tools.odyssey_sim.calib.identify import S


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--scratch', type=Path, required=True)
  ap.add_argument('--split', choices=['train', 'test', 'all'], default='all')
  ap.add_argument('--modes', nargs='+', default=['stock', 'path', 'lead', 'lane', 'smooth'])
  ap.add_argument('--workers', type=int, default=3)
  a = ap.parse_args()
  a.scratch.mkdir(parents=True, exist_ok=True)
  inventory = json.loads((S/'vsim3/inventory.json').read_text())
  jobs = []
  for key in ('000001a3', '000001a7', '000001aa', '000001ab'):
    if a.split == 'train' and key not in ('000001a3', '000001a7'):
      continue
    if a.split == 'test' and key in ('000001a3', '000001a7'):
      continue
    route = next((Path.home()/'comma-logs').glob('*_'+key+'--*'))
    for w in inventory[key]['windows']:
      if not w['ready']:
        continue
      for mode in a.modes:
        stem = a.scratch/f'{w["id"]}_{mode}'
        if stem.with_suffix('.json').exists():
          continue
        cmd = [sys.executable, '-m', 'tools.odyssey_sim.plan.runner', '--mode', mode, '--route', str(route),
               '--window', f'{w["lo"]}:{w["hi"]}', '--output', str(stem.with_suffix('.json'))]
        if mode == 'path':
          cmd += ['--reference', str(S/'vsim3'/f'{w["id"]}_stock.traces.json')]
        elif mode != 'stock':
          cmd += ['--controller', 'tools.odyssey_sim.vsim.closed_loop:recorded', '--scratch', str(a.scratch/f'params_{w["id"]}_{mode}')]
        jobs.append((stem, cmd))
  def execute(job):
    stem, cmd = job
    print('START', stem.name, flush=True)
    with stem.with_suffix('.log').open('w') as out:
      p = subprocess.run(cmd, stdout=out, stderr=subprocess.STDOUT,
                         env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=str(Path.cwd())), check=False)
    if p.returncode:
      stem.with_suffix('.json').write_text(json.dumps({'process_failure': p.returncode}))
    print('END', stem.name, p.returncode, flush=True)
  with ThreadPoolExecutor(max_workers=a.workers) as pool:
    list(pool.map(execute, jobs))


if __name__ == '__main__':
  main()
