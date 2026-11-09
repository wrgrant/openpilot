"""Matched default/alternate model experiments on a fixed video subset."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
from tools.odyssey_sim.calib.identify import S


WINDOWS = ('000001a3_01', '000001aa_01', '000001ab_00')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--scratch', type=Path, required=True)
  ap.add_argument('--workers', type=int, default=3)
  ap.add_argument('--windows', nargs='+', default=WINDOWS)
  ap.add_argument('--skip-transfer', action='store_true')
  ap.add_argument('--all-windows', action='store_true')
  a = ap.parse_args()
  inv = json.loads((S/'vsim3/inventory.json').read_text())
  jobs = []
  items = [w['id'] for k in ('000001a3', '000001a7', '000001aa', '000001ab') for w in inv[k]['windows']] if a.all_windows else a.windows
  for name in ('TSFDOM', 'CHEDDAR'):
    for item in items:
      key = item.split('_')[0]
      route = next((Path.home()/'comma-logs').glob('*_'+key+'--*'))
      w = next(w for w in inv[key]['windows'] if w['id'] == item)
      for mode in ('stock', 'path', 'closed'):
        stem = a.scratch/f'{item}_{name}_{mode}'
        cmd = [sys.executable, '-m', 'tools.odyssey_sim.plan.runner', '--mode', 'stock' if mode == 'closed' else mode,
               '--portable-model', str(a.scratch/(name+'.onnx')), '--route', str(route), '--window', f'{w["lo"]}:{w["hi"]}',
               '--output', str(stem.with_suffix('.json'))]
        if mode == 'closed':
          cmd += ['--controller', 'tools.odyssey_sim.vsim.closed_loop:recorded', '--scratch', str(a.scratch/f'params_{item}_{name}')]
        jobs.append((stem,cmd))
    if a.skip_transfer:
      continue
    # Identical previously validated training video for all injected transfer measurements.
    route = next((Path.home()/'comma-logs').glob('*_000001a3--*'))
    for period in (7, 10, 15, 20):
      stem = a.scratch/f'{name}_sine_{period}'
      jobs.append((stem,[sys.executable, '-m', 'tools.odyssey_sim.plan.runner', '--mode', 'stock', '--portable-model',
                         str(a.scratch/(name+'.onnx')), '--route', str(route), '--window', '610:665', '--dy', '.2',
                         '--shape', 'sine', '--period', str(period), '--output', str(stem.with_suffix('.json'))]))
    stem = a.scratch/f'{name}_transfer_baseline'
    jobs.append((stem,[sys.executable, '-m', 'tools.odyssey_sim.plan.runner', '--mode', 'stock', '--portable-model',
                       str(a.scratch/(name+'.onnx')), '--route', str(route), '--window', '610:665', '--output', str(stem.with_suffix('.json'))]))
  def execute(job):
    stem, cmd = job
    if stem.with_suffix('.json').exists():
      return
    print('START', stem.name, flush=True)
    with stem.with_suffix('.log').open('w') as out:
      p = subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'),check=False)
    if p.returncode:
      stem.with_suffix('.json').write_text(json.dumps({'process_failure':p.returncode}))
    print('END',stem.name,p.returncode,flush=True)
  with ThreadPoolExecutor(max_workers=a.workers) as pool:
    list(pool.map(execute,jobs))


if __name__ == '__main__':
  main()
