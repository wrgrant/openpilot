"""Offline command/motor characterization from plant_data NPZ caches.

Requires normalized ACTUAL CAN command, speed m/s, unsigned motor, active,
pressed and drv. Cache raw route CAN using the existing plant extraction path.
Never uses desired actuator command as a substitute for transmitted command.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from opendbc.sunnypilot.car.honda.eps_mod import ODYSSEY_FIRMWARE_OUT, ODYSSEY_STOCK_OUT, ODYSSEY_TABLE_IN

TORQUE_BP = np.array([0, .1, .2, .3, .4, .5, .6, .8, 1])
SPEED_EDGES = np.array([0, 3, 7, 12, 20, 35, 50])


def characterize(paths):
  cells = [[[] for _ in TORQUE_BP] for _ in SPEED_EDGES[:-1]]
  for path in paths:
    with np.load(path, allow_pickle=False) as d:
      cmd, motor, speed = d['cmd'], d['motor'], d['v']
      good = d['active'].astype(bool) & ~d['pressed'].astype(bool) & (np.abs(d['drv']) < 100)
      good &= np.isfinite(cmd) & np.isfinite(motor) & np.isfinite(speed) & (motor >= 0) & (np.abs(cmd) <= 1)
      # One independent observation per nonoverlapping second; exclude transients.
      for start in range(0, len(cmd)-99, 100):
        sl = slice(start, start+100)
        if not good[sl].all() or np.ptp(cmd[sl]) > .02 or np.ptp(speed[sl]) > 1:
          continue
        if 't' in d and (np.any(np.diff(d['t'][sl]) <= 0) or np.max(np.diff(d['t'][sl])) > .03):
          continue
        c, v = abs(np.median(cmd[sl])), np.median(speed[sl])
        j = int(np.argmin(abs(TORQUE_BP-c)))
        i = int(np.searchsorted(SPEED_EDGES, v, side='right')-1)
        if 0 <= i < len(cells) and abs(TORQUE_BP[j]-c) <= .03:
          cells[i][j].append(float(np.median(motor[sl])))
  return [[{'n': len(x), 'motor': float(np.median(x)) if x else None,
                'p10': float(np.percentile(x, 10)) if x else None,
                'p90': float(np.percentile(x, 90)) if x else None} for x in row] for row in cells]


def fit_map(stock, modified, scale):
  """Invert only observed modified ranges. Missing coverage is a failed fit."""
  rows, speeds = [], []
  for i, (s, m) in enumerate(zip(stock, modified, strict=True)):
    if not all(c['n'] >= 3 for c in s):
      continue
    observed = [j for j, c in enumerate(m) if c['n'] >= 3]
    if len(observed) < 4 or observed[0] != 0:
      continue
    x = TORQUE_BP[observed]
    y_raw = np.array([m[j]['motor'] for j in observed])
    y = np.maximum.accumulate(y_raw)
    target = np.array([c['motor'] for c in s])
    if np.max(abs(y-y_raw)) > max(5, .15*y[-1]) or target.max() > y[-1] or target.min() < y[0]:
      continue
    # Reject plateaus: inverse would otherwise depend on arbitrary tie handling.
    if np.any(np.diff(y) <= 0):
      continue
    # The modified command that reproduces each stock motor torque, written as the stock-equivalent command
    # OdysseyEpsMap expects: the stock command whose table output equals the modified table's output there.
    virtual = np.interp(np.interp(target, y, x), ODYSSEY_TABLE_IN, ODYSSEY_FIRMWARE_OUT[scale])
    if virtual.max() > 1.03:  # 3% allows for interpolating the measured curve between its nine command levels
      continue  # matching stock would need more than stock's own table output: not a stock-equivalent map
    virtual = np.minimum(virtual, 1.0)
    mapped = np.interp(virtual, ODYSSEY_STOCK_OUT[:-1], ODYSSEY_TABLE_IN[:-1])
    mapped[0] = 0
    if np.any(np.diff(mapped) < 0):
      continue
    speeds.append(float((SPEED_EDGES[i]+SPEED_EDGES[i+1])/2))
    rows.append(mapped.tolist())
  if len(rows) < 2:
    raise ValueError('insufficient independent coverage: need >=2 speed bins, full stock curve and overlapping modified curve; no map emitted')
  return {'firmware_scale': scale, 'authority_ceiling': 1.0, 'speed_bp': [0]+speeds,
              'speed_gain': [1]*(len(speeds)+1), 'torque_bp': TORQUE_BP.tolist(), 'torque_v': TORQUE_BP.tolist(),
              'torque_rows': [rows[0]]+rows}


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--stock', type=Path, required=True, help='directory of stock NPZ CAN caches')
  p.add_argument('--modified', type=Path, required=True, help='directory of post-flash NPZ CAN caches')
  p.add_argument('--scale', type=float, choices=(1.5, 2, 2.5), required=True)
  p.add_argument('--out', type=Path, required=True, help='new output directory; never a live params directory')
  args = p.parse_args()
  args.out.mkdir(exist_ok=False)
  stock, modified = characterize(sorted(args.stock.glob('*.npz'))), characterize(sorted(args.modified.glob('*.npz')))
  evidence = {'speed_edges_ms': SPEED_EDGES.tolist(), 'torque_bp': TORQUE_BP.tolist(), 'stock': stock, 'modified': modified}
  try:
    config = fit_map(stock, modified, args.scale)
    (args.out / 'OdysseyEpsMap.json').write_text(json.dumps(config, indent=2)+'\n')
    evidence['status'] = 'candidate: review curve scatter and held-out drive before installation'
  except ValueError as e:
    evidence['status'] = str(e)
  (args.out / 'fit_report.json').write_text(json.dumps(evidence, indent=2)+'\n')
  print(evidence['status'])


if __name__ == '__main__':
  main()
