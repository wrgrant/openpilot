"""Deterministic 1-in-10 public Odyssey segment sample, owner identity unavailable."""

import csv
from pathlib import Path
import numpy as np

S = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')


def main():
  fs = sorted(Path.home().joinpath('comma-logs/commaSteeringControl/HONDA_ODYSSEY_2018').glob('*.csv'))[::10]
  keys = 't vEgo steerFiltered latAccelSteeringAngle roll latActive steeringPressed steeringAngleDeg latAccelLocalizer'.split()
  data = []
  fw = []
  for j, p in enumerate(fs):
    rows = list(csv.DictReader(p.open()))
    a = np.array([[float(r[k]) if k not in ('latActive', 'steeringPressed') else float(r[k] == 'True') for k in keys] for r in rows])
    data.append(np.c_[np.full(len(a), int(p.stem)), a])
    fw.append(rows[0]['epsFwVersion'])
    if j % 200 == 0:
      print(j, p.name, flush=True)
  a = np.concatenate(data)
  d = {k: a[:, i] for i, k in enumerate(['segment'] + keys)}
  np.savez_compressed(S / 'plant_public.npz', **d)
  (S / 'plant_public_firmware.txt').write_text('\n'.join(sorted(set(fw))))
  print('DONE', len(fs), a.shape, flush=True)


if __name__ == '__main__':
  main()
