"""Offline image preparation and calibration inventory; never imports downloaded code."""
import hashlib
import json
from pathlib import Path
import struct

from tools.odyssey_eps.rwd import Image, TABLES, build


def main():
  root = Path.home() / 'comma-logs/odyssey-eps'
  stock = (root / 'dl/rwd/stock_39990-THR-A020.rwd').read_bytes()
  image = Image.decode(stock)
  out = root / 'images'
  out.mkdir(exist_ok=True)
  for scale in (1.5, 2.0):
    # Unknown words remain stock. They are deliberately NOT asserted to be limits.
    (out / f'candidate-{scale:.1f}x-39990-THR,A020.rwd').write_bytes(build(stock, scale=scale))
  (out / 'SHA256SUMS').write_text(''.join(f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}\n' for p in sorted(out.glob('*.rwd'))))
  inventory = {}
  for off in TABLES:
    words = struct.unpack_from('>384H', image.payload, off - 0x100)
    inventory[hex(off)] = {'record_window_start': hex(off - 0x100), 'words': words}
  (root / 'calibration_inventory.json').write_text(json.dumps(inventory, indent=2) + '\n')
  print((out / 'SHA256SUMS').read_text())


if __name__ == '__main__':
  main()
