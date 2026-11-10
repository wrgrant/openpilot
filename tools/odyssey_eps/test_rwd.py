import hashlib
from pathlib import Path
import unittest

from tools.odyssey_eps.rwd import Image, build, BLOCK_CRC, COMMUNITY, DECRYPT, STOCK

DATA = Path.home() / 'comma-logs/odyssey-eps/dl/rwd'


def _payload(raw):
  """Decoded payload of a container that validation refuses."""
  return raw[-4 - 0x74000:-4].translate(DECRYPT)


class TestRwd(unittest.TestCase):
  def test_truncated_rejected(self):
    for raw in (b'', b'\x5a', bytes(100)):
      with self.assertRaises(ValueError):
        Image.decode(raw)

  @unittest.skipUnless(all((DATA / n).exists() for n in ('stock_39990-THR-A020.rwd', 'mod25xv1-39990-THR,A020.rwd')), 'external Honda firmware absent')
  def test_reference_images(self):
    raw = (DATA / 'stock_39990-THR-A020.rwd').read_bytes()
    image = Image.decode(raw)
    self.assertEqual(image.encode(), raw)
    self.assertEqual(hashlib.sha256(raw).hexdigest(), '90cb1d23d8a27f4c45d52fcbc0a6f08813fb2ae87a0c2f9069f552edd7715edb')
    self.assertEqual(build(raw, outputs=STOCK, limits=(1024, 1024), identity=b'39990-THR-A020'), raw)
    # the community's first image changed tables inside the CRC block and kept the stock CRC: it must be refused
    v1 = (DATA / 'mod25xv1-39990-THR,A020.rwd').read_bytes()
    with self.assertRaisesRegex(ValueError, 'block CRC'):
      Image.decode(v1)
    mod = build(raw, outputs=COMMUNITY, limits=(2560, 2560))
    # ours is the same image with the CRC brought up to date (and the sums that cover it)
    differ = [i for i, (a, b) in enumerate(zip(Image.decode(mod).payload, _payload(v1), strict=True)) if a != b]
    self.assertTrue(set(differ) <= {*range(BLOCK_CRC, BLOCK_CRC + 4), 0x73f80, 0x73f81, 0x73ffe, 0x73fff} and differ[0] == BLOCK_CRC)
    damaged = bytearray(raw)
    damaged[100] ^= 1
    with self.assertRaises(ValueError):
      Image.decode(damaged)

  def test_synthetic_without_honda_binary(self):
    import struct
    from tools.odyssey_eps.rwd import TABLES, STOCK_ID, MOD_ID, BLOCK_CRC, block_crc, word_sum
    payload = bytearray(0x74000)
    for i in range(14):
      payload[i*32:i*32+len(STOCK_ID)] = STOCK_ID
    for off in TABLES:
      struct.pack_into('>9H', payload, off-20, *range(0, 1025, 128))
      struct.pack_into('>8H', payload, off, *STOCK)
      struct.pack_into('>2H', payload, off+22, 1024, 1024)
    struct.pack_into('>I', payload, BLOCK_CRC, block_crc(payload))
    struct.pack_into('>H', payload, 0x73f80, word_sum(payload[:0x73f80]))
    struct.pack_into('>H', payload, 0x73ffe, -word_sum(payload[:0x73ffe]) & 65535)
    headers = ((b'\0',), (), (b'0',), (STOCK_ID+b'\0\0', MOD_ID+b'\0\0'), (b'\1\x11\1\x12\x11\x20',)*2, (b'\1\2\3',))
    raw = Image(b'\x5a\0\0', headers, bytes(payload)).encode()
    self.assertEqual(Image.decode(raw).encode(), raw)
    result = Image.decode(build(raw, scale=1.5))
    self.assertEqual(struct.unpack_from('>H', result.payload, TABLES[0])[0], 3028)
    for scale in (float('nan'), 0, 3):
      with self.assertRaises(ValueError):
        build(raw, scale=scale)
