import hashlib
from pathlib import Path
import unittest

from tools.odyssey_eps.rwd import Image, build, COMMUNITY, STOCK

DATA = Path.home() / 'comma-logs/odyssey-eps/dl/rwd'


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
    mod = build(raw, outputs=COMMUNITY, limits=(2560, 2560))
    self.assertEqual(hashlib.sha256(mod).hexdigest(), '7086b01cf9c78b4a094662d8f9f0c9fb915d1ea497d90d56f21c0224c986b9b8')
    self.assertEqual(mod, (DATA / 'mod25xv1-39990-THR,A020.rwd').read_bytes())
    damaged = bytearray(raw)
    damaged[100] ^= 1
    with self.assertRaises(ValueError):
      Image.decode(damaged)

  def test_synthetic_without_honda_binary(self):
    import struct
    from tools.odyssey_eps.rwd import TABLES, STOCK_ID, MOD_ID, word_sum
    payload = bytearray(0x74000)
    for i in range(14):
      payload[i*32:i*32+len(STOCK_ID)] = STOCK_ID
    for off in TABLES:
      struct.pack_into('>9H', payload, off-20, *range(0, 1025, 128))
      struct.pack_into('>8H', payload, off, *STOCK)
      struct.pack_into('>2H', payload, off+22, 1024, 1024)
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
