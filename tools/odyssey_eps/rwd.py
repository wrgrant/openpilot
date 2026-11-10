"""Offline, deliberately THR-A020-only RWD codec. No vehicle/network interfaces.

Offsets below are DECODED PAYLOAD offsets, not offsets into the RWD container.
Cipher permutation transcribed from nrdr-derived check_rwd.py (see README).
"""
from dataclasses import dataclass
import math
import struct
import zlib

STOCK_ID = b'39990-THR-A020'
MOD_ID = b'39990-THR,A020'
TABLES = (0x4bbc2, 0x4bec2, 0x4c1c2, 0x5bbd8, 0x5bed8, 0x5c1d8)
STOCK = (2019, 3230, 3980, 4345, 4577, 4721, 4864, 4864)
COMMUNITY = (3968, 7936, 9950, 10863, 11443, 11803, 12160, 12160)
# The EPS checks this block while running: CRC-32/BZIP2 of flash 0x60000-0x6FF5F, stored big-endian at flash 0x6FF7C.
# The flasher does not check it. An image with a stale value flashes, then latches a permanent fault and drops assist.
# Three of the six table copies are inside the block.
BLOCK_START, BLOCK_END, BLOCK_CRC = 0x54000, 0x63f60, 0x63f7c
REFLECT = bytes(int(f'{i:08b}'[::-1], 2) for i in range(256))


def require(ok, message):
  if not ok:
    raise ValueError(message)


def word_sum(data):
  return sum(struct.unpack(f'>{len(data)//2}H', data)) & 0xffff


def block_crc(payload):
  crc = zlib.crc32(bytes(payload[BLOCK_START:BLOCK_END]).translate(REFLECT))
  return int(f'{crc:032b}'[::-1], 2)


def checksums(payload):
  require(len(payload) == 0x74000, 'wrong payload length')
  require(block_crc(payload) == struct.unpack_from('>I', payload, BLOCK_CRC)[0], 'block CRC (flash 0x60000-0x6FF5F)')
  require(word_sum(payload[:0x73f80]) == struct.unpack_from('>H', payload, 0x73f80)[0], 'payload sum')
  require((-word_sum(payload[:0x73ffe])) & 0xffff == struct.unpack_from('>H', payload, 0x73ffe)[0], 'payload negative sum')


@dataclass(frozen=True)
class Image:
  prefix: bytes
  headers: tuple
  payload: bytes

  def validate(self):
    require(self.prefix[:1] == b'\x5a' and len(self.prefix) == 3, 'not 0x5A')
    require(len(self.headers) == 6, 'header count')
    require(self.headers[2] == (b'0',), 'not EPS address')
    require(self.headers[3] == (STOCK_ID + b'\0\0', MOD_ID + b'\0\0'), 'not THR-A020 header')
    require(self.headers[5] == (b'\1\2\3',), 'unknown cipher keys')
    checksums(self.payload)
    require(self.payload.count(STOCK_ID) + self.payload.count(MOD_ID) == 14, 'unexpected payload identities')
    require(not (STOCK_ID in self.payload and MOD_ID in self.payload), 'mixed identities')
    for off in TABLES:
      require(struct.unpack_from('>9H', self.payload, off - 20) == tuple(range(0, 1025, 128)), 'unexpected input axis')
      require(self.payload[off:off+16] == self.payload[TABLES[0]:TABLES[0]+16], 'table copies differ')

  @classmethod
  def decode(cls, raw):
    try:
      require(len(raw) >= 16, 'truncated container')
      require(sum(raw[:-4]) & 0xffffffff == struct.unpack('<I', raw[-4:])[0], 'container checksum')
      pos = 3
      headers = []
      for _ in range(6):
        count = raw[pos]
        pos += 1
        values = []
        for _ in range(count):
          size = raw[pos]
          pos += 1
          require(pos + size <= len(raw) - 12, 'truncated header')
          values.append(bytes(raw[pos:pos+size]))
          pos += size
        headers.append(tuple(values))
      start, size = struct.unpack_from('>II', raw, pos)
      pos += 8
      require(start == 0xc000 and size == 0x74000 and pos + size == len(raw)-4, 'unexpected flash layout')
      image = cls(bytes(raw[:3]), tuple(headers), bytes(raw[pos:pos+size]).translate(DECRYPT))
      image.validate()
      return image
    except (IndexError, struct.error) as e:
      raise ValueError('malformed container') from e

  def encode(self):
    self.validate()
    raw = bytearray(self.prefix)
    for values in self.headers:
      raw.append(len(values))
      for value in values:
        raw.append(len(value))
        raw.extend(value)
    raw.extend(struct.pack('>II', 0xc000, len(self.payload)))
    raw.extend(self.payload.translate(ENCRYPT))
    raw.extend(struct.pack('<I', sum(raw) & 0xffffffff))
    return bytes(raw)


def build(raw, *, outputs=None, scale=None, limits=(1024, 1024), identity=MOD_ID):
  image = Image.decode(raw)
  require(STOCK_ID in image.payload, 'build requires stock identity')
  require(all(struct.unpack_from('>8H', image.payload, o) == STOCK for o in TABLES), 'build requires stock tables')
  require(identity in (STOCK_ID, MOD_ID), 'unsupported identity')
  require((outputs is None) != (scale is None), 'supply outputs OR scale')
  if scale is not None:
    require(isinstance(scale, (float, int)) and math.isfinite(scale) and 1 <= scale <= 2.5, 'scale outside 1..2.5')
    outputs = tuple(round(x * scale) for x in STOCK)
  outputs = tuple(outputs)
  require(len(outputs) == 8 and all(type(x) is int and 0 <= x <= 65535 for x in outputs), 'invalid outputs')
  require(all(a <= b for a, b in zip(outputs, outputs[1:], strict=False)), 'outputs must be monotone')
  require(len(limits) == 2 and all(type(x) is int and 0 <= x <= 2560 for x in limits), 'invalid limits')
  payload = bytearray(image.payload.replace(STOCK_ID, identity))
  for off in TABLES:
    struct.pack_into('>8H', payload, off, *outputs)
    # Two unknown words six bytes after the eight nonzero output knots.
    struct.pack_into('>2H', payload, off + 22, *limits)
  struct.pack_into('>I', payload, BLOCK_CRC, block_crc(payload))  # before the sums, which cover it
  struct.pack_into('>H', payload, 0x73f80, word_sum(payload[:0x73f80]))
  struct.pack_into('>H', payload, 0x73ffe, -word_sum(payload[:0x73ffe]) & 0xffff)
  result = Image(image.prefix, image.headers, bytes(payload)).encode()
  Image.decode(result)
  return result

DECRYPT = bytes.fromhex("""
24e4a4ead999596483430319ae6e2ec328e8a8eedd9d5d688747071db27232c72cecacf2e1a1616c8b4b0b21b67636cb30f0
b0f6e5a565708f4f0f25ba7a3acf34f4b4fae9a9697493531329be7e3ed338f8b8feedad6d789757172dc28242d73cfcbc02
f1b1717c9b5b1b31c68646db4000c006f5b575809f5f1f35ca8a4adf4404c40af9b97984a3632339ce8e4ee34808c80efdbd
7d88a767273dd29252e74c0ccc1201c1818cab6b2b41d69656eb5010d01605c58590af6f2f45da9a5aef5414d41a09c98994
b3733349de9e5ef35818d81e0dcd8d98b777374de2a262f75c1cdc2211d1919cbb7b3b51e6a666fb6020e02615d595a0bf7f
3f55aa6a2aff
""")
ENCRYPT = bytes.fromhex("""
71a4630a81b4731a91c4832aa1d4933ab1e4a34ac1f4b35ad10bc36ae11bd37af12be38a003bf39a104bfeaa205b0eba306b
1eca407b2eda508b3eea609b4efa70ab5e0980bb6e1990cb7e29a0db8e39b0eb9e49c0fbae59d006be69e016ce79f026de89
0736ee991746fda927560db937661dc947762dd957863de967964df977a65d0887b66d1897c67d28a7d68d38b7e69d48c7f6
ad58d705bd68e715cd78f725dd880235ed981245fca822550cb832651cc842752cd852853ce862954cf872a55c0f82b56c1f
92c57c2fa2d58c3fb2e59c4fc2f5ac5fd204bc6fe214cc7ff224dc8f0134ec9f114403af215413bf316423cf417433df5184
43ef619453ff
""")
