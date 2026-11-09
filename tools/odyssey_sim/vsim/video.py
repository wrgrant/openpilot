"""FrameReader indexing/cache with ffmpeg 8's replacement for removed -vsync.

Keep the compatibility override confined to this reader, not a global monkeypatch.
"""
import subprocess
import sys
import numpy as np
from openpilot.tools.lib.framereader import FrameReader, FfmpegDecoder
from openpilot.tools.lib.filereader import FileReader


class ModernDecoder(FfmpegDecoder):
  def get_iterator(self, start_fidx=0, end_fidx=None, frame_skip=1):
    end_fidx = end_fidx or self.frame_count
    fidx = start_fidx
    while fidx < end_fidx:
      f_b, _, off_b, off_e = self._gop_bounds(fidx)
      with FileReader(self.fn) as f:
        f.seek(off_b)
        raw = self.prefix + f.read(off_e - off_b)
      data = subprocess.check_output(['ffmpeg', '-v', 'error', '-threads', '4', '-hwaccel', self.hwaccel, '-c:v', 'hevc', '-f', 'hevc',
                                      '-flags2', 'showall', '-i', 'pipe:0', '-fps_mode', 'passthrough', '-f', 'rawvideo',
                                      '-pix_fmt', 'nv12', 'pipe:1'], input=raw)
      frames = np.frombuffer(data, np.uint8).reshape(-1, self.h * self.w * 3 // 2)
      for i, frame in enumerate(frames):
        fidx = f_b + i
        if fidx >= end_fidx:
          return
        if fidx >= start_fidx and (fidx - start_fidx) % frame_skip == 0:
          yield fidx, frame
      if not len(frames):
        raise ValueError(f'empty decoded GOP: {self.fn}, frame {fidx}')
      fidx += 1


class ModernFrameReader(FrameReader):
  def __init__(self, path):
    # FrameReader owns indexing/cache. Reuse its index for the decoder adapter.
    super().__init__(path, pix_fmt='nv12', hwaccel='videotoolbox' if sys.platform == 'darwin' else 'none')
    self.decoder.__class__ = ModernDecoder
