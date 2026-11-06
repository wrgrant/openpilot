#!/usr/bin/env python3
"""Replay a recorded drive through the real onroad UI and record it to MP4.

Feeds rlog messages into ui_state.sm (as selfdrive/ui/tests/profile_onroad.py does) and the
segment's qcamera.ts into CameraView through a stand-in VisionIpcClient, so the camera image,
path, lead chevron, HUD, Steering Arc and developer side panel are drawn by the branch's own code.

  BIG=1 RECORD=1 OFFSCREEN=1 FPS=20 SCALE=1 RECORD_OUTPUT=out.mp4 \
    python tools/odyssey_ui_replay.py ~/comma-logs/<dongle>_<route> 22 25
"""
import glob
import os
import subprocess
import sys
import time

import numpy as np
import pyray as rl

FPS = 20
CAM_W, CAM_H = 1928, 1208  # road camera resolution; qcamera is a 526x330 downscale of it

# Hide the window: the recording reads the render texture, so nothing needs to be on screen.
_set_flags = rl.set_config_flags
rl.set_config_flags = lambda flags: _set_flags(flags | rl.ConfigFlags.FLAG_WINDOW_HIDDEN)


class _Frame:
  def __init__(self, data, idx):
    self.width, self.height, self.stride = CAM_W, CAM_H, CAM_W
    self.uv_offset = CAM_W * CAM_H
    self.data = data
    self.idx = idx


class _CameraFeed:
  """Decodes qcamera.ts segments to NV12 at road-camera resolution, one frame at a time."""
  def __init__(self, seg_dirs):
    self.seg_dirs = list(seg_dirs)
    self.proc = None
    self.frame = None
    self.new = False
    self.count = 0
    self.frame_size = CAM_W * CAM_H * 3 // 2

  def _open_next(self):
    if self.proc:
      self.proc.kill()
    if not self.seg_dirs:
      self.proc = None
      return False
    path = os.path.join(self.seg_dirs.pop(0), "qcamera.ts")
    self.proc = subprocess.Popen(["ffmpeg", "-v", "error", "-i", path, "-vf", f"scale={CAM_W}:{CAM_H}:flags=bicubic",
                                  "-pix_fmt", "nv12", "-f", "rawvideo", "pipe:1"], stdout=subprocess.PIPE)
    return True

  def advance(self):
    while True:
      if self.proc is None and not self._open_next():
        return
      buf = self.proc.stdout.read(self.frame_size)
      if len(buf) == self.frame_size:
        self.frame = _Frame(np.frombuffer(buf, dtype=np.uint8), self.count % 4)
        self.count += 1
        self.new = True
        return
      self.proc = None


FEED: _CameraFeed | None = None


class FakeVisionIpcClient:
  def __init__(self, name, stream_type, conflate=True):
    self.stream_type = stream_type
    self._connected = False
    self.num_buffers = 4
    self.width, self.height, self.stride = CAM_W, CAM_H, CAM_W

  def connect(self, blocking):
    self._connected = True
    return True

  def is_connected(self):
    return self._connected

  def recv(self, timeout_ms=0):
    if FEED and FEED.new:
      FEED.new = False
      return FEED.frame
    return None

  def available_streams(self, name, block=False):
    from openpilot.cereal.visionipc import VisionStreamType
    return [VisionStreamType.VISION_STREAM_NARROW_ROAD]


def chunk_messages(messages):
  dt = 1e9 / FPS
  chunks, cur = [], {}
  # initData is stamped well before the rest of the segment; start the clock at the first carState
  next_t = next(m.logMonoTime for m in messages if m.which() == "carState") + dt
  for m in messages:
    while m.logMonoTime >= next_t:
      chunks.append(cur)
      cur = {}
      next_t += dt
    cur.setdefault("_msgs", []).append(m)
  chunks.append(cur)
  return chunks


def main():
  route_dir, first, last = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
  seg_dirs = sorted((d for d in glob.glob(os.path.join(route_dir, "*--*")) if first <= int(d.rsplit("--", 1)[1]) <= last),
                    key=lambda d: int(d.rsplit("--", 1)[1]))

  from openpilot.common.params import Params
  from openpilot.common.version import terms_version, terms_version_sp, training_version, sunnylink_consent_declined
  params = Params()
  # a set-up device: onboarding done, so the UI goes straight to the onroad view
  params.put("HasAcceptedTerms", terms_version)
  params.put("HasAcceptedTermsSP", terms_version_sp)
  params.put("CompletedTrainingVersion", training_version)
  params.put("CompletedSunnylinkConsentVersion", sunnylink_consent_declined)
  for k, v in {"DevUIInfo": 2, "ChevronInfo": 4}.items():
    params.put(k, v)
  for k, v in {"TorqueBar": True, "IsMetric": False}.items():
    params.put_bool(k, v)

  # No live IPC: every socket is silent and the main SubMaster is fed from the log below.
  import openpilot.cereal.messaging as messaging

  class _NullSock:
    def receive(self, non_blocking=True):
      return None

    def send(self, dat):
      pass

    def all_readers_updated(self):
      return True
  messaging.sub_sock = lambda *a, **k: _NullSock()
  messaging.pub_sock = lambda *a, **k: _NullSock()

  import openpilot.selfdrive.ui.onroad.cameraview as cameraview
  cameraview.VisionIpcClient = FakeVisionIpcClient

  from openpilot.tools.lib.logreader import LogReader
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.selfdrive.ui.layouts.main import MainLayout
  from openpilot.system.ui.lib.application import gui_app

  messages = []
  for d in seg_dirs:
    seg = sorted(LogReader(os.path.join(d, "rlog.zst")), key=lambda m: m.logMonoTime)
    start = next(m.logMonoTime for m in seg if m.which() == "carState")
    messages += [m for m in seg if m.logMonoTime >= start - int(1e9)]
  chunks = chunk_messages(messages)
  print(f"{len(seg_dirs)} segments, {len(messages)} messages, {len(chunks)} frames", flush=True)

  global FEED
  FEED = _CameraFeed(seg_dirs)

  gui_app.init_window("Odyssey UI replay", fps=FPS)
  MainLayout()

  sm = ui_state.sm
  sm.simulation = True  # replay clock: don't let frequency checks mark services dead
  frame = 0

  sim_t = time.monotonic()

  def mock_update(timeout=None):
    nonlocal frame, sim_t
    latest = {}  # newest message per service in this frame, as the UI's conflated sockets would deliver
    for m in chunks[frame].get("_msgs", []):
      w = m.which()
      if w == "qNarrowRoadEncodeIdx":
        FEED.advance()
      if w in sm.data:
        latest[w] = m
    msgs = list(latest.values())
    sim_t += 1.0 / FPS  # log time, so alive/frequency checks see the recorded rates
    sm.update_msgs(sim_t, msgs)
    frame += 1
  sm.update = mock_update

  t0 = time.monotonic()
  for _ in gui_app.render():
    if frame >= len(chunks):
      break
    ui_state.update()
    if frame % (FPS * 30) == 0:
      print(f"  {frame / FPS:.0f}s rendered ({time.monotonic() - t0:.0f}s wall)", flush=True)
  gui_app.close()
  print("done", flush=True)


if __name__ == "__main__":
  main()
