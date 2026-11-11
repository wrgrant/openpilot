"""Bounded 100 Hz pulse experiment. During a shot the command is zero except for the pulse, which is what bump mode
sends (steer_deadband.bump_comp switches between zero and the pulse height); all I/O belongs to PulseRuntime's worker.

Entry rules come from replaying recorded drives through this state machine (report_pulse.md). The first version wanted
lane keeping's command inside +-3% and the van within 8 cm of centre for 3 s, and in 59 minutes of eligible steering
it never fired: this van holds 4-5% against road crown and sits 8-18 cm right of centre. So a shot may start while
lane keeping holds up to MAX_RELEASE on average (recorded with the shot as "released"), from a low-passed lane pose,
and ends as soon as the van has moved MAX_DRIFT. Shots are as short as the pulse allows, because drift grows with the
square of the time lane keeping is away.

This is an experiment, not a lane safety controller.
Positive command follows carControl/Honda wheel-angle sign, NOT CAN STEER_TORQUE sign.
"""
from dataclasses import dataclass, asdict
import math

MAX_TORQUE = .15  # absolute controlsd request; panda and Honda rate limits still apply
MAX_RELEASE = .06  # lane keeping's mean command over the settling time; more means a curve, strong crown or wind
# m, low-passed lane pose. A 3.6 m lane leaves this 2.0 m van 0.8 m each side when centred: start within 0.25 m, and
# stop 0.35 m short of the line, or as soon as the van has moved 0.25 m since the shot began.
START_OFFSET, ABORT_OFFSET, MAX_DRIFT = .25, .45, .20
START_HEADING, ABORT_HEADING = .003, .012  # rad, low-passed: about 0.23/0.69 degrees, limits accumulating lateral velocity
MIN_LANE_WIDTH = 3.2  # m between the inner lines: no shots in narrow or merging lanes
VAN_HALF_WIDTH, MIN_EDGE = 1.0, .30  # m: the van's side never gets closer than MIN_EDGE to an inner line during a shot
POSE_SMOOTH_S = .5  # the lane heading is noisy frame to frame (about 0.0012 rad rms): gate on its low-passed value.
# The offset is steadier (about 3 cm) and is gated raw, so drift is seen without the filter's half-second lag.
MAX_CURVE_ACCEL = .25  # m/s² across full 3 s model horizon (a 2900 m radius at 60 mph), not just present curvature
MIN_SPEED, BAND_SPLIT, MAX_SPEED = 25 * .44704, 45 * .44704, 70 * .44704  # no unbounded 'above 45' band
MAX_SPEED_CHANGE = .5  # m/s from shot entry; a changing speed changes the experiment
MIN_LANE_PROB = .7  # both inner lane lines, stronger than ordinary lane correction
DRIVER_TORQUE, DRIVER_SECONDS, HARD_DRIVER_TORQUE = 800., .15, 1200.  # Honda raw sensor units; provisional
MAX_MODEL_AGE, MAX_FRAME_GAP = .15, .03  # seconds; stale road or scheduler gap aborts
SETTLE_S, LEAD_S, TAIL_S, HARD_SHOT_S = 1., .2, .5, 2.3  # a shot is lead-in + signal + tail: 1.5 to 2.2 s
TRAIN_S = 1.0  # pulse trains and shams: four 0.25 s periods
PAIR_GAP_S = .5  # between the two opposite pulses of a pair: long enough to see the first one's wheel response
RAMP_LEVELS, RAMP_STEP_S = (.02, .03, .04, .05, .06, .08), .15  # one direction, rising: where the wheel starts to creep
SQUARE_LEVELS, SQUARE_HALF_S = (.02, .04, .06, .08, .10, .12), .1  # +level then -level: the EPS map, with no net push
CHIRP_HZ, CHIRP_S = (.5, 3.), 1.5  # a sine sweeping these frequencies: the response the simulator has to match
HANDOVER_STEP = .01  # normalized command per frame on the way back to lane keeping
VERSION = 2
QUEUE_MAX, QUEUE_HEIGHT = 12, .12  # extra shots a drive companion may queue (OdysseyPulseQueue), and their largest command
KINDS = ('single', 'pair', 'train', 'ramp', 'square', 'chirp', 'sham')
FIRE_EXPIRE_S = 2.
ACTIVE = ('lead-in', 'pulse', 'tail')


@dataclass(frozen=True)
class Condition:
  """One shot's signal. kind: single (one pulse), pair (a pulse, PAIR_GAP_S, the opposite pulse: `period` is each
  pulse's length), train (pulses of `duty` of each `period`), ramp, square, chirp (see the constants), or sham (nothing).
  direction is the sign of the first push."""
  kind: str
  height: float
  duration: float
  direction: int
  period: float = 0.
  duty: float = 0.

  def command(self, elapsed):
    tick = round((elapsed - LEAD_S) * 100)  # whole 100 Hz frames, so the signal's edges do not depend on float rounding
    if not 0 <= tick < round(self.duration * 100) or self.kind == 'sham':
      return 0.
    t = tick / 100
    if self.kind == 'train':
      period = round(self.period * 100)
      value = self.height if tick % period < round(period * self.duty) else 0.
    elif self.kind == 'pair':
      width, second = round(self.period * 100), round((self.period + PAIR_GAP_S) * 100)
      value = self.height if tick < width else -self.height if second <= tick < second + width else 0.
    elif self.kind == 'ramp':
      value = RAMP_LEVELS[min(tick // round(RAMP_STEP_S * 100), len(RAMP_LEVELS) - 1)]
    elif self.kind == 'square':
      half = round(SQUARE_HALF_S * 100)
      value = SQUARE_LEVELS[min(tick // (2 * half), len(SQUARE_LEVELS) - 1)] * (1 if (tick // half) % 2 == 0 else -1)
    elif self.kind == 'chirp':
      f0, f1 = CHIRP_HZ
      value = self.height * math.sin(2 * math.pi * (f0 * t + (f1 - f0) * t * t / (2 * self.duration)))
    else:
      value = self.height
    return self.direction * value

  def label(self):
    if self.kind in ('single', 'pair'):
      return f"{self.kind} {100 * self.height:.0f}% {self.period or self.duration:g}s"
    if self.kind == 'train':
      return f"train {100 * self.height:.0f}% on {100 * self.duty:.0f}%"
    return f"chirp {100 * self.height:.0f}%" if self.kind == 'chirp' else self.kind


def session():
  """The 26 shots of one pass, most useful first, so a short drive still brings home a balanced set: about 10-15
  minutes of straight highway at the shot rate replayed from recorded drives. Names are the shot ids in the log."""
  def pair(h, w, s):
    return Condition('pair', h, 2 * w + PAIR_GAP_S, s, period=w)

  def train(h, duty, s):
    return Condition('train', h, TRAIN_S, s, period=.25, duty=duty)
  ramp = len(RAMP_LEVELS) * RAMP_STEP_S
  square = len(SQUARE_LEVELS) * 2 * SQUARE_HALF_S
  return (
    ('square+', Condition('square', SQUARE_LEVELS[-1], square, 1)),  # the EPS map: motor torque at each command level
    ('ramp+', Condition('ramp', RAMP_LEVELS[-1], ramp, 1)),  # the level at which the wheel starts to move
    ('pair-6-0.1', pair(.06, .1, 1)),  # the bump mode's own pulse, then its reversal
    ('chirp-5', Condition('chirp', .05, CHIRP_S, 1)),  # frequency response at a small size, for the simulator
    ('train-6-30+', train(.06, .3, 1)),  # does pulse-width modulation deliver a small average torque?
    ('sham-a', Condition('sham', 0., TRAIN_S, 1)),
    ('ramp-', Condition('ramp', RAMP_LEVELS[-1], ramp, -1)),
    ('pair-6-0.2', pair(.06, .2, -1)),
    ('train-6-60-', train(.06, .6, -1)),
    ('pair-4-0.1', pair(.04, .1, -1)),
    ('pair-8-0.1', pair(.08, .1, 1)),
    ('sham-b', Condition('sham', 0., TRAIN_S, -1)),
    ('chirp-9', Condition('chirp', .09, CHIRP_S, -1)),
    ('square-', Condition('square', SQUARE_LEVELS[-1], square, -1)),
    ('pair-6-0.05', pair(.06, .05, 1)),
    ('train-6-30-', train(.06, .3, -1)),
    ('train-6-60+', train(.06, .6, 1)),
    ('pair-4-0.2', pair(.04, .2, 1)),
    ('pair-8-0.2', pair(.08, .2, -1)),
    ('train-8-30+', train(.08, .3, 1)),
    ('chirp-5i', Condition('chirp', .05, CHIRP_S, -1)),
    ('pair-4-0.05', pair(.04, .05, -1)),
    ('pair-8-0.05', pair(.08, .05, 1)),
    ('train-8-30-', train(.08, .3, -1)),
    ('chirp-9i', Condition('chirp', .09, CHIRP_S, 1)),
    ('sham-c', Condition('sham', 0., TRAIN_S, 1)),
  )


def parse_queue(raw):
  """Extra shots queued from outside (tools/odyssey_drive_companion.py) as a JSON list of Condition fields. Anything
  malformed, too long for a shot or over QUEUE_HEIGHT is dropped whole, never clipped into a different shot. Ids are
  stable for the same queue text, so a shot already done is not run again when the queue is re-read."""
  import json
  import zlib
  try:
    items = json.loads(raw or '[]')
  except (ValueError, TypeError):
    return ()
  out = []
  for item in items[:QUEUE_MAX] if isinstance(items, list) else ():
    try:
      c = Condition(str(item['kind']), float(item['height']), float(item['duration']), 1 if float(item['direction']) > 0 else -1,
                    float(item.get('period', 0.)), float(item.get('duty', 0.)))
      values = (c.height, c.duration, c.period, c.duty)
      ok = (c.kind in KINDS and all(math.isfinite(v) and v >= 0 for v in values) and
            0 < c.duration < HARD_SHOT_S - LEAD_S - TAIL_S and
            (c.kind != 'train' or (.1 <= c.period <= 1. and 0 < c.duty <= 1.)) and
            (c.kind != 'pair' or (.02 <= c.period <= .4 and abs(c.duration - (2 * c.period + PAIR_GAP_S)) < .011)) and
            max(abs(c.command(LEAD_S + k * .01)) for k in range(round(c.duration * 100))) <= QUEUE_HEIGHT + 1e-9)
    except (KeyError, TypeError, ValueError, AttributeError):
      continue
    if ok:
      tag = zlib.crc32(json.dumps(asdict(c), sort_keys=True).encode()) & 0xffffff
      out.append((f'q:{tag:06x}:{len(out)}', c))
  return tuple(out)


def plan(passes=3):
  """The session, then the same again with every first push reversed, then as the first: repeats for the noise."""
  out = []
  for repeat in range(passes):
    for name, c in session():
      out.append((f'{repeat}:{name}', c if repeat % 2 == 0 else Condition(c.kind, c.height, c.duration, -c.direction, c.period, c.duty)))
  return tuple(out)


@dataclass(frozen=True)
class Inputs:
  active: bool = True
  speed: float = 25.
  offset: float = 0.
  heading: float = 0.
  curve_accel: float = 0.
  lane_prob: float = 1.
  lane_width: float = 3.7
  driver_torque: float = 0.
  blinker: bool = False
  brake: bool = False
  lane_change: bool = False
  model_age: float = 0.
  valid: bool = True
  angle: float = 0.
  rate: float = 0.


def band(speed):
  return '25-45' if speed < BAND_SPLIT else '45-70'


class Pulse:
  def __init__(self, progress=None):
    self.plan = plan()
    self.done = {b: set() for b in ('25-45', '45-70')}
    if isinstance(progress, dict) and progress.get('version') == VERSION:
      for b in self.done:
        ids = progress.get('done', {}).get(b, [])
        if isinstance(ids, list):
          self.done[b] = set(ids) & {i for i, _ in self.plan}
    self.state, self.reason, self.last_result = 'wait', 'waiting for a straight', ''
    self.last, self.stable, self.driver_since = None, None, None
    self.start, self.shot, self.record = 0., None, None
    self.events = []
    self.previous_command = 0.
    self.last_direction = 0
    self.broken = False
    self.speed_band = '45-70'
    self.settle_speed = None
    self.base_sum, self.base_n, self.offset_start = 0., 0, 0.
    self.mode, self.burst = 'manual', 5
    self.series_index, self.series_total = 0, 0
    self.recovery_start = None
    self.shot_mode = 'manual'
    self.fire_until = None
    self.offset_measured = False
    self.manual_start = None
    self.extra = ()  # queued shots (parse_queue), run before what is left of the plan
    self.aborts = {}  # reason -> count since controls started, for the panel
    self.ignore_wheel = False  # the driver's override: the wheel torque sensor neither blocks nor stops a shot
    self.wheel_peak, self.wheel_peak_time = 0., -math.inf  # largest sensor reading in the last second, for the panel

  @property
  def in_shot(self):
    return self.state in ACTIVE

  def progress(self):
    planned = {i for i, _ in self.plan}  # queued shots are not kept across drives
    return {'version': VERSION, 'done': {b: sorted(ids & planned) for b, ids in self.done.items()}}

  def queue_left(self):
    return sum(i not in self.done[self.speed_band] for i, _ in self.extra)

  def gate(self, now, x, manual=False):
    if not x.valid or not all(math.isfinite(v) for v in asdict(x).values()):
      return 'invalid data'
    if not x.active:
      return 'steering inactive'
    if x.model_age < 0 or x.model_age > MAX_MODEL_AGE:
      return 'stale model'
    if self.last is not None and not 0 < now - self.last <= MAX_FRAME_GAP:
      return 'frame gap'
    if x.blinker:
      return 'blinker'
    if x.brake:
      return 'brake'
    if x.lane_change:
      return 'lane change'
    if not MIN_SPEED <= x.speed <= MAX_SPEED:
      return 'speed band'
    if not manual:
      if x.lane_prob < MIN_LANE_PROB:
        return 'lane lines lost'
      if x.lane_width < MIN_LANE_WIDTH:
        return 'narrow lane'
      if abs(x.offset) > ABORT_OFFSET:
        return 'offset'
      if x.lane_width / 2 - VAN_HALF_WIDTH - abs(x.offset) < MIN_EDGE:
        return 'near the line'
      if abs(x.heading) > ABORT_HEADING:
        return 'heading'
      if abs(x.curve_accel) > MAX_CURVE_ACCEL:
        return 'curve ahead'
    if not self.ignore_wheel and (abs(x.driver_torque) >= HARD_DRIVER_TORQUE or
                                  (self.driver_since is not None and now - self.driver_since >= DRIVER_SECONDS)):
      return "you're steering"
    if self.in_shot and (abs(x.speed - self.record['speed']) > MAX_SPEED_CHANGE or band(x.speed) != self.speed_band):
      return 'speed changed'
    measured = x.lane_prob >= MIN_LANE_PROB
    if manual and self.in_shot and measured and x.lane_width / 2 - VAN_HALF_WIDTH - abs(x.offset) < MIN_EDGE:
      return 'near the line'
    if self.in_shot and (not manual or (measured and self.offset_measured)) and abs(x.offset - self.offset_start) > MAX_DRIFT:
      return 'drifted'
    return ''

  def finish(self, now, x, reason=''):
    if self.record is not None:
      end = now if math.isfinite(now) else self.last
      angle = getattr(x, 'angle', self.record['angle_start'])
      angle = angle if math.isfinite(angle) else self.record['angle_start']
      self.record.update(end=end, completed=not reason, abort=reason, wheel_delta=angle-self.record['angle_start'])
      self.events.append(self.record)
      if not reason:
        self.done[self.speed_band].add(self.shot[0])
      c = self.shot[1]
      self.last_result = f'{c.label()}: ' + (f"stopped: {reason}" if reason else f"wheel {self.record['wheel_delta']:+.2f}°")
      if reason:
        self.aborts[reason] = self.aborts.get(reason, 0) + 1
    self.recovery_start = now
    if self.series_total and not reason:
      self.series_total = self.burst if self.mode == 'burst' else 0
    if reason or self.series_index >= self.series_total:
      self.series_total = 0
    self.record, self.stable, self.state = None, None, 'recover'

  def step(self, now, normal, x, selected, paused=False, fire=False):
    """Pure bounded computation; caller drains events. Internal fault latches experiment off."""
    try:
      if fire and self.series_total:
        self.series_total = 0
        self.fire_until = None
        if self.in_shot:
          self.finish(now, x, 'driver ended burst')
        return normal
      if fire and selected and not paused and not self.in_shot:
        self.fire_until = now + FIRE_EXPIRE_S
      if not selected or paused:
        self.fire_until = None
      if self.fire_until is not None and now >= self.fire_until:
        self.last_result = f'fire expired: {self.reason}'
        self.fire_until = None
      return self._step(now, normal, x, selected, paused)
    except Exception:
      self.broken = True
      if self.record is not None:
        self.finish(now, x, 'internal fault')
      self.reason = 'internal fault'
      return normal
    finally:
      self.last = now

  def _step(self, now, normal, x, selected, paused):
    if abs(x.rate) > .05:
      self.last_direction = 1 if x.rate > 0 else -1
    self.driver_since = (now if self.driver_since is None else self.driver_since) if abs(x.driver_torque) >= DRIVER_TORQUE else None
    if abs(x.driver_torque) >= self.wheel_peak or now - self.wheel_peak_time > 1.:
      self.wheel_peak, self.wheel_peak_time = abs(x.driver_torque), now
    if not self.in_shot and self.series_total:
      self.series_total = self.burst if self.mode == 'burst' and self.series_index < self.burst else 0
    manual = bool(self.record.get('manual')) if self.in_shot else self.fire_until is not None or bool(self.series_total)
    reason = self.gate(now, x, manual) if selected else 'not selected'
    reason = 'paused' if paused else reason
    if self.broken or not math.isfinite(normal) or not math.isfinite(now):
      self.broken = True
      reason = 'internal fault'
    if reason:
      self.series_total = 0
      if self.in_shot:
        self.finish(now, x, reason)
      self.stable, self.reason = None, reason
      self.previous_command = normal
      return normal  # safety aborts return control immediately, bypassing cosmetic slew
    if not self.in_shot:
      self.speed_band = band(x.speed)
      candidates = [(i, c) for i, c in (*self.extra, *self.plan) if i not in self.done[self.speed_band]]
      if not candidates:
        if self.state == 'recover' and abs(normal-self.previous_command) > HANDOVER_STEP:
          self.previous_command += math.copysign(HANDOVER_STEP, normal-self.previous_command)
          self.reason = 're-centring'
          return self.previous_command
        self.fire_until = None
        self.series_total = 0
        self.state, self.reason = 'done', 'done'
        return normal
      if self.settle_speed is None:
        self.settle_speed = x.speed
      speed_stable = abs(x.speed-self.settle_speed) <= .3
      if not speed_stable:
        self.settle_speed = x.speed
      room = x.lane_width / 2 - VAN_HALF_WIDTH - abs(x.offset)  # from the van's side to the nearer inner line
      ready = (speed_stable and abs(x.offset) <= START_OFFSET and room >= MIN_EDGE + MAX_DRIFT and abs(x.heading) <= START_HEADING and
               abs(x.rate) <= .5 and (self.ignore_wheel or abs(x.driver_torque) < DRIVER_TORQUE))
      if not ready or self.stable is None:
        self.base_sum, self.base_n = 0., 0
      self.stable = (now if self.stable is None else self.stable) if ready else None
      self.base_sum, self.base_n = self.base_sum + normal, self.base_n + 1
      if self.stable is not None and now - self.stable >= SETTLE_S and abs(self.base_sum / self.base_n) > MAX_RELEASE:
        self.stable = None  # holding too much torque for a straight: start the settling time over
      recovery_ready = not self.series_total or (now - self.recovery_start >= 1. and abs(x.rate) < .5)
      if (manual and recovery_ready) or (self.mode == 'auto' and self.stable is not None and now - self.stable >= SETTLE_S):
        if self.series_total:
          self.series_index += 1
        else:
          self.series_index = 1
          self.series_total = self.burst if self.mode == 'burst' and manual else 0
        self.shot_mode = self.mode
        self.fire_until = None
        self.offset_measured = x.lane_prob >= MIN_LANE_PROB
        self.shot, self.start = candidates[0], now
        if manual:
          self.manual_start = now
        self.offset_start = x.offset
        self.record = {'id': self.shot[0], 'condition': asdict(self.shot[1]), 'baseline': 0.,
                       'released': normal if manual else self.base_sum / self.base_n, 'start': now,
                       'mode': self.shot_mode, 'series_index': self.series_index,
                       'manual': manual, 'speed': x.speed, 'band': self.speed_band, 'angle_start': x.angle, 'last_direction': self.last_direction}
        self.events.append({**self.record, 'phase': 'start'})
        self.state = 'lead-in'
      else:
        self.reason = 're-centring' if self.state == 'recover' else 'waiting for a straight'
        if self.state == 'recover':
          normal = self.previous_command + max(-HANDOVER_STEP, min(HANDOVER_STEP, normal - self.previous_command))
        self.previous_command = normal
        return normal
    t = now - self.start
    c = self.shot[1]
    if t >= HARD_SHOT_S:
      self.finish(now, x, 'hard timeout')
      return normal
    if t >= LEAD_S + c.duration + TAIL_S:
      self.finish(now, x)
      self.previous_command += max(-HANDOVER_STEP, min(HANDOVER_STEP, normal-self.previous_command))
      return self.previous_command
    self.state = 'lead-in' if t < LEAD_S else 'pulse' if t < LEAD_S + c.duration else 'tail'
    self.reason = 'pulsing' if self.state == 'pulse' else self.state
    self.previous_command = max(-MAX_TORQUE, min(MAX_TORQUE, c.command(t)))
    return self.previous_command


class PulseRuntime:
  """Params, logging and persistence run on a worker. Queue failure latches shots off.

  Worker heartbeat is checked at every frame: no worker means no experimental steering.
  Losing power before persistence can repeat a completed shot, never skip an uncompleted one.
  """
  def __init__(self, params):
    import queue
    import threading
    import time
    # Warm numpy's fit before controlsd's first frame (first LAPACK call can take tens of ms).
    import numpy as np
    np.polynomial.polynomial.polyfit(np.arange(8.), np.zeros(8), 3)
    self.params = params
    self.controller = Pulse()
    self.config = (False, True, None)
    self.mode, self.burst, self.button = 'manual', 5, False
    self.ignore_wheel = False
    self.fire_request = None
    self.fire_seen = None
    self.extra = ()
    self.heartbeat = time.monotonic()
    self.queue = queue.Queue(maxsize=512)
    self.stop = threading.Event()
    self.last_status = -1.
    self.pose = None
    self.model_time = -math.inf
    self.worker = threading.Thread(target=self._worker, daemon=True, name='odyssey-pulse')
    self.worker.start()

  def close(self):
    self.stop.set()
    self.worker.join(timeout=.5)

  def _worker(self):
    import json
    import queue
    import time
    from openpilot.common.swaglog import cloudlog
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import mem_params
    mem = mem_params(self.params)
    try:
      raw = json.loads(self.params.get('OdysseyPulseProgress') or '{}')
      import os
      if self.params.get('OdysseyABExperiment') == 'pulse' and not os.environ.get('REPLAY'):
        self.params.put_bool('OdysseyPulsePaused', True)  # every drive starts paused: the driver taps the panel to begin
      if not os.environ.get('REPLAY'):  # a queue or a note left from an earlier drive means nothing now
        self.params.put('OdysseyPulseQueue', '[]')
        mem.put('OdysseyAgentNote', '')
        mem.remove('OdysseyPulseFire')
      queue_raw, queue_read = None, -1.
      self.config = (False, True, raw)
      last_poll = -1.
      while not self.stop.is_set():
        now = time.monotonic()
        self.heartbeat = now
        if now-last_poll >= .1:
          raw = self.config[2]
          if self.params.get_bool('OdysseyPulseReset'):
            raw = {}
            self.params.put_bool('OdysseyPulseReset', False)
          mode = self.params.get('OdysseyPulseMode')
          self.mode = mode if mode in ('manual', 'burst', 'auto') else 'manual'
          try:
            self.burst = max(1, int(self.params.get('OdysseyPulseBurst') or '5'))
          except ValueError:
            self.burst = 5
          self.button = self.params.get_bool('OdysseyPulseButton')
          self.ignore_wheel = self.params.get_bool('OdysseyPulseIgnoreWheel')
          fire_raw = mem.get('OdysseyPulseFire')
          if fire_raw and fire_raw != self.fire_seen:
            self.fire_seen = fire_raw
            try:
              stamp = float(fire_raw)
              if 0 <= now - stamp < FIRE_EXPIRE_S:
                self.fire_request = stamp
            except ValueError:
              pass
          if now - queue_read >= 1.:
            queue_read, raw_queue = now, self.params.get('OdysseyPulseQueue') or '[]'
            if raw_queue != queue_raw:
              queue_raw, self.extra = raw_queue, parse_queue(raw_queue)
          self.config = (self.params.get('OdysseyABExperiment') == 'pulse', self.params.get_bool('OdysseyPulsePaused'), raw)
          last_poll = now
        try:
          item = self.queue.get(timeout=.05)
        except queue.Empty:
          continue
        kind, data = item
        if kind == 'event':
          cloudlog.event('odyssey_pulse', **data)
        elif kind == 'progress':
          self.params.put('OdysseyPulseProgress', json.dumps(data))
        else:
          mem.put('OdysseyABStatus', json.dumps(data))
    except Exception:
      self.config = (False, True, None)  # heartbeat expiry also prevents further shots
      cloudlog.exception('odyssey_pulse worker disabled')

  def update(self, now, normal, cs, md, active, model_updated, valid, model_age, output):
    import time
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_lane import lane_pose
    c = self.controller
    try:
      selected, paused, progress = self.config
      if progress is not None:
        if c.in_shot:
          c.finish(now, Inputs(angle=cs.steeringAngleDeg), 'progress reset')
          for event in c.events:
            self.queue.put_nowait(('event', event))
        c = self.controller = Pulse(progress)
        self.config = (selected, paused, None)
      request = getattr(self, 'fire_request', None)
      fire = request is not None and request != getattr(self, 'fire_consumed', None)
      if fire:
        self.fire_consumed = request
        age = time.monotonic() - request
        fire = 0 <= age < FIRE_EXPIRE_S
      from opendbc.car.structs import CarState
      button_fire = getattr(self, 'button', False) and any(
        b.pressed and b.type == CarState.ButtonEvent.Type.gapAdjustCruise for b in getattr(cs, 'buttonEvents', ()))
      fire = fire or button_fire
      if button_fire:
        age = 0.
      c.mode, c.burst = getattr(self, 'mode', 'manual'), getattr(self, 'burst', 5)
      if not selected and not c.in_shot and c.state != 'recover':
        c.fire_until = None
        return normal
      if time.monotonic() - self.heartbeat > .5:
        c.broken = True
      if model_updated:
        pose = lane_pose(md)
        if pose is None or self.pose is None or not 0 < now - self.model_time <= MAX_MODEL_AGE:
          self.pose = pose  # after a gap, or with no lines, start from the new value
        else:
          a = (now - self.model_time) / (POSE_SMOOTH_S + now - self.model_time)
          self.pose = (pose[0], self.pose[1] + a * (pose[1] - self.pose[1]))  # raw offset, low-passed heading
        lines = md.laneLines
        self.width = float(lines[2].y[0] - lines[1].y[0]) if len(lines) == 4 and len(lines[1].y) and len(lines[2].y) else 0.
        self.model_time = now
        # Use whole planned 3 second road horizon and the action request; reject incomplete horizon.
        rates, velocities, ts = list(md.orientationRate.z), list(md.velocity.x), list(md.orientationRate.t)
        if (len(rates) != len(velocities) or len(ts) != len(rates) or not ts or ts[-1] < 3. or
            not all(math.isfinite(v) for v in rates + velocities + ts)):
          self.curve = math.inf
        else:
          ahead = [abs(r*v) for r, v, t in zip(rates, velocities, ts, strict=True) if 0 <= t <= 3.]
          self.curve = max([abs(md.action.desiredCurvature * cs.vEgo**2), *ahead]) if ahead else math.inf
      pose = self.pose or (0., 0.)  # no usable lane lines: reported below as lines lost, which is what the driver can act on
      x = Inputs(active=active, speed=cs.vEgo, offset=pose[0], heading=pose[1], curve_accel=getattr(self, 'curve', math.inf),
                 lane_prob=min(md.laneLineProbs[1], md.laneLineProbs[2]) if self.pose is not None and len(md.laneLineProbs) == 4 else 0.,
                 lane_width=getattr(self, 'width', 0.), driver_torque=cs.steeringTorque,
                 blinker=cs.leftBlinker or cs.rightBlinker, brake=cs.brakePressed, lane_change=str(md.meta.laneChangeState) not in ('off', '0'),
                 model_age=max(model_age, now-self.model_time), valid=valid, angle=cs.steeringAngleDeg, rate=cs.steeringRateDeg)
      c.ignore_wheel = getattr(self, 'ignore_wheel', False)
      c.extra = getattr(self, 'extra', ())
      result = c.step(now, normal, x, selected, paused, fire=fire)
      if fire and c.fire_until is not None:
        c.fire_until = min(c.fire_until, now + FIRE_EXPIRE_S - age)
      while c.events:
        event = c.events[0]
        self.queue.put_nowait(('event', event))
        c.events.pop(0)
        if 'completed' in event:
          self.queue.put_nowait(('progress', c.progress()))
      if c.in_shot:
        self.queue.put_nowait(('event', {'phase': 'frame', 'id': c.shot[0], 'start': c.start, 'mono_s': now,
                                       'manual': c.record['manual'], 'command': result, 'carOutput': output, 'state': c.state}))
      if selected and now-self.last_status >= .1:
        self.queue.put_nowait(('status', {'experiment': 'pulse', 'mono_s': now, 'state': c.state, 'reason': c.reason,
                                        'finished': len(c.done[c.speed_band] & {i for i, _ in c.plan}), 'total': len(c.plan), 'band': c.speed_band,
                                        'last_result': c.last_result, 'paused': paused, 'wheel': round(c.wheel_peak),
                                        'wheel_limit': DRIVER_TORQUE, 'ignore_wheel': c.ignore_wheel, 'aborts': dict(c.aborts),
                                        'manual_start': c.manual_start, 'mode': c.shot_mode if c.in_shot else c.mode,
                                        'burst': c.burst, 'series_index': c.series_index, 'series_total': c.series_total,
                                        'next_shot': next((shot.label() for i, shot in (*c.extra, *c.plan) if i not in c.done[c.speed_band]), 'done'),
                                        'queue': c.queue_left()}))
        self.last_status = now
      return result
    except Exception:
      c.broken = True
      # No logging or I/O on the controls thread, including fault paths.
      if c.in_shot:
        c.finish(now, Inputs(angle=cs.steeringAngleDeg), 'runtime fault')
      return normal
