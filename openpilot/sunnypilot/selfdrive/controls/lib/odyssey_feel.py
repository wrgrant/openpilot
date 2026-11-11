"""Feel test: which lane wander and which steering bumps the driver notices, and which he finds annoying.

Selected as experiment `feel` (OdysseyABExperiment). It runs blind trials from a balanced, shuffled plan: about 20 s of
stimulus, then at least 10 s of rest while the panel asks for a rating (0 didn't notice ... 4 bad). Two families:
  wander  a slow sideways sway added to the model's desired curvature as feedforward lateral accel. Open loop: the
          model's own centring fights it, so tools/odyssey_feel.py scores the measured lane position, not this command.
          Steady state is y = A sin(wt), i.e. a = -A w^2 sin(wt); it fades in and out over FADE_S so the van never
          starts with a sideways velocity step (a bare sine from rest drifts at A*w, 0.37 m/s at 0.35 m and 6 s).
  bumps   three torque blips added to lane keeping's command, 5-7 s apart, alternating direction.
Zero-size trials are shams. Trials pause (and a running one aborts) when the driver steers, the blinker is on, lateral
is off, below 25 mph, or the lane lines fade. Every start, abort, rating and "felt that" tap is an `odyssey_feel`
cloudlog record (a logMessage in the rlog). Progress (which trials are rated) goes to a file, so a process restart
resumes the same plan.

No new param keys (the test deploys by a Python-only controls restart): the panel sends ratings and "felt that" taps
through an append-only file in /dev/shm (post_input / InputReader), pause is the existing OdysseyPulsePaused, and the
status goes to the existing memory param OdysseyABStatus. The status never says what the stimulus was.

Positive accel/curvature is to the left (openpilot curvature); positive torque follows actuators.torque.
"""
from dataclasses import dataclass, asdict
import functools
import json
import math
import os
import random
import tempfile

STIM_S, REST_S, RATE_TIMEOUT_S, SETTLE_S = 20., 10., 30., 3.  # s
FADE_S = 4.  # s raised-cosine fade at each end of a wander trial
MAX_ACCEL = .4  # m/s^2: cap on the wander's extra lateral accel
MAX_TORQUE = .10  # normalized: cap on a bump's extra torque
MIN_SPEED = 25 * .44704
MIN_LANE_PROB = .5
PRESS_S = .3  # a press must last this long to abort a running trial: shorter ones are torque-sensor blips (ODYSSEY.md)
MAX_MODEL_AGE = .2
MAX_MISSES = 3  # aborts or unrated timeouts before a trial is skipped for this session
BLOCKS = 2  # each block holds every condition once: 20 trials, about 10 minutes; two blocks fit a 20-minute session
FRAME_LOG_S = .1  # stimulus frame records, 10 Hz
VERSION = 1

AMPLITUDES = (0., .08, .15, .25, .35)  # m, wander
PERIODS = (6., 12.)  # s, wander
SIZES = (0., .02, .04, .06, .08)  # normalized torque, bumps
WIDTHS = (.1, .3)  # s, bumps
BLIPS, FIRST_BLIP, BLIP_GAP = 3, (2., 4.), (5., 7.)  # s

RATINGS = ("didn't notice", "noticed", "mildly annoying", "annoying", "bad")

_SHM = '/dev/shm' if os.path.isdir('/dev/shm') else tempfile.gettempdir()
INPUT_PATH = os.path.join(_SHM, 'odyssey_feel_input')
PROGRESS_PATH = '/data/odyssey_feel_progress.json' if os.path.isdir('/data/params') else os.path.join(_SHM, 'odyssey_feel_progress.json')


@dataclass(frozen=True)
class Stimulus:
  family: str  # 'wander' or 'bump'
  amplitude: float = 0.  # m (wander)
  period: float = 0.  # s (wander)
  size: float = 0.  # normalized torque (bump)
  width: float = 0.  # s (bump)
  direction: int = 1  # sign of the first push
  blips: tuple = ()  # s from trial start (bump)

  @property
  def sham(self):
    return (self.amplitude if self.family == 'wander' else self.size) == 0.

  @property
  def scale(self):
    """Wander only: the share of the amplitude that keeps the whole profile under MAX_ACCEL. Scaling, not clipping,
    keeps the sway symmetric, so the van still ends where it started (0.35 m at 6 s runs at about 0.82)."""
    return _wander_scale(self.amplitude, self.period) if self.family == 'wander' and not self.sham else 1.

  def accel(self, t):
    """Wander's extra lateral accel at t s into the trial: d2/dt2 of A e(t) sin(wt), e fading in and out."""
    if self.family != 'wander' or self.sham or not 0 <= t < STIM_S:
      return 0.
    return max(-MAX_ACCEL, min(MAX_ACCEL, self.direction * self.scale * _wander_shape(self.amplitude, self.period, t)))

  def torque(self, t):
    if self.family != 'bump' or self.sham:
      return 0.
    for k, start in enumerate(self.blips):
      if start <= t < start + self.width:
        return max(-MAX_TORQUE, min(MAX_TORQUE, self.size * self.direction * (1 if k % 2 == 0 else -1)))
    return 0.


def _wander_shape(amplitude, period, t):
  w = 2 * math.pi / period

  def fade(s):  # value, first and second derivative of the raised-cosine ramp, s seconds from an end
    if s >= FADE_S:
      return 1., 0., 0.
    k = math.pi / FADE_S
    return .5 * (1 - math.cos(k * s)), .5 * k * math.sin(k * s), .5 * k * k * math.cos(k * s)
  f, f1, f2 = fade(t)
  g, g1, g2 = fade(STIM_S - t)
  e, e1, e2 = f * g, f1 * g - f * g1, f2 * g - 2 * f1 * g1 + f * g2  # g's derivative w.r.t. t carries a minus sign
  s, s1, s2 = math.sin(w * t), w * math.cos(w * t), -w * w * math.sin(w * t)
  return amplitude * (e2 * s + 2 * e1 * s1 + e * s2)


@functools.lru_cache(maxsize=64)
def _wander_scale(amplitude, period):
  peak = max(abs(_wander_shape(amplitude, period, n / 100)) for n in range(round(STIM_S * 100)))
  return min(1., MAX_ACCEL / peak) if peak > 0 else 1.


def conditions():
  """The 20 conditions of one block, by name: every amplitude x period, every size x width; zero is the sham."""
  out = [(f'wander-{a:g}-{p:g}', 'wander', a, p) for a in AMPLITUDES for p in PERIODS]
  return out + [(f'bump-{s:g}-{w:g}', 'bump', s, w) for s in SIZES for w in WIDTHS]


def plan(seed, blocks=BLOCKS):
  """Balanced and shuffled: each block runs every condition once in its own random order, with a random first
  direction and, for bumps, random blip times. Seeded, so a restart rebuilds the same plan from the progress file."""
  out = []
  for b in range(blocks):
    rng = random.Random(f'{seed}:{b}')
    block = conditions()
    rng.shuffle(block)
    for name, family, x, y in block:
      direction = rng.choice((1, -1))
      if family == 'wander':
        out.append((f'{b}:{name}', Stimulus('wander', amplitude=x, period=y, direction=direction)))
      else:
        t = [round(rng.uniform(*FIRST_BLIP), 2)]
        for _ in range(BLIPS - 1):
          t.append(round(t[-1] + rng.uniform(*BLIP_GAP), 2))
        out.append((f'{b}:{name}', Stimulus('bump', size=x, width=y, direction=direction, blips=tuple(t))))
  return tuple(out)


@dataclass(frozen=True)
class Inputs:
  active: bool = True
  speed: float = 25.
  pressed: bool = False
  blinker: bool = False
  lane_change: bool = False
  brake: bool = False
  lane_prob: float = 1.
  model_age: float = 0.
  valid: bool = True
  maneuver: bool = False


def parse_input(line):
  """One panel tap: {"kind": "rate", "value": 0-4, "tap_s": t} or {"kind": "felt", "tap_s": t}; None if malformed."""
  try:
    d = json.loads(line)
    tap = float(d['tap_s'])
    if not math.isfinite(tap) or d.get('kind') not in ('rate', 'felt'):
      return None
    if d['kind'] == 'felt':
      return ('felt', None, tap)
    value = d.get('value')
    if type(value) is not int or not 0 <= value < len(RATINGS):
      return None
    return ('rate', value, tap)
  except (ValueError, TypeError, KeyError, AttributeError):
    return None


def post_input(kind, value=None, now=None, path=None):
  """Called by the panel on a tap. One short O_APPEND write, so concurrent taps never interleave."""
  import time
  record = {'kind': kind, 'tap_s': time.monotonic() if now is None else now}
  if kind == 'rate':
    record['value'] = int(value)
  fd = os.open(path or INPUT_PATH, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o666)
  try:
    os.write(fd, (json.dumps(record) + '\n').encode())
  finally:
    os.close(fd)


class InputReader:
  """Reads taps appended since it started (taps from before a restart mean nothing now). A shrunk file starts over."""
  def __init__(self, path=None):
    self.path = path or INPUT_PATH
    try:
      self.offset = os.path.getsize(self.path)
    except OSError:
      self.offset = 0
    self.partial = b''

  def read(self):
    try:
      size = os.path.getsize(self.path)
    except OSError:
      return []
    if size < self.offset:
      self.offset, self.partial = 0, b''
    if size == self.offset:
      return []
    with open(self.path, 'rb') as f:
      f.seek(self.offset)
      data = f.read(min(size - self.offset, 1 << 16))
    self.offset += len(data)
    data = self.partial + data
    lines = data.split(b'\n')
    self.partial = lines.pop()
    return [x for x in (parse_input(line.decode(errors='replace')) for line in lines if line.strip()) if x is not None]


class Feel:
  """Pure 100 Hz state machine. step() returns (extra lateral accel m/s^2, extra torque); the caller drains events.
  States: wait (for a clear road), stimulus, rate (asking), rest (rated, letting the van settle), done."""
  def __init__(self, progress=None, seed=None):
    progress = progress if isinstance(progress, dict) and progress.get('version') == VERSION else {}
    self.seed = progress.get('seed') if isinstance(progress.get('seed'), int) else (
      seed if seed is not None else random.SystemRandom().randrange(1 << 30))
    self.plan = plan(self.seed)
    for _, stim in self.plan:
      stim.scale  # noqa: B018  warm the cache here, on the worker, not on the controls thread at a trial's start
    ids = {i for i, _ in self.plan}
    self.done = {i for i in progress.get('done', []) if i in ids} if isinstance(progress.get('done'), list) else set()
    self.trials = progress.get('trials') if isinstance(progress.get('trials'), int) and progress.get('trials') >= 0 else 0
    self.skipped, self.misses = set(), {}
    self.state, self.reason = 'wait', 'waiting'
    self.trial = None  # (number, id, Stimulus) while a stimulus runs
    self.start = self.end = 0.
    self.last_trial = None  # the last completed stimulus: (number, id, Stimulus); ratings apply to it
    self.rating = None
    self.clear_since = self.pressed_since = None
    self.events = []
    self.felt = 0
    self.dirty = False  # progress changed: the runtime writes it out

  def progress(self):
    return {'version': VERSION, 'seed': self.seed, 'done': sorted(self.done), 'trials': self.trials}

  def remaining(self):
    return [(n, (i, s)) for n, (i, s) in enumerate(self.plan) if i not in self.done and i not in self.skipped]

  def gate(self, now, x, running):
    if not x.valid or not all(math.isfinite(v) for v in (x.speed, x.lane_prob, x.model_age)):
      return 'invalid data'
    if not x.active:
      return 'steering inactive'
    if x.maneuver:
      return 'maneuver mode'
    if x.model_age < 0 or x.model_age > MAX_MODEL_AGE:
      return 'stale model'
    if x.blinker:
      return 'blinker'
    if x.lane_change:
      return 'lane change'
    if x.brake:
      return 'brake'
    if x.speed < MIN_SPEED:
      return 'under 25 mph'
    if x.lane_prob < MIN_LANE_PROB:
      return 'lane lines lost'
    if x.pressed and (not running or now - self.pressed_since >= PRESS_S):
      return "you're steering"
    return ''

  def _context(self, now):
    ctx = {'mono_s': now, 'state': self.state}
    if self.trial is not None:
      ctx.update(trial=self.trial[0], id=self.trial[1], elapsed=round(now - self.start, 3))
    elif self.last_trial is not None:
      ctx.update(last_trial=self.last_trial[0], since_end=round(now - self.end, 3))
    return ctx

  def _miss(self, trial_id, now):
    self.misses[trial_id] = self.misses.get(trial_id, 0) + 1
    if self.misses[trial_id] >= MAX_MISSES:
      self.skipped.add(trial_id)
      self.events.append({'phase': 'skip', 'id': trial_id, 'mono_s': now, 'misses': self.misses[trial_id]})

  def tap(self, now, kind, value, tap_s):
    """A panel tap. Ratings apply to the last completed stimulus until the next one starts (a later tap corrects an
    earlier one). A rating during a stimulus or after an abort is logged and ignored: the trial is not over yet."""
    if kind == 'felt':
      self.felt += 1
      self.events.append({'phase': 'felt', 'tap_s': tap_s, **self._context(now)})
      return
    if self.state == 'stimulus' or self.last_trial is None:
      self.events.append({'phase': 'stray_rating', 'value': value, 'tap_s': tap_s, **self._context(now)})
      return
    number, trial_id, stim = self.last_trial
    self.events.append({'phase': 'rating', 'trial': number, 'id': trial_id, 'value': value, 'label': RATINGS[value],
                        'tap_s': tap_s, 'since_end': round(now - self.end, 3), 'mono_s': now, 'correction': self.rating is not None})
    self.rating = value
    if trial_id not in self.done:
      self.done.add(trial_id)
      self.skipped.discard(trial_id)
      self.dirty = True
    if self.state == 'rate':
      self.state = 'rest'

  def step(self, now, x, selected, paused, taps=()):
    for kind, value, tap_s in taps:
      self.tap(now, kind, value, tap_s)
    self.pressed_since = (now if self.pressed_since is None else self.pressed_since) if x.pressed else None
    reason = 'not selected' if not selected else 'paused' if paused else self.gate(now, x, self.state == 'stimulus')
    self.clear_since = None if reason else (now if self.clear_since is None else self.clear_since)

    if self.state == 'stimulus':
      if reason:
        number, trial_id, stim = self.trial
        self.events.append({'phase': 'abort', 'trial': number, 'id': trial_id, 'reason': reason,
                            'elapsed': round(now - self.start, 3), 'mono_s': now, 'stimulus': asdict(stim)})
        self._miss(trial_id, now)
        self.trial, self.state, self.reason = None, 'wait', reason
        return 0., 0.  # aborts hand the road straight back
      t = now - self.start
      if t < STIM_S:
        stim = self.trial[2]
        return stim.accel(t), stim.torque(t)
      number, trial_id, stim = self.trial
      self.events.append({'phase': 'end', 'trial': number, 'id': trial_id, 'mono_s': now})
      self.last_trial, self.trial, self.end, self.rating = self.trial, None, now, None
      self.state = 'rate'

    if self.state == 'rate' and now - self.end >= RATE_TIMEOUT_S:
      number, trial_id, _ = self.last_trial
      self.events.append({'phase': 'unrated', 'trial': number, 'id': trial_id, 'mono_s': now})
      self._miss(trial_id, now)
      self.state = 'wait'
    if self.state == 'rest' and now - self.end >= REST_S:
      self.state = 'wait'
    if self.state in ('rate', 'rest'):
      self.reason = 'rate it' if self.state == 'rate' else 'rest'
      return 0., 0.

    left = self.remaining()
    if not left:
      self.state, self.reason = 'done', 'done'
      return 0., 0.
    self.state = 'wait'
    if reason:
      self.reason = reason
      return 0., 0.
    if now - self.clear_since < SETTLE_S:
      self.reason = 'settling'
      return 0., 0.
    _, (trial_id, stim) = min(left, key=lambda item: (self.misses.get(item[1][0], 0), item[0]))
    self.trials += 1
    self.dirty = True
    self.trial, self.start, self.last_trial, self.rating = (self.trials, trial_id, stim), now, None, None
    self.events.append({'phase': 'start', 'trial': self.trials, 'id': trial_id, 'mono_s': now, 'speed': round(x.speed, 2),
                        'lane_prob': round(x.lane_prob, 3), 'stimulus': asdict(stim), 'scale': round(stim.scale, 4), 'stim_s': STIM_S})
    self.state, self.reason = 'stimulus', 'running'
    return stim.accel(0.), stim.torque(0.)

  def status(self):
    """For the panel. Blind: the trial number and state, never what the stimulus is."""
    number = self.trial[0] if self.trial else self.last_trial[0] if self.last_trial else self.trials + 1
    return {'experiment': 'feel', 'state': self.state, 'reason': self.reason, 'trial': number,
            'rated': len(self.done), 'total': len(self.plan), 'rating': self.rating if self.state != 'stimulus' else None,
            'felt': self.felt}


class FeelRuntime:
  """Params, file I/O and logging run on a worker; the controls thread only computes. Any fault zeroes the stimulus."""
  def __init__(self, params, input_path=None, progress_path=None):
    import collections
    import queue
    import threading
    import time
    self.params = params
    self.input_path, self.progress_path = input_path or INPUT_PATH, progress_path or PROGRESS_PATH
    self.controller = None  # built by the worker from the progress file
    self.config = (False, True)  # selected, paused
    self.inbox = collections.deque(maxlen=64)
    self.queue = queue.Queue(maxsize=1024)
    self.heartbeat = time.monotonic()
    self.broken = False
    self.last_status = self.last_frame = -1.
    self.stop = threading.Event()
    self.worker = threading.Thread(target=self._worker, daemon=True, name='odyssey-feel')
    self.worker.start()

  def close(self):
    self.stop.set()
    self.worker.join(timeout=.5)

  def _write_progress(self, data):
    tmp = self.progress_path + '.tmp'
    with open(tmp, 'w') as f:
      json.dump(data, f)
    os.replace(tmp, self.progress_path)

  def _worker(self):
    import queue
    import time
    from openpilot.common.swaglog import cloudlog
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import mem_params
    try:
      mem = mem_params(self.params)
      try:
        with open(self.progress_path) as f:
          progress = json.load(f)
      except (OSError, ValueError):
        progress = None
      controller = Feel(progress)
      if not os.environ.get('REPLAY') and self.params.get('OdysseyABExperiment') == 'feel':
        self.params.put_bool('OdysseyPulsePaused', True)  # every start is paused: the driver taps the panel to begin
      reader = InputReader(self.input_path)
      self.controller = controller
      last_poll = -1.
      while not self.stop.is_set():
        now = time.monotonic()
        self.heartbeat = now
        if now - last_poll >= .05:
          last_poll = now
          self.config = (self.params.get('OdysseyABExperiment') == 'feel', self.params.get_bool('OdysseyPulsePaused'))
          for tap in reader.read():
            self.inbox.append(tap)
        try:
          kind, data = self.queue.get(timeout=.02)
        except queue.Empty:
          continue
        if kind == 'event':
          cloudlog.event('odyssey_feel', **data)
        elif kind == 'progress':
          try:
            self._write_progress(data)
          except OSError:
            cloudlog.exception('odyssey_feel progress not saved')  # the log still has every rating
        else:
          mem.put('OdysseyABStatus', json.dumps(data))
    except Exception:
      self.config = (False, True)
      cloudlog.exception('odyssey_feel worker disabled')

  def update(self, now, cs, md, active, valid, model_age, maneuver):
    """Returns (curvature to add to the desired curvature, torque to add to the command). Zero unless a trial runs."""
    import time
    c = self.controller
    if c is None or self.broken:
      return 0., 0.
    try:
      selected, paused = self.config
      if not selected and c.state != 'stimulus' and not self.inbox:
        return 0., 0.
      if time.monotonic() - self.heartbeat > .5:
        selected = False  # no worker means no logging and no stimulus
      taps = [self.inbox.popleft() for _ in range(len(self.inbox))]
      probs = md.laneLineProbs
      x = Inputs(active=bool(active), speed=float(cs.vEgo), pressed=bool(cs.steeringPressed),
                 blinker=bool(cs.leftBlinker or cs.rightBlinker), lane_change=str(md.meta.laneChangeState) not in ('off', '0'),
                 brake=bool(cs.brakePressed), lane_prob=float(min(probs[1], probs[2])) if len(probs) == 4 else 0.,
                 model_age=float(model_age), valid=bool(valid), maneuver=bool(maneuver))
      accel, torque = c.step(now, x, selected, paused, taps)
      if not (math.isfinite(accel) and math.isfinite(torque)):
        raise ValueError('non-finite stimulus')
      while c.events:
        self.queue.put_nowait(('event', c.events.pop(0)))
      if c.dirty:
        c.dirty = False
        self.queue.put_nowait(('progress', c.progress()))
      if c.state == 'stimulus' and now - self.last_frame >= FRAME_LOG_S:
        self.queue.put_nowait(('event', {'phase': 'frame', 'trial': c.trial[0], 'mono_s': now, 'accel': round(accel, 4),
                                         'torque': round(torque, 4), 'speed': round(x.speed, 2)}))
        self.last_frame = now
      if selected and now - self.last_status >= .1:
        self.queue.put_nowait(('status', {**c.status(), 'mono_s': now, 'paused': paused}))
        self.last_status = now
      return accel / max(x.speed, MIN_SPEED) ** 2, torque
    except Exception:
      self.broken = True  # no logging or I/O on the controls thread, including fault paths
      return 0., 0.
